#!/usr/bin/env python3
"""Drive N laps of the saved route at a top speed, with a bag, and sum it up.

    scripts/run_laps.py --speed 1.5 --laps 3 --label la12
    scripts/run_laps.py --summary bags/20260914-091932_f1b_150_3laps_la12

Talks to the web UI's API on the robot (the same buttons a phone presses):
sets the top speed, starts a bag named `<stamp>_<map>_<speed>_<label>`,
starts the route in loop+flow mode, counts passes of the first waypoint
until N laps are done, stops the route and the bag. Then, unless
--no-fetch, copies the bag into ./bags/, verifies it, deletes it on the
robot and prints the numbers the session always ended up computing by
hand: lap times, top/mean speed, collision-ahead stops, recoveries, the
Pi's CPU and the pack's lowest voltage.

The summary needs a sourced ROS 2 environment (rosbag2_py); everything
else is plain Python.
"""

import argparse
import json
import os
import subprocess
import sys
import time
import urllib.request

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def api(host, path, body=None, timeout=10):
    req = urllib.request.Request(
        f"http://{host}/api{path}",
        data=json.dumps(body).encode() if body is not None else None,
        headers={"Content-Type": "application/json"},
        method="POST" if body is not None else "GET")
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.load(r)


def run(args):
    host = f"{args.host}:{args.port}"
    s = api(host, "/status")
    if s.get("mode") != "localize" or not s.get("pose"):
        sys.exit(f"robot is not localized (mode={s.get('mode')}, pose={s.get('pose')})")
    if s.get("route", {}).get("active"):
        sys.exit("a route is already running")
    label = f"{(s.get('map_name') or 'map').lower()}_{int(round(args.speed * 100)):03d}_{args.laps}laps"
    if args.label:
        label += "_" + args.label
    print("speed:", api(host, "/speed", {"value": args.speed}, 20))
    b = api(host, "/bag", {"on": True, "label": label})
    name = b["bag"]["name"]
    print("bag:", name)
    time.sleep(2)
    print("route:", api(host, "/route/start", {"loop": True, "flow": True}))
    t0 = time.time()
    prev = None
    vmin = 99.0
    passes = []
    try:
        # 25 s/lap assumed real lap time tracks the requested --speed. Not
        # true for a controller whose own speed regulation (e.g. VP's
        # max_lateral_accel) dominates the route's curvy sections regardless
        # of the ceiling: measured 57.4 s/lap at --speed 0.4 AND 56.8 s/lap at
        # --speed 0.8, same route, same controller (2026-09-19, VP #22
        # real-hardware test) - scaling the budget by 1/speed made the second
        # case *worse*, cutting a 3-lap run to 2. 80 s/lap flat, based on the
        # slowest measurement plus margin, regardless of --speed: the early
        # break below still ends things the moment the actual laps are done,
        # so an over-generous budget here costs nothing, while an
        # under-generous one silently truncates the test.
        while time.time() - t0 < args.laps * 80 + 90:
            st = api(host, "/status")
            r = st["route"]
            idx = r.get("index")
            v = (st.get("battery") or {}).get("volts")
            if v:
                vmin = min(vmin, v)
            if idx == 0 and prev not in (None, 0):
                passes.append(time.time() - t0)
                print(f"{passes[-1]:6.1f}s  wp0 pass #{len(passes)}  nav={st['nav']['state']}  "
                      f"v={v}  cpu={(st.get('cpu') or {}).get('pct')}%")
                if len(passes) > args.laps:
                    break
            prev = idx
            if not r["active"]:
                print("route ended by itself:", r.get("error") or st["nav"])
                break
            time.sleep(0.3)
    finally:
        api(host, "/route/stop", {})
        # /route/stop and /bag off have both reported success once while the
        # robot kept driving and the bag kept recording for another 12
        # minutes (2026-09-19) - only actually stopping when the web UI
        # process itself was restarted. Verify instead of trusting the first
        # reply: poll for the pose to actually settle and the route to
        # report inactive, retrying the stop if not, before declaring done.
        for attempt in range(5):
            time.sleep(2)
            p1 = api(host, "/status")["pose"]
            time.sleep(2)
            st = api(host, "/status")
            p2 = st["pose"]
            moving = abs(p1["x"] - p2["x"]) > 0.01 or abs(p1["y"] - p2["y"]) > 0.01
            if not moving and not st["route"].get("active"):
                break
            print(f"  stop attempt {attempt}: still moving or route active - retrying")
            api(host, "/route/stop", {})
        else:
            print("  WARNING: robot did not settle after 5 stop attempts")
        api(host, "/bag", {"on": False}, 30)
        # rosbag2 only writes metadata.yaml on a clean shutdown - its
        # presence is proof the recorder process actually exited, not just
        # that the API said so.
        time.sleep(2)
        closed = subprocess.run(
            ["ssh", args.ssh, f"test -f {args.remote_bags}/{name}/metadata.yaml"]
        ).returncode == 0
        print("bag recorder:", "closed" if closed else "STILL RECORDING")
    laps = [round(b_ - a, 1) for a, b_ in zip(passes, passes[1:])]
    if laps:
        print(f"laps: {laps}  mean {sum(laps) / len(laps):.1f}  min {min(laps)}  max {max(laps)}")
    print(f"battery min {vmin:.2f} V")
    if args.no_fetch:
        return
    local = fetch(args, name)
    if local:
        summary(local)


def fetch(args, name):
    """rsync the bag into ./bags, verify by checksum, delete on the robot."""
    dst = os.path.join(WS, "bags", name)
    src = f"{args.ssh}:{args.remote_bags}/{name}/"
    os.makedirs(dst, exist_ok=True)
    if subprocess.call(["rsync", "-a", src, dst + "/"]) != 0:
        print("fetch failed; bag left on the robot")
        return None
    diff = subprocess.run(["rsync", "-rnc", "--out-format=%n", src, dst + "/"],
                          capture_output=True, text=True).stdout
    if any(line and not line.endswith("/") for line in diff.splitlines()):
        print("verify failed; bag left on the robot")
        return None
    subprocess.call(["ssh", args.ssh,
                     f"rm -rf {args.remote_bags}/{name} {args.remote_bags}/{name}.log"])
    print(f"bag -> {os.path.relpath(dst, WS)} (removed on the robot)")
    return dst


def summary(path):
    """The run in numbers, from the bag."""
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError:
        print("(no ROS environment: skipping the bag summary)")
        return
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=""),
                rosbag2_py.ConverterOptions("", ""))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    want = {"/cmd_vel", "/odom", "/rosout", "/behavior_tree_log", "/diagnostics", "/battery"}
    reader.set_filter(rosbag2_py.StorageFilter(topics=[t for t in want if t in types]))
    cmd, odo, cpu, volts = [], [], [], []
    coll = patience = planfail = 0
    recov = []
    while reader.has_next():
        topic, raw, t = reader.read_next()
        m = deserialize_message(raw, types[topic])
        if topic == "/cmd_vel":
            cmd.append(m.linear.x)
        elif topic == "/odom":
            odo.append(m.twist.twist.linear.x)
        elif topic == "/rosout":
            coll += "collision ahead" in m.msg
            patience += "patience exceeded" in m.msg
            planfail += "failed to plan" in m.msg
        elif topic == "/behavior_tree_log":
            recov += [e.node_name for e in m.event_log
                      if e.node_name in ("BackUp", "Wait") and e.current_status == "RUNNING"]
        elif topic == "/diagnostics":
            for st in m.status:
                if st.name == "cpu":
                    kv = {v.key: v.value for v in st.values}
                    cpu.append(float(kv.get("total_pct", "nan")))
        elif topic == "/battery":
            volts.append(m.voltage)
    if not cmd:
        print("(empty bag)")
        return
    moving = [v for v in cmd if v > 0.05]
    print(f"commanded: max {max(cmd):.2f}  mean(moving) {sum(moving) / max(len(moving), 1):.2f} m/s"
          f"  reverse samples {sum(1 for v in cmd if v < -0.01)}")
    if odo:
        o = sorted(odo)
        print(f"measured:  max {o[-1]:.2f}  p99 {o[int(0.99 * (len(o) - 1))]:.2f} m/s")
    print(f"collision-ahead {coll}  patience {patience}  planner failures {planfail}  recoveries {recov}")
    if cpu:
        print(f"Pi CPU: mean {sum(cpu) / len(cpu):.0f}%  max {max(cpu):.0f}%")
    if volts:
        print(f"battery: min {min(volts):.2f} V")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--speed", type=float, default=1.0, help="top speed, m/s")
    ap.add_argument("--laps", type=int, default=3)
    ap.add_argument("--label", default="", help="what changed, goes into the bag name")
    ap.add_argument("--host", default=os.environ.get("PI_HOST", "rpi4.local"))
    ap.add_argument("--port", default="8080")
    ap.add_argument("--ssh", default=os.environ.get("PI_SSH", "pi@rpi4.local"))
    ap.add_argument("--remote-bags", default="/home/pi/picar_ws/bags")
    ap.add_argument("--no-fetch", action="store_true", help="leave the bag on the robot")
    ap.add_argument("--summary", metavar="BAG", help="only summarise an existing local bag")
    args = ap.parse_args()
    if args.summary:
        summary(args.summary)
    else:
        run(args)


if __name__ == "__main__":
    main()
