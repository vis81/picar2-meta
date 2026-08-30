#!/usr/bin/env python3
"""
Phone-facing web UI for PICAR-2 map building.

Owns the mode stack above bringup (which runs as a boot service):

    cartographer  →  nav2  →  explore_lite

explore_lite drives by sending NavigateToPose goals and reads Nav2's global
costmap, so autonomous mapping needs all three. Starting a mode is process
control, not ROS messaging, which is why this is a custom backend rather than
rosbridge.

The map is served as a raw byte grid and coloured in the browser — no
server-side image encoding, so no PIL/OpenCV dependency.

Usage:
    server.py [--port 8080] [--ws PATH]

Env vars are used if flags aren't given:
    PICAR_WEBUI_PORT, PICAR_WS
"""
from __future__ import annotations

import argparse
import math
import os
import shutil
import signal
import subprocess
import sys
import threading
import time

import rclpy
import numpy as np
from flask import Flask, Response, jsonify, request, send_from_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from waitress import serve

# Commands older than this are treated as released. The controller's own
# reference_timeout (0.5 s) is the backstop if this process dies outright.
DRIVE_TIMEOUT_S = 0.4
DRIVE_RATE_HZ = 20.0

# Speed caps for the touch joystick — well under the hardware limits, since
# this is for nudging during mapping, not driving fast.
MAX_LINEAR = 0.25
MAX_ANGULAR = 0.8


class RobotLink(Node):
    """ROS side: map in, pose in, cmd_vel out, cartographer services."""

    def __init__(self):
        super().__init__("picar_webui")

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, "/map", self._on_map, map_qos)

        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._lock = threading.Lock()
        self._map: OccupancyGrid | None = None
        self._map_seq = 0
        self._drive = (0.0, 0.0)
        self._drive_stamp = 0.0
        self._drive_was_active = False

        self.create_timer(1.0 / DRIVE_RATE_HZ, self._drive_tick)

    # ── map ──────────────────────────────────────────────────────────────
    def _on_map(self, msg: OccupancyGrid):
        with self._lock:
            self._map = msg
            self._map_seq += 1

    def map_snapshot(self):
        with self._lock:
            return self._map, self._map_seq

    # ── pose ─────────────────────────────────────────────────────────────
    def pose(self):
        """Robot pose in the map frame, or None before the map exists."""
        try:
            tf = self._tf_buffer.lookup_transform(
                "map", "base_footprint", Time()
            )
        except Exception:
            return None

        t = tf.transform.translation
        q = tf.transform.rotation
        # yaw only — two_d_mode means roll/pitch are held at zero
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        return {"x": t.x, "y": t.y, "yaw": yaw}

    # ── drive ────────────────────────────────────────────────────────────
    def set_drive(self, linear: float, angular: float):
        with self._lock:
            self._drive = (
                clamp(linear, -MAX_LINEAR, MAX_LINEAR),
                clamp(angular, -MAX_ANGULAR, MAX_ANGULAR),
            )
            self._drive_stamp = time.monotonic()

    def _drive_tick(self):
        with self._lock:
            linear, angular = self._drive
            fresh = (time.monotonic() - self._drive_stamp) < DRIVE_TIMEOUT_S

        if fresh:
            msg = Twist()
            msg.linear.x = linear
            msg.angular.z = angular
            self._cmd_pub.publish(msg)
            self._drive_was_active = True
        elif self._drive_was_active:
            # One explicit zero on release, then go quiet and let the
            # controller's reference_timeout hold the stop.
            self._cmd_pub.publish(Twist())
            self._drive_was_active = False


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class ModeStack:
    """Starts and stops the cartographer → nav2 → explore chain."""

    LAYERS = ("cartographer", "nav2", "explore")

    def __init__(self):
        self._procs: dict[str, subprocess.Popen] = {}
        self._lock = threading.Lock()

    def _launch(self, name: str, launch_file: str):
        cmd = ["ros2", "launch", "picar2_bringup", launch_file]
        self._procs[name] = subprocess.Popen(
            cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,   # so we can kill the whole launch tree
        )

    def running(self, name: str) -> bool:
        p = self._procs.get(name)
        return p is not None and p.poll() is None

    def status(self) -> dict:
        return {name: self.running(name) for name in self.LAYERS}

    def start_mapping(self, autonomous: bool):
        with self._lock:
            if not self.running("cartographer"):
                self._launch("cartographer", "cartographer.launch.py")
                # Nav2's costmaps need /map to exist before they settle.
                time.sleep(5.0)
            if not self.running("nav2"):
                self._launch("nav2", "nav2.launch.py")
                time.sleep(3.0)
            if autonomous and not self.running("explore"):
                self._launch("explore", "explore.launch.py")

    def set_explore(self, on: bool):
        with self._lock:
            if on and not self.running("explore"):
                self._launch("explore", "explore.launch.py")
            elif not on:
                self._stop("explore")

    def stop_all(self):
        with self._lock:
            for name in reversed(self.LAYERS):
                self._stop(name)

    def _stop(self, name: str):
        p = self._procs.pop(name, None)
        if p is None or p.poll() is not None:
            return
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGINT)
            p.wait(timeout=10)
        except Exception:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGKILL)
            except Exception:
                pass


def save_map(ws: str, name: str) -> tuple[bool, str]:
    """Seal the trajectory, write the pbstream, then the PGM/YAML pair.

    finish_trajectory ends mapping — cartographer cannot resume afterwards,
    which is why the UI calls this 'Finish & save'.
    """
    maps = os.path.join(ws, "maps")
    os.makedirs(maps, exist_ok=True)
    base = os.path.join(maps, name)

    steps = [
        (
            ["ros2", "service", "call", "/finish_trajectory",
             "cartographer_ros_msgs/srv/FinishTrajectory",
             "{trajectory_id: 0}"],
            "finish_trajectory",
        ),
        (
            ["ros2", "service", "call", "/write_state",
             "cartographer_ros_msgs/srv/WriteState",
             f"{{filename: '{base}.pbstream', include_unfinished_submaps: false}}"],
            "write_state",
        ),
        (
            ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", base,
             "--ros-args", "-p", "map_subscribe_transient_local:=true"],
            "map_saver",
        ),
    ]
    for cmd, label in steps:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
        if r.returncode != 0:
            return False, f"{label} failed: {r.stderr.strip()[:200]}"
    return True, base


def list_maps(ws: str) -> list[dict]:
    maps = os.path.join(ws, "maps")
    if not os.path.isdir(maps):
        return []
    out = []
    for f in sorted(os.listdir(maps)):
        if f.endswith(".pbstream"):
            name = f[: -len(".pbstream")]
            path = os.path.join(maps, f)
            out.append({
                "name": name,
                "size": os.path.getsize(path),
                "mtime": int(os.path.getmtime(path)),
                "has_grid": os.path.exists(os.path.join(maps, name + ".yaml")),
            })
    return out


def build_app(link: RobotLink, modes: ModeStack, ws: str, root: str) -> Flask:
    app = Flask(__name__, static_folder=None)

    @app.route("/")
    def index():
        return send_from_directory(root, "index.html")

    @app.route("/<path:path>")
    def static_files(path):
        return send_from_directory(root, path)

    @app.route("/api/status")
    def status():
        grid, seq = link.map_snapshot()
        return jsonify({
            "modes": modes.status(),
            "pose": link.pose(),
            "map_seq": seq,
            "has_map": grid is not None,
            "maps": list_maps(ws),
        })

    @app.route("/api/map")
    def get_map():
        grid, seq = link.map_snapshot()
        if grid is None:
            return ("no map yet", 404)

        info = grid.info
        # int8 occupancy (-1 unknown, 0..100) → unsigned bytes, 255 = unknown.
        # numpy, not a comprehension: a whole-house grid is ~500k cells and a
        # Python loop over it costs ~100 ms per fetch.
        cells = np.frombuffer(bytes(grid.data), dtype=np.int8)
        body = np.where(cells < 0, 255, cells).astype(np.uint8).tobytes()
        resp = Response(body, mimetype="application/octet-stream")
        resp.headers["X-Map-Seq"] = str(seq)
        resp.headers["X-Map-Width"] = str(info.width)
        resp.headers["X-Map-Height"] = str(info.height)
        resp.headers["X-Map-Resolution"] = str(info.resolution)
        resp.headers["X-Map-Origin-X"] = str(info.origin.position.x)
        resp.headers["X-Map-Origin-Y"] = str(info.origin.position.y)
        return resp

    @app.route("/api/mapping/start", methods=["POST"])
    def mapping_start():
        auto = bool((request.json or {}).get("autonomous", True))
        threading.Thread(
            target=modes.start_mapping, args=(auto,), daemon=True
        ).start()
        return jsonify({"ok": True})

    @app.route("/api/mapping/stop", methods=["POST"])
    def mapping_stop():
        modes.stop_all()
        return jsonify({"ok": True})

    @app.route("/api/explore", methods=["POST"])
    def explore():
        on = bool((request.json or {}).get("on", False))
        modes.set_explore(on)
        return jsonify({"ok": True, "exploring": on})

    @app.route("/api/mapping/save", methods=["POST"])
    def mapping_save():
        name = (request.json or {}).get("name", "").strip()
        if not name or "/" in name or name.startswith("."):
            return jsonify({"ok": False, "error": "bad map name"}), 400

        # Stop exploring first so the robot is still while the graph is sealed.
        modes.set_explore(False)
        ok, detail = save_map(ws, name)
        return jsonify({"ok": ok, "detail": detail}), (200 if ok else 500)

    @app.route("/api/drive", methods=["POST"])
    def drive():
        body = request.json or {}
        link.set_drive(float(body.get("linear", 0.0)),
                       float(body.get("angular", 0.0)))
        return ("", 204)

    return app


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", type=int,
                    default=int(os.environ.get("PICAR_WEBUI_PORT", "8080")))
    ap.add_argument("--ws", default=os.environ.get("PICAR_WS", "/ws"))
    ap.add_argument("--root", default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "public"))
    ap.add_argument("--bind", default="0.0.0.0")
    return ap.parse_args()


def main() -> int:
    args = parse_args()

    if not os.path.isdir(args.root):
        print(f"web root not found: {args.root}", file=sys.stderr)
        return 1
    if shutil.which("ros2") is None:
        print("ros2 not on PATH — source the workspace first", file=sys.stderr)
        return 1

    rclpy.init()
    link = RobotLink()
    modes = ModeStack()

    spin = threading.Thread(target=rclpy.spin, args=(link,), daemon=True)
    spin.start()

    app = build_app(link, modes, args.ws, args.root)
    print(f"picar web UI on http://{args.bind}:{args.port}  (ws={args.ws})")
    try:
        serve(app, host=args.bind, port=args.port, threads=8)
    finally:
        modes.stop_all()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
