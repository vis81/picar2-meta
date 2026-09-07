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
from explore_lite_msgs.msg import ExploreStatus
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool
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
        # explore_lite searches the global costmap, not /map — and unlike
        # cartographer's output it uses the standard scale (0 free, 100
        # occupied), so it is also the only one worth thresholding.
        self.create_subscription(OccupancyGrid, "/global_costmap/costmap",
                                 self._on_costmap, map_qos)
        self._costmap_free = 0

        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        # explore_lite stops permanently the first time it finds no frontiers.
        # At startup that happens routinely, before the global costmap has
        # copied the map in — resume puts it back to work.
        self._resume_pub = self.create_publisher(Bool, "/explore/resume", 10)
        self.create_subscription(
            ExploreStatus, "/explore/status", self._on_explore_status, 10)
        self.explore_status = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._lock = threading.Lock()
        self._map: OccupancyGrid | None = None
        self._map_seq = 0
        self._drive = (0.0, 0.0)
        self._drive_stamp = 0.0
        self._drive_was_active = False
        self.pose_error = "no lookup yet"
        self._tf_valid_from = 0.0

        self.create_timer(1.0 / DRIVE_RATE_HZ, self._drive_tick)

    def _on_explore_status(self, msg: ExploreStatus):
        self.explore_status = msg.status

    def resume_explore(self):
        self._resume_pub.publish(Bool(data=True))

    def free_cells(self) -> int:
        """Known-free cells in cartographer's map. Note the scale: cartographer
        publishes probabilities, so free space reads ~42 and occupied ~68 —
        not the 0/100 of a map_server grid. Threshold at the 50 midpoint."""
        with self._lock:
            if self._map is None:
                return 0
            cells = np.frombuffer(bytes(self._map.data), dtype=np.int8)
            return int(np.count_nonzero((cells >= 0) & (cells < 50)))

    # ── map ──────────────────────────────────────────────────────────────
    def _on_map(self, msg: OccupancyGrid):
        with self._lock:
            self._map = msg
            self._map_seq += 1

    def map_snapshot(self):
        with self._lock:
            return self._map, self._map_seq

    def forget_map(self):
        """Drop the current grid and costmap count.

        Both are latched state that nothing ever resets, which is harmless
        while the process only ever mounts one stack. Once modes can be
        switched it is not: the readiness gates would pass instantly on the
        previous mode's grid, so a map_server that never loaded would look
        exactly like success. Bump the sequence so the browser refetches
        rather than keeping the old picture on screen.
        """
        with self._lock:
            self._map = None
            self._map_seq += 1
        self._costmap_free = 0
        self.explore_status = None

    def forget_tf(self):
        """Treat transforms older than now as belonging to the previous mode.

        pose() looks up the latest available transform and tf2 keeps ten
        seconds of history, so a dead cartographer's last map→odom stays
        lookup-able well after the process is gone, and anything gating on
        "is there a pose yet" would pass on that ghost.

        This marks a cutoff rather than calling Buffer.clear(), because the
        buffer also holds /tf_static — the robot's own links, published once
        at bringup with transient-local durability. Clearing drops them, and
        an already-subscribed listener gets no replay, so the whole chain
        below odom would be gone for good.
        """
        self._tf_valid_from = time.time()
        self.pose_error = "no lookup yet"

    def _on_costmap(self, msg: OccupancyGrid):
        cells = np.frombuffer(bytes(msg.data), dtype=np.int8)
        self._costmap_free = int(np.count_nonzero((cells >= 0) & (cells <= 25)))

    def costmap_free(self) -> int:
        return self._costmap_free

    def has_map(self) -> bool:
        with self._lock:
            return self._map is not None

    def map_info(self):
        with self._lock:
            if self._map is None:
                return None
            i = self._map.info
            return {
                "width": i.width, "height": i.height,
                "resolution": i.resolution, "cells": len(self._map.data),
            }

    def tf_frames(self) -> str:
        """Whole TF tree as the buffer sees it — shows which link is missing."""
        try:
            return self._tf_buffer.all_frames_as_yaml()
        except Exception as e:
            return f"error: {e}"

    def costmap_ready(self) -> bool:
        """Not just "is it published" — explore_lite needs free space in it to
        search outward from, or its first plan finds no frontiers and stops
        for good."""
        return self._costmap_free > 400

    # ── pose ─────────────────────────────────────────────────────────────
    def pose(self):
        """Robot pose in the map frame, or None before the map exists."""
        try:
            tf = self._tf_buffer.lookup_transform(
                "map", "base_footprint", Time()
            )
            stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
            if stamp < self._tf_valid_from:
                # Left over from the mode we just tore down — see forget_tf.
                self.pose_error = "stale transform from the previous mode"
                return None
            self.pose_error = None
        except Exception as e:
            # Keep the reason — a null pose is the symptom of a broken TF
            # chain, and which exception it is says where the break is.
            self.pose_error = f"{type(e).__name__}: {e}"
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

    # Ordered producers-first, so reversed() is a safe teardown order:
    # explore → nav2 → amcl → cartographer, consumers before what they read.
    # cartographer and amcl are peers — both publish /map and broadcast
    # map→odom, so they must never run at the same time.
    LAYERS = ("cartographer", "amcl", "nav2", "explore")
    LOG_DIR = "/tmp/picar-webui"

    def __init__(self):
        self._procs: dict[str, subprocess.Popen] = {}
        self._lock = threading.Lock()
        # Transitions are serialised, and each carries a generation so a
        # superseded one unwinds instead of racing the new one. A plain
        # threading.Event cannot express "abort the previous transition but
        # not this one" — see _begin/_join.
        self._busy = threading.Lock()
        self._gen = 0
        self._gen_lock = threading.Lock()
        # The mode is separate from the layer set, because layers come up
        # lazily within a mode: "mapping" may or may not have Nav2 yet.
        self.mode = "idle"          # idle | mapping | localize
        self.map_name = None        # localize: which saved map is loaded
        self.phase = "idle"
        os.makedirs(self.LOG_DIR, exist_ok=True)

    # ── transition generations ───────────────────────────────────────────
    def _begin(self) -> int:
        """Claim a transition, superseding any in flight."""
        with self._gen_lock:
            self._gen += 1
            return self._gen

    def _join(self) -> int:
        """Join the current transition without superseding it, so an action
        taken while a mode is still coming up queues behind it."""
        with self._gen_lock:
            return self._gen

    def _current(self, gen: int) -> bool:
        with self._gen_lock:
            return gen == self._gen

    def log_path(self, name: str) -> str:
        return os.path.join(self.LOG_DIR, f"{name}.log")

    def _launch(self, name: str, launch_file: str,
                args: dict[str, str] | None = None):
        # Output goes to a file, never DEVNULL: a launch that dies on startup
        # is the most likely failure here, and discarding stderr makes it
        # invisible from the phone.
        cmd = ["ros2", "launch", "picar2_bringup", launch_file]
        cmd += [f"{k}:={v}" for k, v in (args or {}).items()]
        log = open(self.log_path(name), "wb")
        log.write(f"$ {' '.join(cmd)}\n".encode())
        log.flush()
        self._procs[name] = subprocess.Popen(
            cmd,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,   # so we can kill the whole launch tree
        )

    def running(self, name: str) -> bool:
        p = self._procs.get(name)
        return p is not None and p.poll() is None

    def exit_code(self, name: str):
        p = self._procs.get(name)
        return None if p is None else p.poll()

    def log_tail(self, name: str, lines: int = 40) -> str:
        try:
            with open(self.log_path(name), "r", errors="replace") as f:
                return "".join(f.readlines()[-lines:])
        except OSError:
            return ""

    def status(self) -> dict:
        return {name: self.running(name) for name in self.LAYERS}

    def detail(self) -> dict:
        """Per-layer state including why a layer stopped, for the UI."""
        out = {}
        for name in self.LAYERS:
            code = self.exit_code(name)
            out[name] = {
                "running": self.running(name),
                "exit": code,
                "failed": code is not None and code != 0,
            }
        return out

    def _wait_until(self, pred, timeout: float, phase: str, gen: int) -> bool:
        self.phase = phase
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if not self._current(gen):
                return False
            if pred():
                return True
            time.sleep(0.5)
        return False

    def _teardown(self, names):
        """Stop the named layers, consumers before producers."""
        with self._lock:
            for name in reversed(self.LAYERS):
                if name in names:
                    self._stop(name)

    def set_mode(self, mode: str, link: "RobotLink",
                 map_yaml: str | None = None, map_name: str | None = None):
        """Worker: switch the whole stack to one mode.

        Stop-then-start, never overlapping. Cartographer and AMCL both
        publish /map *and* broadcast map→odom, so even a moment of overlap
        gives odom two parents and a pose that flickers between two
        estimates.
        """
        gen = self._begin()
        with self._busy:
            if not self._current(gen):
                return
            try:
                if mode == "mapping":
                    self._to_mapping(link, gen)
                elif mode == "localize":
                    self._to_localize(link, gen, map_yaml, map_name)
                else:
                    self._to_idle(link)
            except Exception as e:
                # A dead thread leaves the phase frozen on whatever it was
                # doing, which reads exactly like a hang.
                self.phase = f"error: {type(e).__name__}: {e}"

    def _to_idle(self, link: "RobotLink"):
        """Caller holds _busy."""
        self.phase = "stopping"
        self._teardown(set(self.LAYERS))
        self.mode = "idle"
        self.map_name = None
        self.phase = "idle"
        link.forget_map()
        link.forget_tf()

    def _to_mapping(self, link: "RobotLink", gen: int):
        """Caller holds _busy."""
        self.phase = "switching to mapping"
        self._teardown({"explore", "nav2", "amcl"})
        if not self._current(gen):
            return
        # Both are latched state the new stack must not inherit — otherwise
        # the gate below passes on AMCL's grid and we never notice that
        # cartographer failed to start.
        link.forget_map()
        link.forget_tf()
        self.mode = "mapping"
        self.map_name = None

        if not self._start_cartographer(link, gen):
            return
        # Nav2 is deliberately not started. Driving by hand needs only
        # cartographer, so this reaches a usable joystick in about ten
        # seconds rather than the minute or more Nav2's costmap gate costs.
        self.phase = "mapping"

    def _to_localize(self, link: "RobotLink", gen: int,
                     map_yaml: str, map_name: str):
        """Caller holds _busy."""
        self.phase = "switching to localize"
        self._teardown({"explore", "nav2", "cartographer"})
        if not self._current(gen):
            return
        link.forget_map()
        link.forget_tf()
        self.mode = "localize"
        self.map_name = map_name

        with self._lock:
            self._launch("amcl", "amcl.launch.py", {"map_yaml": map_yaml})
        if not self._wait_until(link.has_map, 30.0,
                                f"loading {map_name}", gen):
            if self._current(gen):
                self.phase = "the map never loaded — see the amcl log"
            return
        # Terminal until the user acts: AMCL withholds map→odom until it is
        # told where the robot is, and Nav2 cannot start without that
        # transform, so nothing else can proceed here.
        self.phase = "set the robot's position"

    # Enough rejections to be certain it is wedged rather than mid-startup:
    # they arrive at the scan rate, so this is about two seconds' worth.
    POISON_LINES = 20

    def _scans_rejected(self) -> bool:
        """True when cartographer is discarding every scan.

        It takes a scan stamped far in the future (+96 s, +623 s and +669 s
        have all been seen), stores that as the last subdivision time, and
        then drops everything older — which is everything — until wall time
        catches up.

        The bad scan comes from the lidar driver, not from cartographer:
        restarting cartographer alone does not clear it (observed failing
        three times in a row), while restarting bringup fixes it
        immediately. Sampling the topic from outside shows nothing wrong,
        which fits a rare bad message that a BEST_EFFORT subscriber drops
        and cartographer's RELIABLE one receives.
        """
        tail = self.log_tail("cartographer", 120)
        return tail.count("Ignored subdivision") >= self.POISON_LINES

    def _start_cartographer(self, link: "RobotLink", gen: int) -> bool:
        """Launch cartographer and wait for its first grid.

        One retry, because the wedge above is occasionally cleared by a
        fresh subscription. If it survives that, only a bringup restart
        helps, so stop burning the user's time and say so.
        """
        for attempt in range(2):
            if not self.running("cartographer"):
                with self._lock:
                    self._launch("cartographer", "cartographer.launch.py")

            deadline = time.monotonic() + 45.0
            self.phase = ("waiting for the map" if attempt == 0
                          else "retrying cartographer")
            while time.monotonic() < deadline:
                if not self._current(gen):
                    return False
                if link.has_map():
                    return True
                if self._scans_rejected():
                    break            # wedged — no point waiting out the clock
                time.sleep(0.5)

            if attempt == 0:
                with self._lock:
                    self._stop("cartographer")
                link.forget_map()

        if self._current(gen):
            self.phase = self._diagnose_no_map()
        return False

    def _diagnose_no_map(self) -> str:
        """Say why no map arrived, rather than just that none did.

        The usual cause is not a slow start. Cartographer occasionally
        latches onto a scan stamped in the future and then discards every
        scan older than it — once per scan, for as long as it takes wall
        time to catch up (96 s and 623 s have both been seen). It never
        recovers on its own, and from the outside it looks exactly like
        mapping that simply is not progressing. Restarting clears the
        stored timestamp.
        """
        tail = self.log_tail("cartographer", 200)
        if "Ignored subdivision" in tail:
            # Restarting cartographer does not clear this — the bad scan
            # comes from the lidar driver, which lives in bringup.
            return "lidar timestamps are bad — restart bringup on the robot"
        if not self.running("cartographer"):
            return "cartographer exited — see its log"
        return "cartographer published no map"

    def ensure_nav2(self, link: "RobotLink", gen: int) -> bool:
        """Bring Nav2 up if it isn't, and wait for a usable global costmap.

        The pose precondition is not politeness, it is required: Nav2's
        costmap blocks on a map→base_footprint transform during activation
        and returns FAILURE after initial_transform_timeout (60 s), which
        kills the lifecycle bringup for good rather than retrying. Under AMCL
        that transform does not exist until an initial pose has been set.
        """
        if link.pose() is None:
            # Wait for it while mapping — cartographer may still be starting,
            # and the transform is usually a second or two away. Under AMCL it
            # never arrives on its own, so say what to do instead of stalling.
            waited = self.mode == "mapping" and self._wait_until(
                lambda: link.pose() is not None, 30.0,
                "waiting for the robot's position", gen)
            if not waited:
                if self._current(gen):
                    self.phase = ("set the robot's position first"
                                  if self.mode == "localize"
                                  else "no robot position — is the lidar up?")
                return False

        if not self.running("nav2"):
            with self._lock:
                self._launch("nav2", "nav2.launch.py")
        elif link.costmap_ready():
            return True
        if not self._wait_until(link.costmap_ready, 180.0,
                                "waiting for nav2", gen):
            if self._current(gen):
                self.phase = "nav2 global costmap never appeared"
            return False
        return True

    def start_explore(self, link: "RobotLink"):
        """Worker: bring Nav2 up if needed, then start exploring."""
        # _join, not _begin: pressing this while the mode is still coming up
        # should queue behind that, not cancel it.
        gen = self._join()
        with self._busy:
            if not self._current(gen) or self.mode != "mapping":
                return
            try:
                self._start_explore_locked(link, gen)
            except Exception as e:
                self.phase = f"error: {type(e).__name__}: {e}"

    def _start_explore_locked(self, link: "RobotLink", gen: int):
        """Caller holds _busy."""
        if not self.ensure_nav2(link, gen):
            return
        if not self.running("explore"):
            with self._lock:
                self._launch("explore", "explore.launch.py")
            threading.Thread(target=self._nurse_explore, args=(link, gen),
                             daemon=True).start()
        self.phase = "mapping"

    def stop_explore(self):
        """Synchronous — /api/mapping/save relies on the robot being still
        by the time it returns. Nav2 is left up: re-arming should be instant,
        and an idle Nav2 publishes no /cmd_vel."""
        with self._lock:
            self._stop("explore")

    def _nurse_explore(self, link: "RobotLink", gen: int):
        """explore_lite gives up permanently the first time a frontier search
        comes back empty, and that happens for transient reasons too — most
        often the global costmap mid-resize as the map grows, which briefly
        leaves no FREE_SPACE for nearestCell() to find.

        So supervise for the whole session, not just the opening seconds: any
        'complete' while the map is still growing is treated as a stall and
        resumed. Only a complete on a map that has stopped changing for
        STABLE_S is taken at face value."""
        STABLE_S = 45.0
        POLL_S = 5.0
        last_seq = -1
        last_change = time.monotonic()

        while self._current(gen):
            time.sleep(POLL_S)
            if not self.running("explore"):
                return

            _, seq = link.map_snapshot()
            if seq != last_seq:
                last_seq = seq
                last_change = time.monotonic()

            if link.explore_status != ExploreStatus.EXPLORATION_COMPLETE:
                continue

            stable_for = time.monotonic() - last_change
            if stable_for < STABLE_S:
                self.phase = f"nudging explore (map grew {stable_for:.0f}s ago)"
                link.resume_explore()
            else:
                self.phase = "explored — map stable"

    def stop_all(self, link: "RobotLink | None" = None):
        gen = self._begin()
        with self._busy:
            if not self._current(gen):
                return
            if link is not None:
                self._to_idle(link)
            else:
                # Process shutdown: no link to clean up, just kill the tree.
                self.phase = "idle"
                self._teardown(set(self.LAYERS))
                self.mode = "idle"
                self.map_name = None

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
        pose = link.pose()
        return jsonify({
            "mode": modes.mode,
            "map_name": modes.map_name,
            "localized": modes.mode == "localize" and pose is not None,
            "modes": modes.status(),
            "detail": modes.detail(),
            "phase": modes.phase,
            "explore_status": link.explore_status,
            "pose": pose,
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

    @app.route("/api/mode", methods=["POST"])
    def set_mode():
        body = request.json or {}
        mode = body.get("mode", "idle")
        if mode not in ("idle", "mapping", "localize"):
            return jsonify({"ok": False, "error": f"unknown mode {mode!r}"}), 400

        map_yaml = map_name = None
        if mode == "localize":
            map_name = (body.get("map") or "").strip()
            if not map_name or "/" in map_name or map_name.startswith("."):
                return jsonify({"ok": False, "error": "bad map name"}), 400
            map_yaml = os.path.join(ws, "maps", map_name + ".yaml")
            # Check before launching: map_server would otherwise come up,
            # fail to load, and leave the user watching a phase that never
            # advances with nothing saying why.
            if not os.path.isfile(map_yaml):
                return jsonify({
                    "ok": False,
                    "error": f"no saved grid for {map_name!r}",
                }), 400

        # Switching tears down a whole stack and waits for the next one, far
        # longer than a request should take — hand it to a worker and let the
        # client follow `phase`.
        threading.Thread(
            target=modes.set_mode, args=(mode, link, map_yaml, map_name),
            daemon=True,
        ).start()
        return jsonify({"ok": True}), 202

    @app.route("/api/explore", methods=["POST"])
    def explore():
        on = bool((request.json or {}).get("on", False))
        if not on:
            modes.stop_explore()
            return jsonify({"ok": True})
        if modes.mode != "mapping":
            return jsonify({"ok": False, "error": "only while mapping"}), 409
        # Starting may have to bring Nav2 up first, which takes far longer
        # than a request should, so hand it to a worker and let the client
        # follow the phase.
        threading.Thread(target=modes.start_explore, args=(link,),
                         daemon=True).start()
        return jsonify({"ok": True}), 202

    @app.route("/api/mapping/save", methods=["POST"])
    def mapping_save():
        name = (request.json or {}).get("name", "").strip()
        if not name or "/" in name or name.startswith("."):
            return jsonify({"ok": False, "error": "bad map name"}), 400

        # Stop exploring first so the robot is still while the graph is sealed.
        modes.stop_explore()
        ok, detail = save_map(ws, name)
        return jsonify({"ok": ok, "detail": detail}), (200 if ok else 500)

    @app.route("/api/explore/resume", methods=["POST"])
    def explore_resume():
        link.resume_explore()
        return jsonify({"ok": True})

    @app.route("/api/logs")
    def logs():
        layer = request.args.get("layer", "cartographer")
        if layer not in ModeStack.LAYERS:
            return ("unknown layer", 400)
        return Response(modes.log_tail(layer, 60), mimetype="text/plain")

    @app.route("/api/debug")
    def debug():
        """What environment the launches actually inherit — the usual reason
        `ros2 launch` dies instantly is a workspace that was never sourced."""
        which = subprocess.run(["bash", "-lc", "command -v ros2"],
                               capture_output=True, text=True)
        pkg = subprocess.run(["ros2", "pkg", "prefix", "picar2_bringup"],
                             capture_output=True, text=True)
        return jsonify({
            "pose_error": link.pose_error,
            "free_cells": link.free_cells(),
            "costmap_free": link.costmap_free(),
            "map_info": link.map_info(),
            "explore_status": link.explore_status,
            "tf_frames": link.tf_frames(),
            "publishers": {
                "/tf": link.count_publishers("/tf"),
                "/scan": link.count_publishers("/scan"),
                "/lidar_node/scan": link.count_publishers("/lidar_node/scan"),
                "/map": link.count_publishers("/map"),
                "/global_costmap/costmap": link.count_publishers("/global_costmap/costmap"),
                "/cmd_vel": link.count_publishers("/cmd_vel"),
            },
            "subscribers": {
                "/cmd_vel": link.count_subscribers("/cmd_vel"),
            },
            "ros2": which.stdout.strip(),
            "picar2_bringup_prefix": pkg.stdout.strip() or pkg.stderr.strip(),
            "ROS_DISTRO": os.environ.get("ROS_DISTRO"),
            "RMW_IMPLEMENTATION": os.environ.get("RMW_IMPLEMENTATION"),
            "CYCLONEDDS_URI": os.environ.get("CYCLONEDDS_URI"),
            "ROS_DOMAIN_ID": os.environ.get("ROS_DOMAIN_ID"),
            "AMENT_PREFIX_PATH": os.environ.get("AMENT_PREFIX_PATH", "").split(":"),
            "PICAR_WS": os.environ.get("PICAR_WS"),
            "cwd": os.getcwd(),
        })

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
