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
import logging
import os
import shutil
import signal
import subprocess
import sys
import threading
import time
import xmlrpc.client

import rclpy
import numpy as np
import yaml
from flask import Flask, Response, jsonify, request, send_from_directory
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from explore_lite_msgs.msg import ExploreStatus
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from nav2_msgs.srv import SetInitialPose
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool
from rclpy.action import ActionClient
from rclpy.node import Node
# Aliased: rcl_interfaces.msg.Parameter is already imported above for the
# SetParameters service, and an unaliased import here shadows it — which
# broke set_max_speed with a TypeError from deep inside rclpy.
from rclpy.parameter import Parameter as RclpyParameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from waitress import serve

# Commands older than this are treated as released. The controller's own
# reference_timeout (0.5 s) is the backstop if this process dies outright.
DRIVE_TIMEOUT_S = 0.4
DRIVE_RATE_HZ = 20.0

# The touch joystick drives at whatever top speed navigation is set to, so
# one setting governs both. This is only the fallback for when navigation
# is not running and there is nothing to read a speed from — chiefly the
# window in localize mode between entering it and setting the initial pose.
# It matches desired_linear_vel in nav2.yaml, so crossing that boundary
# does not change how the robot drives.
MAX_LINEAR = 0.40
# Ackermann steering ties the two together: w = v / r. Deriving the angular
# cap from the linear one keeps the tightest reachable turn constant as the
# speed changes, instead of the robot steering ever more widely as it speeds
# up. 0.34 m is the mechanical minimum radius; cmd_vel_relay clamps angular
# at 1.2 rad/s regardless, so asking for more than that achieves nothing.
MIN_TURN_RADIUS_M = 0.34
RELAY_MAX_ANGULAR = 1.2


class RobotLink(Node):
    """ROS side: map in, pose in, cmd_vel out, cartographer services."""

    def __init__(self):
        # The UI is a ROS node too: it looks up map->base_footprint and reads
        # /map. Under Gazebo both are stamped with /clock, and on the wall
        # clock the lookup never resolves — the UI sits on "no robot position"
        # forever with a perfectly good map on screen. Set from the profile
        # rather than a command-line argument, because this script has its own
        # argparse and rejects --ros-args.
        sim = os.environ.get("PICAR_PROFILE", "robot") == "sim"
        super().__init__(
            "picar_webui",
            parameter_overrides=[
                RclpyParameter("use_sim_time", RclpyParameter.Type.BOOL, True)
            ] if sim else [])

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
        # The local costmap is what the robot believes is in its way right
        # now — the lidar and ToF returns after marking and inflation. Worth
        # showing: a false obstacle is invisible on the static map, and that
        # is exactly the failure that cost a test route most of one leg.
        self.create_subscription(OccupancyGrid, "/local_costmap/costmap",
                                 self._on_local_costmap, map_qos)
        self._local: OccupancyGrid | None = None

        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        # explore_lite stops permanently the first time it finds no frontiers.
        # At startup that happens routinely, before the global costmap has
        # copied the map in — resume puts it back to work.
        self._resume_pub = self.create_publisher(Bool, "/explore/resume", 10)
        # AMCL takes an initial pose two ways. Prefer the service: it is
        # acknowledged, whereas the topic subscription is volatile, so a
        # message published before discovery finishes is dropped with no
        # error at all — the most confusing failure the user could hit.
        self._initpose_cli = self.create_client(
            SetInitialPose, "/set_initial_pose")
        self._initpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10)
        # A real action client, not a shell-out to `ros2 action send_goal`:
        # cancelling has to land in well under a second (until it does, two
        # publishers are fighting over an unmuxed /cmd_vel), and a
        # subprocess cannot be cancelled from outside. Everything here is
        # callback-driven, so the existing spin thread carries it and
        # _drive_tick stays the only timer.
        self._nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        # The controller's top speed, read and written live. RPP handles this
        # one in its dynamic-parameter callback, so a set takes effect on the
        # next control cycle without restarting anything. It is not written
        # back to nav2.yaml and does not survive a Nav2 restart — which the
        # mode switch does — so it is read fresh whenever Nav2 reappears.
        self._speed_get = self.create_client(
            GetParameters, "/controller_server/get_parameters")
        self._speed_set = self.create_client(
            SetParameters, "/controller_server/set_parameters")
        self.max_speed = None
        self._goal_handle = None
        # Each goal carries a sequence number so late callbacks from a
        # superseded one cannot clobber the current goal's state, and a
        # cancel arriving before the server has accepted is remembered
        # rather than dropped.
        self._goal_seq = 0
        self._cancel_pending = False
        self.nav_state = "idle"
        self.nav_goal = None
        self.nav_distance = None

        # ── routes ───────────────────────────────────────────────────────
        # Two ways to drive a list of waypoints, and they differ in what
        # decides a waypoint has been reached:
        #
        #   stop-at-each  one NavigateToPose per waypoint. Nav2's goal
        #                 checker decides, so arrival is held to
        #                 xy_goal_tolerance — at the cost of decelerating
        #                 into every waypoint and accelerating out.
        #   flow-through  a rolling NavigateThroughPoses window. The
        #                 controller sees one continuous path, so it never
        #                 slows for an intermediate waypoint.
        #
        # Flow-through was tried before and reverted, because it trusted
        # the action's SUCCEEDED to mean "reached them": the BT trims goals
        # inside RemovePassedGoals' radius, and a window trimmed empty
        # reports success having visibly missed them — measured at 0.75 m.
        # It is back because arrival is no longer the BT's call. The server
        # watches the robot's own pose and counts a waypoint passed at its
        # closest approach, so trimming can no longer fabricate progress.
        self._route_nav = ActionClient(self, NavigateThroughPoses,
                                       "navigate_through_poses")
        self.waypoints: list[dict] = []
        self.route_active = False
        self.route_loop = False
        self.route_flow = False
        self.route_idx = 0
        self.route_passed = 0
        self.route_error = None
        self._route_poses: list[dict] = []
        self._route_seq = 0
        self._route_handle = None
        self._route_retries = 0
        self._route_sent_idx = None     # index the live window starts at
        self._route_min_d = None        # closest approach so far, current wp
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
        self.forget_nav()
        # Waypoints are map-frame coordinates and a new mode brings a new
        # map, so keeping them would point the robot at places that no
        # longer mean anything.
        self.waypoints = []
        self.route_passed = 0
        self.route_error = None

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
        # The node's clock, not time.time(): under use_sim_time the transform
        # stamps are simulation time (seconds since the sim started) while
        # time.time() is the wall clock, so every transform compares as older
        # than the cutoff and the pose is rejected forever. On the robot the
        # two agree, which is why this only ever showed up in Gazebo.
        self._tf_valid_from = self.get_clock().now().nanoseconds * 1e-9
        self.pose_error = "no lookup yet"

    def _on_costmap(self, msg: OccupancyGrid):
        cells = np.frombuffer(bytes(msg.data), dtype=np.int8)
        self._costmap_free = int(np.count_nonzero((cells >= 0) & (cells <= 25)))

    def costmap_free(self) -> int:
        return self._costmap_free

    def _on_local_costmap(self, msg: OccupancyGrid):
        with self._lock:
            self._local = msg

    def local_cell_size(self) -> float:
        with self._lock:
            return self._local.info.resolution if self._local else 0.05

    # Lethal and inscribed. Inflation below this is derived from these cells
    # rather than sensed, so drawing it would triple the point count to say
    # the same thing.
    OBSTACLE_COST = 99

    def local_obstacles(self) -> tuple[bytes | None, str]:
        """Marked cells of the local costmap as map-frame xy pairs.

        Transformed here rather than in the browser because the costmap is
        published in odom while the UI draws in map. Those frames differ by
        the localisation correction, which is a rotation as well as an
        offset: sending the grid and its origin would smear every obstacle
        by up to a quarter of a metre across a 3 m window at a few degrees
        of yaw error, and put them somewhere the robot never saw anything.
        """
        with self._lock:
            msg = self._local
        if msg is None:
            return None, "no local costmap published yet"
        info = msg.info
        cells = np.frombuffer(bytes(msg.data), dtype=np.int8)
        if cells.size != info.width * info.height:
            return None, "costmap data does not match its declared size"
        idx = np.nonzero(cells >= self.OBSTACLE_COST)[0]
        if idx.size == 0:
            return b"", ""
        rows, cols = np.divmod(idx, info.width)
        # Cell centres in the costmap's own frame.
        px = info.origin.position.x + (cols + 0.5) * info.resolution
        py = info.origin.position.y + (rows + 0.5) * info.resolution

        frame = msg.header.frame_id or "odom"
        if frame.lstrip("/") != "map":
            try:
                tf = self._tf_buffer.lookup_transform("map", frame, Time())
            except Exception as e:                           # noqa: BLE001
                # Better nothing than obstacles drawn somewhere the robot
                # never actually saw anything.
                return None, "no %s->map transform: %s" % (frame, e)
            t = tf.transform.translation
            q = tf.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            c, sn = math.cos(yaw), math.sin(yaw)
            px, py = t.x + c * px - sn * py, t.y + sn * px + c * py
        return np.column_stack([px, py]).astype(np.float32).tobytes(), ""

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

    # ── initial pose ─────────────────────────────────────────────────────
    def set_initial_pose(self, x: float, y: float, yaw: float):
        """Tell AMCL where the robot is. Returns (ok, message)."""
        msg = PoseWithCovarianceStamped()
        # AMCL rejects an initial pose in any other frame.
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        # RViz's "2D Pose Estimate" covariance: half a metre and ~15° of
        # doubt, which is about how well anyone can tap a map.
        cov = [0.0] * 36
        cov[0] = 0.25
        cov[7] = 0.25
        cov[35] = 0.06853891945200942
        msg.pose.covariance = cov

        if not self._initpose_cli.wait_for_service(timeout_sec=2.0):
            return False, "AMCL isn't running yet"

        done = threading.Event()
        fut = self._initpose_cli.call_async(SetInitialPose.Request(pose=msg))
        fut.add_done_callback(lambda _f: done.set())
        # Never spin here — the node has its own spin thread, and spinning
        # from the request thread would run callbacks on two threads at once.
        done.wait(timeout=3.0)
        # Belt and braces: the topic is idempotent for AMCL and covers the
        # case where the service exists but the call was slow.
        self._initpose_pub.publish(msg)

        # AMCL withholds map→odom until it believes it knows where it is, so
        # a pose appearing is positive proof it accepted this one. Without
        # the check a pose outside the map is discarded in silence.
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            if self.pose() is not None:
                return True, ""
            time.sleep(0.1)
        return False, "AMCL took the pose but published no transform"

    # ── speed ────────────────────────────────────────────────────────────
    SPEED_PARAM = "FollowPath.desired_linear_vel"
    # Above roughly this, cmd_vel_relay's max_angular_vel (1.2 rad/s) starts
    # clipping the turn rate a Reeds-Shepp path asks for on the planner's
    # 0.5 m minimum radius, and the robot under-steers off its own plan.
    SPEED_MAX = 0.6
    SPEED_MIN = 0.1

    def _call(self, client, req, timeout=3.0):
        """One request against a parameter service. Never spins — the node
        has its own spin thread, and spinning here would run callbacks on
        two threads at once."""
        if not client.wait_for_service(timeout_sec=1.5):
            return None
        done = threading.Event()
        fut = client.call_async(req)
        fut.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout=timeout):
            return None
        try:
            return fut.result()
        except Exception:
            return None

    def refresh_max_speed(self):
        """Read the controller's current top speed. Returns it, or None."""
        res = self._call(self._speed_get,
                         GetParameters.Request(names=[self.SPEED_PARAM]))
        if res is None or not res.values:
            return None
        v = res.values[0]
        if v.type != ParameterType.PARAMETER_DOUBLE:
            return None                 # unset params come back as NOT_SET
        self.max_speed = round(float(v.double_value), 3)
        return self.max_speed

    def set_max_speed(self, value: float):
        """Change it. Returns (ok, message)."""
        try:
            v = float(value)
        except (TypeError, ValueError):
            return False, "not a number"
        if not (self.SPEED_MIN <= v <= self.SPEED_MAX):
            return False, (f"pick a speed between {self.SPEED_MIN:.2f} and "
                           f"{self.SPEED_MAX:.2f} m/s")
        req = SetParameters.Request(parameters=[Parameter(
            name=self.SPEED_PARAM,
            value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                 double_value=v))])
        res = self._call(self._speed_set, req)
        if res is None:
            return False, "navigation isn't running"
        if not res.results or not res.results[0].successful:
            reason = (res.results[0].reason if res.results else "") or "refused"
            return False, reason
        self.max_speed = round(v, 3)
        return True, ""

    # ── navigation goals ─────────────────────────────────────────────────
    def send_goal(self, x: float, y: float, yaw: float):
        """Ask Nav2 to drive somewhere. Returns (ok, message)."""
        if not self._nav.wait_for_server(timeout_sec=2.0):
            return False, "starting navigation — try again in a moment"

        goal = NavigateToPose.Goal()
        p = PoseStamped()
        p.header.frame_id = "map"
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = float(x)
        p.pose.position.y = float(y)
        p.pose.orientation.z = math.sin(yaw / 2.0)
        p.pose.orientation.w = math.cos(yaw / 2.0)
        goal.pose = p

        # Set before sending so /api/status shows the intent immediately,
        # rather than a half-second of looking like nothing happened.
        self._goal_seq += 1
        seq = self._goal_seq
        self._cancel_pending = False
        self.nav_state = "pending"
        self.nav_goal = {"x": float(x), "y": float(y), "yaw": float(yaw)}
        self.nav_distance = None
        fut = self._nav.send_goal_async(
            goal, feedback_callback=lambda m: self._on_nav_feedback(m, seq))
        fut.add_done_callback(lambda f: self._on_nav_accepted(f, seq))
        return True, ""

    def _on_nav_accepted(self, fut, seq):
        if seq != self._goal_seq:
            return                      # a newer goal has taken over
        try:
            gh = fut.result()
        except Exception:
            self.nav_state = "rejected"
            return
        if not gh.accepted:
            self.nav_state = "rejected"
            self._goal_handle = None
            return
        self._goal_handle = gh
        self.nav_state = "active"
        gh.get_result_async().add_done_callback(
            lambda f: self._on_nav_result(f, seq))
        # Someone grabbed the joystick while we were waiting to be accepted.
        # Without this the cancel is lost and Nav2 starts driving into a
        # /cmd_vel stream the human thinks they own.
        if self._cancel_pending:
            self.cancel_goal()

    def _on_nav_feedback(self, msg, seq):
        if seq != self._goal_seq:
            return
        self.nav_distance = float(msg.feedback.distance_remaining)

    def _on_nav_result(self, fut, seq):
        if seq != self._goal_seq:
            # A preempted goal reports ABORTED after the new one was
            # accepted; letting that through would show "could not get
            # there" while the robot drives, and drop the live handle.
            return
        # Read the status, not the result fields: the installed nav2_msgs is
        # the packaged one, and NavigateToPose.Result has drifted across
        # releases, while GoalStatus has not.
        try:
            status = fut.result().status
        except Exception:
            status = GoalStatus.STATUS_ABORTED
        self.nav_state = {
            GoalStatus.STATUS_SUCCEEDED: "succeeded",
            GoalStatus.STATUS_CANCELED: "canceled",
        }.get(status, "aborted")
        self._goal_handle = None
        self.nav_distance = None
        # A route is a chain of these, so advance it from the same place a
        # single goal reports — one result path, one set of guards.
        self._route_on_goal_done(self.nav_state)

    def cancel_goal(self):
        """Idempotent, and safe before the goal has been accepted."""
        if self.nav_state in ("idle", "succeeded", "aborted",
                              "canceled", "rejected"):
            return
        self._cancel_pending = True
        gh = self._goal_handle
        if gh is None:
            # Still waiting to be accepted — there is nothing to cancel yet,
            # so record it and let _on_nav_accepted do it on arrival.
            self.nav_state = "canceling"
            return
        self.nav_state = "canceling"
        try:
            gh.cancel_goal_async()      # fire and forget; the result lands
        except Exception:                # in _on_nav_result as "canceled"
            pass

    def nav_server_ready(self) -> bool:
        """Cheap, non-blocking check that the action server is discovered."""
        try:
            return self._nav.server_is_ready()
        except Exception:
            return False

    def nav_status(self) -> dict:
        return {"state": self.nav_state, "goal": self.nav_goal,
                "distance": self.nav_distance}

    def forget_nav(self):
        self.max_speed = None
        self.route_stop()
        self.cancel_goal()
        self._goal_seq += 1             # orphan any callback still in flight
        self._cancel_pending = False
        self._goal_handle = None
        self.nav_state = "idle"
        self.nav_goal = None
        self.nav_distance = None

    # ── routes ───────────────────────────────────────────────────────────
    # One waypoint at a time through NavigateToPose, advancing when each is
    # actually reached.
    #
    # NavigateThroughPoses with a rolling window keeps the robot moving
    # between waypoints, but its BT trims goals within RemovePassedGoals'
    # 0.7 m radius, so waypoints count as reached from well away — and a
    # window can be trimmed empty and report SUCCEEDED having visibly
    # missed them. Measured on this robot: a three-waypoint route finished
    # with the robot 0.75 m from the nearest one. Waypoints a person placed
    # deliberately deserve the goal checker's 0.25 m, at the cost of a pause
    # at each.
    ANCHOR_RADIUS_M = 0.6
    # Nav2's goal checker wants the robot within xy_goal_tolerance (0.25 m)
    # and stopped there. A car that cannot turn tighter than 0.5 m often
    # parks a little outside that and shuffles until the BT gives up —
    # observed at 0.333 m with the heading already correct to 5 degrees.
    # A route only needs to pass through a waypoint, so treat "Nav2 gave up
    # but we are practically on it" as reached and carry on.
    NEAR_ENOUGH_M = 0.6

    def add_waypoint(self, x: float, y: float, yaw: float,
                     auto_yaw: bool = False) -> int:
        self.waypoints.append({"x": float(x), "y": float(y),
                               "yaw": float(yaw), "auto": bool(auto_yaw)})
        return len(self.waypoints)

    def clear_waypoints(self):
        self.route_stop()
        self.waypoints = []

    def drop_last_waypoint(self):
        if self.waypoints:
            self.waypoints.pop()

    def route_start(self, loop: bool, flow: bool = False):
        """Begin driving the waypoints. Returns (ok, message)."""
        if len(self.waypoints) < 2:
            return False, "add at least two waypoints"
        client = self._route_nav if flow else self._nav
        if not client.wait_for_server(timeout_sec=2.0):
            return False, "navigation isn't ready yet"

        # Snapshot, so editing the list mid-route cannot corrupt the run.
        self.route_loop = bool(loop)
        self.route_flow = bool(flow)
        self._route_poses = self._resolved_waypoints()
        self.route_passed = 0
        self.route_error = None
        self.route_idx = self._route_first_index()
        self.route_active = True
        self._route_min_d = None
        self._route_sent_idx = None
        self._route_retries = 0
        # A route and a single goal both drive; never let them overlap.
        self.cancel_goal()
        if self.route_flow:
            self._route_send_window()
        else:
            self._route_send_current()
        return True, ""

    def _resolved_waypoints(self) -> list[dict]:
        """Fill in arrival headings the user did not choose.

        An arbitrary heading is the difference between a reachable waypoint
        and an unreachable one: the planner is a Reeds-Shepp lattice with a
        0.5 m turning radius, so "arrive here facing that way" can simply
        have no solution in a small space, and Nav2 answers by running its
        backup behaviour over and over. Facing the next waypoint is both
        the natural way to drive a route and nearly always feasible, since
        it is the direction of travel.
        """
        wps = [dict(w) for w in self.waypoints]
        n = len(wps)
        for i, w in enumerate(wps):
            if not w.get("auto"):
                continue
            if i + 1 < n:
                nxt = wps[i + 1]
            elif self.route_loop and n > 1:
                nxt = wps[0]
            else:
                nxt = None
            if nxt is not None:
                w["yaw"] = math.atan2(nxt["y"] - w["y"], nxt["x"] - w["x"])
            elif n > 1:
                # Last waypoint of a one-shot route: carry on facing the way
                # we arrived rather than inventing a turn at the end.
                prev = wps[i - 1]
                w["yaw"] = math.atan2(w["y"] - prev["y"], w["x"] - prev["x"])
        return wps

    def _route_first_index(self) -> int:
        """Where to begin.

        A loop is a lap you join at the nearest waypoint. A one-shot route
        is a list the user wrote in order, so it starts at the beginning —
        starting from the nearest would silently skip everything before it.
        Either way, if the robot is already standing on a waypoint, begin
        with the next one rather than driving to where it already is.
        """
        n = len(self._route_poses)
        p = self.pose()
        if p is None:
            return 0
        d = [math.hypot(w["x"] - p["x"], w["y"] - p["y"])
             for w in self._route_poses]
        closest = min(range(n), key=lambda i: d[i])
        on_a_waypoint = d[closest] < self.ANCHOR_RADIUS_M

        if self.route_loop:
            return (closest + 1) % n if on_a_waypoint else closest
        if on_a_waypoint and closest == 0:
            return 1 if n > 1 else 0
        return 0

    def _route_send_current(self):
        if not self.route_active:
            return
        n = len(self._route_poses)
        if self.route_idx >= n:
            self._route_finish("route complete")
            return
        wp = self._route_poses[self.route_idx]
        ok, err = self.send_goal(wp["x"], wp["y"], wp["yaw"])
        if not ok:
            self._route_finish(err or "could not send the waypoint")

    # ── flow-through routes ──────────────────────────────────────────────
    # The window holds this many waypoints. Only its final pose is a goal
    # the controller must stop on, so keeping the end two or more waypoints
    # ahead is what stops the robot decelerating into the one it is
    # currently passing.
    ROUTE_WINDOW = 3
    # Inside this, a waypoint counts as being passed...
    CAPTURE_RADIUS_M = 0.45
    # ...and it is confirmed once the robot is moving away from it again,
    # which puts the count at the closest approach rather than at the edge
    # of some radius. Must exceed the pose noise or a jittering estimate
    # would tick waypoints off on the spot.
    RECEDE_M = 0.08
    # Consecutive aborts on the same waypoint before the route gives up.
    ROUTE_ABORT_RETRIES = 2
    # RemovePassedGoals in the through-poses BT must trim inside
    # CAPTURE_RADIUS_M. If it trimmed later, the planner would stop routing
    # through a waypoint the robot had not yet come close enough to count,
    # and the route would stall on it forever.

    def _route_window(self) -> list[dict]:
        n = len(self._route_poses)
        size = min(self.ROUTE_WINDOW, n)
        if self.route_loop:
            return [self._route_poses[(self.route_idx + i) % n]
                    for i in range(size)]
        return self._route_poses[self.route_idx:self.route_idx + size]

    def _pose_stamped(self, wp: dict) -> PoseStamped:
        p = PoseStamped()
        p.header.frame_id = "map"
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = float(wp["x"])
        p.pose.position.y = float(wp["y"])
        p.pose.orientation.z = math.sin(float(wp["yaw"]) / 2.0)
        p.pose.orientation.w = math.cos(float(wp["yaw"]) / 2.0)
        return p

    def _route_send_window(self):
        if not self.route_active:
            return
        window = self._route_window()
        if not window:
            self._route_finish("route complete")
            return
        self._route_seq += 1
        seq = self._route_seq
        self._route_sent_idx = self.route_idx
        self.nav_state = "active"
        self.nav_goal = {"x": window[0]["x"], "y": window[0]["y"],
                         "yaw": window[0]["yaw"]}
        goal = NavigateThroughPoses.Goal(
            poses=[self._pose_stamped(w) for w in window])
        fut = self._route_nav.send_goal_async(goal)
        fut.add_done_callback(lambda f: self._on_window_accepted(f, seq))

    def _on_window_accepted(self, fut, seq):
        if seq != self._route_seq or not self.route_active:
            return                      # a newer window has taken over
        try:
            gh = fut.result()
        except Exception:
            self._route_finish("navigation refused the route")
            return
        if not gh.accepted:
            self._route_finish("navigation refused the route")
            return
        self._route_handle = gh
        gh.get_result_async().add_done_callback(
            lambda f: self._on_window_done(f, seq))

    def _on_window_done(self, fut, seq):
        """A window ended. In flow mode this is not how waypoints are
        counted — _route_flow_tick does that from the robot's pose — so the
        only questions here are whether the route is over and whether Nav2
        is telling us it cannot continue."""
        if seq != self._route_seq:
            return                      # superseded; its abort is our doing
        if not self.route_active:
            return
        try:
            status = fut.result().status
        except Exception:
            status = GoalStatus.STATUS_ABORTED

        if status == GoalStatus.STATUS_CANCELED:
            self._route_finish(None)
            return

        n = len(self._route_poses)
        at_end = (not self.route_loop) and self.route_idx >= n - 1
        if status == GoalStatus.STATUS_SUCCEEDED:
            if at_end:
                # The last waypoint of a one-shot route is a real goal and
                # the robot stopped on it, so the goal checker's verdict is
                # the right one to take.
                self.route_passed += 1
                self.route_idx = n
                self._route_finish("route complete")
            else:
                # The window ran out before the route did — the robot is
                # parked on the window's end. Carry on from where we are.
                self._route_send_window()
            return

        # Aborted. Close enough to the waypoint means press on, the same
        # judgement the stop-at-each path makes.
        wp = self._route_poses[self.route_idx % n]
        p = self.pose()
        near = (p is not None and
                math.hypot(p["x"] - wp["x"], p["y"] - wp["y"])
                < self.NEAR_ENOUGH_M)
        if near:
            self._route_retries = 0
            self._route_pass_current()
            return

        # Far from it, so this is not an arrival — but not necessarily a
        # dead end either. Nav2 aborts a goal for reasons that have nothing
        # to do with whether the waypoint is reachable: a TF lookup landing
        # a few ms in the future is enough, and on a loaded Pi that happens
        # to a route that has been lapping cleanly for minutes. Ask again
        # before giving up, and only call it unreachable when repeated
        # attempts make no progress — _route_pass_current resets the count,
        # so these are consecutive failures, not a lifetime budget.
        if self._route_retries < self.ROUTE_ABORT_RETRIES:
            self._route_retries += 1
            self._route_send_window()
            return
        self._route_finish("could not reach waypoint "
                           f"{self.route_idx % n + 1}")

    def _route_flow_tick(self):
        """Count the current waypoint passed at its closest approach.

        Called from the drive timer. Deliberately independent of the action
        result: the BT's idea of a passed goal is a radius, and trusting it
        is what made this approach report success while metres away.
        """
        if not self.route_active or not self.route_flow:
            return
        n = len(self._route_poses)
        if n == 0:
            return
        # The final waypoint of a one-shot route is a genuine stop, so let
        # the goal checker have it rather than calling it passed in transit.
        if (not self.route_loop) and self.route_idx >= n - 1:
            return
        p = self.pose()
        if p is None:
            return
        wp = self._route_poses[self.route_idx % n]
        d = math.hypot(p["x"] - wp["x"], p["y"] - wp["y"])
        if self._route_min_d is None or d < self._route_min_d:
            self._route_min_d = d
        if (self._route_min_d < self.CAPTURE_RADIUS_M
                and d > self._route_min_d + self.RECEDE_M):
            self._route_pass_current()

    def _route_pass_current(self):
        """Advance past the current waypoint, re-sending the window when it
        has been consumed far enough to be worth it. Each send preempts the
        running goal, so sending on every waypoint would reintroduce a stop
        at each — the thing this mode exists to avoid."""
        self.route_passed += 1
        self._route_min_d = None
        self._route_retries = 0
        n = len(self._route_poses)
        if self.route_loop:
            self.route_idx = (self.route_idx + 1) % n
        else:
            self.route_idx += 1
            if self.route_idx >= n:
                self._route_finish("route complete")
                return

        consumed = 0
        if self._route_sent_idx is not None:
            consumed = (self.route_idx - self._route_sent_idx) % n \
                if self.route_loop else self.route_idx - self._route_sent_idx
        if self._route_sent_idx is None or consumed >= self.ROUTE_WINDOW - 1:
            self._route_send_window()

    def _route_on_goal_done(self, state: str):
        """Called from the goal result once a waypoint's goal has ended."""
        if not self.route_active or self.route_flow:
            # A flow route is driven by the through-poses client and
            # advanced from the pose; a NavigateToPose result reaching here
            # is the leftover single goal that route_start cancelled.
            return
        if state == "canceled":
            self._route_finish(None)
            return
        if state != "succeeded":
            wp = self._route_poses[self.route_idx]
            p = self.pose()
            near = (p is not None and
                    math.hypot(p["x"] - wp["x"], p["y"] - wp["y"])
                    < self.NEAR_ENOUGH_M)
            if not near:
                self._route_finish("could not reach waypoint "
                                   f"{self.route_idx + 1}")
                return
            # Close enough — press on rather than abandoning the route for
            # the sake of a few centimetres.

        self.route_passed += 1
        n = len(self._route_poses)
        if self.route_loop:
            self.route_idx = (self.route_idx + 1) % n
        else:
            self.route_idx += 1
            if self.route_idx >= n:
                self._route_finish("route complete")
                return
        self._route_send_current()

    def _route_finish(self, error):
        was_flow = self.route_flow
        self.route_active = False
        self.route_error = error
        if was_flow and self._route_handle is not None:
            # "route complete" included: the window may still hold poses
            # beyond the last one the route cared about.
            self._cancel_window()

    def route_stop(self):
        """Idempotent."""
        if not self.route_active:
            return
        self.route_active = False
        self._cancel_window()
        self.cancel_goal()

    def _cancel_window(self):
        """Cancel a through-poses window, if one is running. Bumping the
        sequence first means the resulting abort is ignored rather than
        read as a navigation failure."""
        gh, self._route_handle = self._route_handle, None
        self._route_seq += 1
        self._route_sent_idx = None
        if gh is None:
            return
        try:
            gh.cancel_goal_async()
        except Exception:
            pass
        self.nav_state = "canceling"

    def route_status(self) -> dict:
        return {
            "waypoints": self.waypoints,
            "active": self.route_active,
            "loop": self.route_loop,
            "flow": self.route_flow,
            "index": self.route_idx if self.route_active else None,
            "passed": self.route_passed,
            "error": self.route_error,
        }

    # ── drive ────────────────────────────────────────────────────────────
    def drive_limits(self) -> tuple[float, float]:
        """What the joystick is allowed to command right now."""
        lin = self.max_speed if self.max_speed is not None else MAX_LINEAR
        return lin, min(RELAY_MAX_ANGULAR, lin / MIN_TURN_RADIUS_M)

    def set_drive(self, linear: float, angular: float):
        max_lin, max_ang = self.drive_limits()
        with self._lock:
            self._drive = (
                clamp(linear, -max_lin, max_lin),
                clamp(angular, -max_ang, max_ang),
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

        # A flow route advances on the robot's position rather than on an
        # action result, so it needs a clock. This is the only timer.
        if self.route_active and self.route_flow:
            try:
                self._route_flow_tick()
            except Exception:               # never kill the drive timer
                pass


def clamp(v, lo, hi):
    return max(lo, min(hi, v))



class Supervisor:
    """The supervisord running as PID 1 in this container.

    The navigation layers used to be children of this process, so restarting
    the web UI to pick up a change tore down AMCL and Nav2 with it and cost a
    re-localization every time. They are supervisord programs now; this is the
    handle on them.

    Every call is wrapped: supervisord is in the same container and should
    always be there, but a UI that raises 500 because a status poll lost a
    socket is worse than one that reports a layer as stopped.
    """

    SOCKET = os.environ.get("SUPERVISOR_SOCKET", "unix:///tmp/supervisor.sock")

    def __init__(self):
        self._rpc = None
        self._lock = threading.Lock()

    def _server(self):
        if self._rpc is None:
            from supervisor.xmlrpc import SupervisorTransport
            self._rpc = xmlrpc.client.ServerProxy(
                "http://127.0.0.1",
                transport=SupervisorTransport(None, None, self.SOCKET))
        return self._rpc

    def available(self) -> bool:
        try:
            with self._lock:
                self._server().supervisor.getSupervisorVersion()
            return True
        except Exception:                                    # noqa: BLE001
            return False

    def info(self, program: str) -> dict | None:
        try:
            with self._lock:
                return self._server().supervisor.getProcessInfo(program)
        except Exception:                                    # noqa: BLE001
            return None

    def start(self, program: str) -> tuple[bool, str]:
        try:
            with self._lock:
                # wait=False: a ros2 launch tree takes tens of seconds to be
                # useful and supervisord's own startsecs only proves it did not
                # die immediately. Readiness is decided by _wait_until against
                # the actual topics, which is the only honest test.
                self._server().supervisor.startProcess(program, False)
            return True, ""
        except xmlrpc.client.Fault as e:
            # Already running is not an error to the caller: the layer is up,
            # which is what was asked for.
            if e.faultString.startswith("ALREADY_STARTED"):
                return True, ""
            return False, e.faultString
        except Exception as e:                               # noqa: BLE001
            return False, repr(e)

    def stop(self, program: str) -> tuple[bool, str]:
        try:
            with self._lock:
                self._server().supervisor.stopProcess(program, True)
            return True, ""
        except xmlrpc.client.Fault as e:
            if e.faultString.startswith("NOT_RUNNING"):
                return True, ""
            return False, e.faultString
        except Exception as e:                               # noqa: BLE001
            return False, repr(e)


class ModeStack:
    """Starts and stops the cartographer → nav2 → explore chain."""

    # Ordered producers-first, so reversed() is a safe teardown order:
    # explore → nav2 → amcl → cartographer, consumers before what they read.
    # cartographer and amcl are peers — both publish /map and broadcast
    # map→odom, so they must never run at the same time.
    # "map" rather than "cartographer": the layer runs whichever SLAM backend
    # is selected, and calling it cartographer while slam_toolbox is in it
    # would be a lie the UI then repeats.
    LAYERS = ("map", "amcl", "nav2", "explore")

    # Layer -> supervisord program. cartographer and slam_toolbox share one
    # program because they are alternatives, never peers.
    PROGRAM = {"map": "map", "amcl": "amcl", "nav2": "nav", "explore": "explore"}

    # Layer -> what run-layer.sh should launch. The map layer's entry is chosen
    # at start time from slam_backend.
    SLAM_LAUNCH = {"cartographer": "cartographer.launch.py",
                   "slam_toolbox": "slam.launch.py"}
    # Under the workspace, not /tmp. The service runs `docker run --rm`, and
    # /tmp is inside the container, so a layer log written there dies with the
    # container — which is exactly when it is wanted. Only /dev and the
    # workspace are bind-mounted, so the workspace is the one place a log
    # outlives the run that produced it. Falls back to /tmp off the robot,
    # where PICAR_WS is not set.
    # supervisord owns the layer logs now and writes them here; the UI reads
    # the same files it always did, so /api/logs is unchanged.
    LOG_DIR = os.path.join(os.environ.get("PICAR_WS", "/tmp"), "logs")
    RUN_DIR = os.path.join(os.environ.get("PICAR_WS", "/tmp"), "run")

    def __init__(self):
        self.sup = Supervisor()
        # cartographer | slam_toolbox. Both are the "map" layer; which one runs
        # is a choice, so it belongs in state rather than in the launch call.
        self.slam_backend = "cartographer"
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
        self.ws = None              # set by build_app; needed to load waypoints
        self.phase = "idle"
        os.makedirs(self.LOG_DIR, exist_ok=True)
        os.makedirs(self.RUN_DIR, exist_ok=True)

    def adopt(self, link: "RobotLink" = None):
        """Work out the current mode from what supervisord is already running.

        The layers outlive this process now, which is the point — but it means
        a restarted UI must not assume it starts from idle. Without this it
        reports "idle" over a live AMCL and Nav2, and the first thing the user
        does is start a mode that tears down the stack they were using.

        Called once at startup, before anything can change underneath it.
        """
        if not self.sup.available():
            return                       # not under supervisord; nothing to adopt
        mapping = self.running("map")
        localizing = self.running("amcl")
        if mapping and localizing:
            # Both publish /map and map->odom. Whichever is newer is what the
            # user last asked for; stop the other rather than leave two.
            keep = "amcl" if self._started_at("amcl") >= self._started_at("map") else "map"
            drop = "map" if keep == "amcl" else "amcl"
            self._stop(drop)
            mapping, localizing = keep == "map", keep == "amcl"

        if mapping:
            self.mode = "mapping"
            self.slam_backend = self._backend_from_args()
            self.phase = "mapping (adopted)"
        elif localizing:
            self.mode = "localize"
            self.map_name = self._map_name_from_args()
            # _to_localize loads these when a person picks the mode; adoption
            # has to as well, or a restarted UI shows an empty route over a
            # map whose waypoints are sitting on disk. They belong to the map,
            # so the map name is what selects them.
            if link is not None and self.ws and self.map_name:
                link.waypoints = load_waypoints(self.ws, self.map_name)
                logging.info("adopted %d waypoint(s) for %s",
                             len(link.waypoints), self.map_name)
            self.phase = "localized (adopted)"
        else:
            return
        logging.info("adopted running stack: mode=%s map=%s backend=%s",
                     self.mode, self.map_name, self.slam_backend)

        # A mode is more than one layer, and a transition can be cut short —
        # restarting the UI mid-switch leaves the map layer up with Nav2 never
        # started, and nothing afterwards notices. Observed on the robot:
        # adopted "mapping", no Nav2, autonomous controls greyed out forever
        # with the stack looking healthy.
        #
        # prepare_nav2 is the same worker a real transition ends with: it joins
        # the current generation, returns immediately if Nav2 is already up, and
        # waits for a pose before starting it. Safe to call when nothing is
        # missing, which is why it is not conditional on more than this.
        if link is not None and not self.running("nav2"):
            logging.info("adopted stack has no nav2 — completing the mode")
            self.phase = f"{self.phase}, starting nav2"
            threading.Thread(target=self._complete_adopted, args=(link,),
                             daemon=True).start()

    # How long to let TF fill before deciding an adopted stack has no pose.
    # This process starts with an empty buffer while AMCL or cartographer has
    # been publishing all along, so the first lookups fail for reasons that
    # have nothing to do with the robot.
    ADOPT_POSE_WAIT_S = 20.0

    def _complete_adopted(self, link: "RobotLink"):
        """Finish a mode that was adopted without its full layer set.

        Waits for a pose first. ensure_nav2 refuses without one, and in
        localize mode it asks the user to set the position rather than
        waiting — right when a person picked the mode, wrong here, where
        AMCL has been localized all along and only this process is new.
        Observed on the robot: adopted a localized stack and sat on "set the
        robot's position first" with map->base_footprint resolving fine.
        """
        deadline = time.monotonic() + self.ADOPT_POSE_WAIT_S
        while time.monotonic() < deadline:
            if link.pose() is not None:
                break
            time.sleep(0.5)
        else:
            logging.info("adopted stack has no pose after %.0fs — leaving Nav2 "
                         "to the normal path", self.ADOPT_POSE_WAIT_S)
            return
        self.prepare_nav2(link)

    def _started_at(self, name: str) -> int:
        i = self.sup.info(self.PROGRAM[name])
        return int(i.get("start", 0)) if i else 0

    def _read_args(self, name: str) -> str:
        try:
            with open(self._args_path(name)) as f:
                return f.read().strip()
        except OSError:
            return ""

    def _backend_from_args(self) -> str:
        launch = self._read_args("map").split(" ")[0] if self._read_args("map") else ""
        for backend, f in self.SLAM_LAUNCH.items():
            if f == launch:
                return backend
        return self.slam_backend

    def _map_name_from_args(self):
        """Recover which map AMCL was given, so the UI can name it."""
        for tok in self._read_args("amcl").split():
            if tok.startswith("map_yaml:="):
                base = os.path.basename(tok.split(":=", 1)[1])
                return base[:-5] if base.endswith(".yaml") else base
        return None

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

    # supervisord's stdout_logfile per program. nav's is nav2.log because that
    # is the name the logs endpoint and every past debugging session use.
    LOG_FILE = {"map": "map.log", "amcl": "amcl.log",
                "nav2": "nav2.log", "explore": "explore.log"}

    def log_path(self, name: str) -> str:
        return os.path.join(self.LOG_DIR, self.LOG_FILE.get(name, f"{name}.log"))

    def _args_path(self, name: str) -> str:
        return os.path.join(self.RUN_DIR, f"{self.PROGRAM[name]}.args")

    def _write_args(self, name: str, launch_file: str,
                    args: dict[str, str] | None = None):
        """Hand run-layer.sh the launch file and its arguments.

        supervisord programs have fixed commands, but the map layer picks a
        backend and amcl needs whichever map is loaded, so the varying part
        lives in a file the program reads at start. Written before the start
        call, never after: the program reads it once, immediately.
        """
        line = " ".join([launch_file] + [f"{k}:={v}" for k, v in (args or {}).items()])
        os.makedirs(self.RUN_DIR, exist_ok=True)
        tmp = self._args_path(name) + ".tmp"
        with open(tmp, "w") as f:
            f.write(line + "\n")
        os.replace(tmp, self._args_path(name))               # atomic

    def _launch(self, name: str, launch_file: str,
                args: dict[str, str] | None = None):
        if self.running(name):
            self._stop(name)
        self._write_args(name, launch_file, args)
        ok, why = self.sup.start(self.PROGRAM[name])
        if not ok:
            # Recorded rather than raised: the caller's next _wait_until will
            # time out with a phase the user can read, which beats a traceback
            # on the phone.
            self.phase = f"could not start {name}: {why}"

    def running(self, name: str) -> bool:
        i = self.sup.info(self.PROGRAM[name])
        return bool(i) and i.get("statename") in ("RUNNING", "STARTING")

    def exit_code(self, name: str):
        """None while running or never started, else the exit status.

        supervisord reports exitstatus 0 for a program that has not run, so
        STOPPED-without-having-run must not read as a clean exit — detail()
        turns that into "failed" and the UI would show a layer that died.
        """
        i = self.sup.info(self.PROGRAM[name])
        if not i or i.get("statename") in ("RUNNING", "STARTING"):
            return None
        if i.get("statename") == "STOPPED" and not i.get("start"):
            return None                                      # never started
        return i.get("exitstatus")

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
        self.phase = "mapping"
        # The joystick is usable now — that is why the mode does not wait
        # for Nav2. Bring it up in the background so the autonomous
        # controls can enable themselves when it is genuinely ready.
        threading.Thread(target=self.prepare_nav2, args=(link,),
                         daemon=True).start()

    def _to_localize(self, link: "RobotLink", gen: int,
                     map_yaml: str, map_name: str):
        """Caller holds _busy."""
        self.phase = "switching to localize"
        # amcl is in the set because "Change map" re-enters here while AMCL
        # is already up. Without it the second launch overwrites the handle
        # over a running one, the first process group becomes unkillable and two
        # map_server/amcl sets broadcast map→odom at once.
        self._teardown({"explore", "nav2", "map", "amcl"})
        if not self._current(gen):
            return
        link.forget_map()
        link.forget_tf()
        self.mode = "localize"
        self.map_name = map_name
        # forget_map() has just cleared the waypoints, which is right — they
        # belong to whichever map is loaded. Bring back this map's own.
        if self.ws:
            link.waypoints = load_waypoints(self.ws, map_name)

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
        tail = self.log_tail("map", 120)
        return tail.count("Ignored subdivision") >= self.POISON_LINES

    def _start_cartographer(self, link: "RobotLink", gen: int) -> bool:
        """Launch cartographer and wait for its first grid.

        One retry, because the wedge above is occasionally cleared by a
        fresh subscription. If it survives that, only a bringup restart
        helps, so stop burning the user's time and say so.
        """
        for attempt in range(2):
            if not self.running("map"):
                with self._lock:
                    self._launch("map", self.SLAM_LAUNCH[self.slam_backend])

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
                    self._stop("map")
                link.forget_map()

        if self._current(gen):
            self.phase = self._diagnose_no_map()
            # Cartographer is still running and the wedge does clear itself
            # once wall time passes the bogus stamp, so the map can still
            # turn up minutes later. Leaving a failure on screen after it
            # does is worse than the failure.
            threading.Thread(target=self._watch_late_map,
                             args=(link, gen), daemon=True).start()
        return False

    def _watch_late_map(self, link: "RobotLink", gen: int):
        deadline = time.monotonic() + 900.0
        while time.monotonic() < deadline:
            time.sleep(2.0)
            if not self._current(gen) or self.mode != "mapping":
                return
            if link.has_map():
                self.phase = "mapping"
                # _to_mapping returned before it could start Nav2, so pick
                # that up here too — otherwise a late-clearing wedge leaves
                # the autonomous controls greyed out for the whole session.
                threading.Thread(target=self.prepare_nav2, args=(link,),
                                 daemon=True).start()
                return

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
        tail = self.log_tail("map", 200)
        if "Ignored subdivision" in tail:
            # Restarting cartographer does not clear this — the bad scan
            # comes from the lidar driver, which lives in bringup.
            return "lidar timestamps are bad — restart bringup on the robot"
        if not self.running("map"):
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

    def prepare_nav2(self, link: "RobotLink"):
        """Worker: bring Nav2 up now that a pose exists.

        Nav2 cannot start before a pose exists — its costmap blocks on
        map→base_footprint at activation — so this runs at the first moment
        one does: when the map appears while mapping, and when the initial
        pose is set while localizing. Doing it here rather than on the
        first goal is what lets the UI grey out the autonomous controls
        honestly instead of accepting a press it cannot act on for a
        minute. Manual driving keeps working throughout, which is the point
        of not simply blocking the mode on Nav2.
        """
        gen = self._join()
        with self._busy:
            if not self._current(gen) or self.mode not in ("localize",
                                                           "mapping"):
                return
            if self.running("nav2"):
                return
            try:
                if self.ensure_nav2(link, gen) and self._current(gen):
                    self.phase = ("mapping" if self.mode == "mapping"
                                  else "localized")
            except Exception as e:
                self.phase = f"error: {type(e).__name__}: {e}"

    def nav_ready(self, link: "RobotLink") -> bool:
        """Nav2 is up and can take a goal.

        The costmap going ready is not enough on its own — it beats
        bt_navigator's activation by about a third of a second, and a goal
        sent into that gap is rejected outright.
        """
        return (self.running("nav2") and link.costmap_ready()
                and link.nav_server_ready())

    def goal_after_nav2(self, link: "RobotLink", x: float, y: float,
                        yaw: float):
        """Worker: bring Nav2 up, then send the goal that asked for it."""
        gen = self._join()
        with self._busy:
            if not self._current(gen) or self.mode not in ("localize",
                                                           "mapping"):
                return
            try:
                if not self.ensure_nav2(link, gen):
                    return
                # The costmap goes ready before bt_navigator finishes
                # activating — measured at a third of a second apart — and a
                # goal sent into that gap comes back "Action server is
                # inactive. Rejecting the goal." Retry rather than making
                # the user press it again.
                deadline = time.monotonic() + 25.0
                while time.monotonic() < deadline and self._current(gen):
                    ok, err = link.send_goal(x, y, yaw)
                    if not ok:
                        self.phase = err
                        return
                    settle = time.monotonic() + 3.0
                    while (time.monotonic() < settle
                           and link.nav_state == "pending"):
                        time.sleep(0.1)
                    if link.nav_state != "rejected":
                        self.phase = ("mapping" if self.mode == "mapping"
                                      else "localized")
                        return
                    time.sleep(1.0)
                if self._current(gen):
                    self.phase = "navigation would not accept the goal"
            except Exception as e:
                self.phase = f"error: {type(e).__name__}: {e}"

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
        """Blocks until the program is down.

        supervisord signals the whole process group and escalates to SIGKILL
        after stopwaitsecs, which is what killing a ros2 launch tree needs —
        signalling the launch process alone leaves its nodes orphaned and
        still publishing, and two map sources at once is not merely untidy.
        """
        self.sup.stop(self.PROGRAM[name])


def save_map(ws: str, name: str, backend: str = "cartographer") -> tuple[bool, str]:
    """Seal the trajectory, write the pbstream, then the PGM/YAML pair.

    finish_trajectory ends mapping — cartographer cannot resume afterwards,
    which is why the UI calls this 'Finish & save'.
    """
    maps = os.path.join(ws, "maps")
    os.makedirs(maps, exist_ok=True)
    base = os.path.join(maps, name)

    # The two backends seal a map through entirely different services, and
    # calling cartographer's at a live slam_toolbox does not fail fast — it
    # waits out the 120 s service timeout and reports "is cartographer still
    # up?", which is true and useless. Found by the UI test sweep, after the
    # backend selector shipped without anyone saving a slam_toolbox map.
    if backend == "slam_toolbox":
        steps = [
            (
                ["ros2", "service", "call", "/slam_toolbox/serialize_map",
                 "slam_toolbox/srv/SerializePoseGraph",
                 f"{{filename: '{base}'}}"],
                "serialize_map",
            ),
            (
                # Writes the pgm/yaml pair itself, so no separate map_saver.
                ["ros2", "service", "call", "/slam_toolbox/save_map",
                 "slam_toolbox/srv/SaveMap",
                 f"{{name: {{data: '{base}'}}}}"],
                "save_map",
            ),
        ]
    else:
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
    # finish_trajectory is asynchronous: cartographer accepts it and runs the
    # final optimisation in the background, so write_state issued straight
    # after can race a trajectory that is still finishing. Observed on the
    # robot — the first save failed, and a second click 24 s later found
    # "Trajectory 0 already pending finish" and then succeeded. Retrying is
    # the fix rather than a fixed sleep, because how long the optimisation
    # takes depends on how large the map is.
    ATTEMPTS = 4
    BACKOFF_S = 6.0
    for cmd, label in steps:
        last = ""
        for attempt in range(ATTEMPTS):
            try:
                r = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
            except subprocess.TimeoutExpired:
                return False, f"{label} timed out — is cartographer still up?"
            except OSError as e:
                return False, f"{label} could not run: {e}"
            if r.returncode == 0:
                break
            # Both streams: the ros2 CLI puts some failures on stdout, and a
            # message that only reached the browser is a failure nobody can
            # diagnose afterwards — which is how the first occurrence of this
            # very race was lost.
            last = ((r.stderr or "") + (r.stdout or "")).strip()
            logging.warning("save_map: %s attempt %d/%d failed: %s",
                            label, attempt + 1, ATTEMPTS, last[:400])
            if attempt < ATTEMPTS - 1:
                time.sleep(BACKOFF_S)
        else:
            logging.error("save_map: %s gave up after %d attempts", label, ATTEMPTS)
            return False, f"{label} failed after {ATTEMPTS} tries: {last[:200]}"
    return True, base


def waypoints_path(ws: str, map_name: str) -> str:
    return os.path.join(ws, "maps", f"{map_name}.waypoints.yaml")


def save_waypoints(ws: str, map_name: str, wps: list[dict]) -> None:
    """Write a map's waypoints beside its grid.

    Uses the layout scripts/loop_waypoints.py already reads — the RViz Nav2
    panel's "Save Waypoints" format, orientation w-first — so the same file
    drives either the phone or that script.
    """
    if not map_name:
        return
    maps = os.path.join(ws, "maps")
    os.makedirs(maps, exist_ok=True)
    body = {}
    for i, w in enumerate(wps):
        yaw = float(w["yaw"])
        body[f"waypoint{i}"] = {
            "pose": [float(w["x"]), float(w["y"]), 0.0],
            "orientation": [math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)],
            "auto_heading": bool(w.get("auto", False)),
        }
    tmp = waypoints_path(ws, map_name) + ".tmp"
    with open(tmp, "w") as f:
        yaml.safe_dump({"waypoints": body}, f, sort_keys=False)
    os.replace(tmp, waypoints_path(ws, map_name))   # atomic


def load_waypoints(ws: str, map_name: str) -> list[dict]:
    try:
        with open(waypoints_path(ws, map_name)) as f:
            doc = yaml.safe_load(f) or {}
    except OSError:
        return []
    out = []
    for key in sorted((doc.get("waypoints") or {}),
                      key=lambda k: int("".join(c for c in k if c.isdigit()) or 0)):
        w = doc["waypoints"][key]
        try:
            x, y = float(w["pose"][0]), float(w["pose"][1])
            o = w["orientation"]                     # w, x, y, z
            yaw = math.atan2(2.0 * float(o[0]) * float(o[3]),
                             1.0 - 2.0 * float(o[3]) ** 2)
        except (KeyError, IndexError, TypeError, ValueError):
            continue
        out.append({"x": x, "y": y, "yaw": yaw,
                    "auto": bool(w.get("auto_heading", False))})
    return out


def list_maps(ws: str) -> list[dict]:
    """Every saved map, by base name.

    Enumerating only *.pbstream would hide exactly the maps AMCL can use:
    a pgm+yaml pair from slam_toolbox, or one copied in by hand, has no
    pbstream at all. A yaml only counts when its image sits beside it,
    which keeps stray config files out of the picker — maps/ also collects
    slam_toolbox .posegraph/.data pairs.
    """
    maps = os.path.join(ws, "maps")
    if not os.path.isdir(maps):
        return []

    found: dict[str, dict] = {}
    names = sorted(os.listdir(maps))
    for f in names:
        if f.endswith(".pbstream"):
            found.setdefault(f[: -len(".pbstream")], {})["pbstream"] = f
        elif f.endswith(".yaml"):
            base = f[: -len(".yaml")]
            if any(os.path.exists(os.path.join(maps, base + ext))
                   for ext in (".pgm", ".png")):
                found.setdefault(base, {})["grid"] = f

    out = []
    for name in sorted(found):
        which = found[name]
        # Age from the yaml, since that is what AMCL loads and what the save
        # wrote last. Size from the image or the pbstream — the yaml is a
        # couple of hundred bytes and would show as 0.0 MB.
        ref = which.get("grid") or which.get("pbstream")
        big = next((f"{name}{ext}" for ext in (".pgm", ".png")
                    if os.path.exists(os.path.join(maps, name + ext))),
                   which.get("pbstream") or ref)
        try:
            size = os.path.getsize(os.path.join(maps, big))
            mtime = int(os.path.getmtime(os.path.join(maps, ref)))
        except OSError:
            continue
        out.append({
            "name": name,
            "size": size,
            "mtime": mtime,
            "has_grid": "grid" in which,
            "has_pbstream": "pbstream" in which,
        })
    return out


def build_app(link: RobotLink, modes: ModeStack, ws: str, root: str) -> Flask:
    app = Flask(__name__, static_folder=None)
    modes.ws = ws

    def persist():
        """Waypoints belong to a map, so they are only saveable once one is
        named. In mapping mode the map has no name until it is saved, and
        mapping_save writes them out at that point."""
        if modes.mode == "localize" and modes.map_name:
            try:
                save_waypoints(ws, modes.map_name, link.waypoints)
            except OSError:
                pass

    speed_probe = threading.Event()

    def _probe_speed():
        try:
            link.refresh_max_speed()
        finally:
            speed_probe.clear()

    def body() -> dict:
        """request.json is whatever was sent — a bare string or list would
        make .get() raise and turn a bad request into a 500."""
        try:
            b = request.get_json(silent=True)
        except Exception:
            b = None
        return b if isinstance(b, dict) else {}

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
        if link.max_speed is None and modes.running("nav2"):
            # Off the request thread: the parameter service is far too slow
            # to sit inside a poll that runs twice a second. One flight at a
            # time, and forget_nav() clears the cache when Nav2 goes away.
            if not speed_probe.is_set():
                speed_probe.set()
                threading.Thread(target=_probe_speed, daemon=True).start()
        return jsonify({
            "mode": modes.mode,
            "map_name": modes.map_name,
            "localized": modes.mode == "localize" and pose is not None,
            "modes": modes.status(),
            "slam_backend": modes.slam_backend,
            "detail": modes.detail(),
            "phase": modes.phase,
            "explore_status": link.explore_status,
            "pose": pose,
            # Computed on every failed lookup and, until now, visible nowhere:
            # a null pose is the symptom of a broken TF chain and this says
            # where the break is. Diagnosing a sim-only pose failure without it
            # took a detour through tf2_echo.
            "pose_error": link.pose_error,
            "map_seq": seq,
            "has_map": grid is not None,
            "nav": link.nav_status(),
            "nav_ready": modes.nav_ready(link),
            "route": link.route_status(),
            "max_speed": link.max_speed,
            "drive_limits": list(link.drive_limits()),
            "speed_range": [link.SPEED_MIN, link.SPEED_MAX],
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

    @app.route("/api/costmap")
    def get_costmap():
        """Marked cells of the local costmap, as raw float32 map-frame xy.

        Binary rather than JSON: a 3 m window at 5 cm is 3600 cells, and in a
        cluttered room enough of them are marked that the JSON of the same
        thing is several times the size, fetched twice a second.
        """
        pts, why = link.local_obstacles()
        if pts is None:
            return (why, 404)
        resp = Response(pts, mimetype="application/octet-stream")
        resp.headers["X-Count"] = str(len(pts) // 8)
        # The drawn square is one costmap cell, so the client needs the
        # costmap's resolution, not the map's — they are configured apart.
        resp.headers["X-Cell"] = str(link.local_cell_size())
        return resp

    @app.route("/api/mode", methods=["POST"])
    def set_mode():
        b = body()
        mode = b.get("mode", "idle")
        if mode not in ("idle", "mapping", "localize"):
            return jsonify({"ok": False, "error": f"unknown mode {mode!r}"}), 400

        map_yaml = map_name = None
        if mode == "localize":
            map_name = (b.get("map") or "").strip()
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

    @app.route("/api/initialpose", methods=["POST"])
    def initial_pose():
        if modes.mode != "localize":
            return jsonify({"ok": False,
                            "error": "only in localize mode"}), 409
        b = body()
        try:
            x = float(b["x"]); y = float(b["y"]); yaw = float(b["yaw"])
        except (KeyError, TypeError, ValueError):
            return jsonify({"ok": False, "error": "need x, y and yaw"}), 400

        ok, err = link.set_initial_pose(x, y, yaw)
        if ok:
            modes.phase = "localized"
            # Now that a pose exists Nav2 can start, so get on with it
            # rather than making the first goal wait a minute for it.
            threading.Thread(target=modes.prepare_nav2, args=(link,),
                             daemon=True).start()
            return jsonify({"ok": True})
        # 503 if AMCL is not there yet, 504 if it is but did not take it.
        code = 503 if "isn't running" in err else 504
        return jsonify({"ok": False, "error": err}), code

    @app.route("/api/goal", methods=["POST"])
    def goal():
        if modes.mode not in ("localize", "mapping"):
            return jsonify({"ok": False,
                            "error": "start a mode first"}), 409
        b = body()
        try:
            x = float(b["x"]); y = float(b["y"]); yaw = float(b["yaw"])
        except (KeyError, TypeError, ValueError):
            return jsonify({"ok": False, "error": "need x, y and yaw"}), 400
        if link.pose() is None:
            return jsonify({"ok": False,
                            "error": ("set the robot's position first"
                                      if modes.mode == "localize"
                                      else "waiting for the map")}), 409

        # explore_lite drives by sending its own NavigateToPose goals, so
        # leaving it running would have the two of you preempting each other
        # every few seconds. Choosing a destination by hand means you are
        # steering now.
        if modes.running("explore"):
            modes.stop_explore()
        link.route_stop()

        if not modes.running("nav2"):
            # First goal in this mode brings Nav2 up, which is far too slow
            # for a request. It could not have been started earlier: Nav2's
            # costmap needs map→base_footprint at activation, and AMCL only
            # publishes that once an initial pose has been set.
            threading.Thread(target=modes.goal_after_nav2,
                             args=(link, x, y, yaw), daemon=True).start()
            return jsonify({"ok": True, "starting": True}), 202

        ok, err = link.send_goal(x, y, yaw)
        if not ok:
            return jsonify({"ok": False, "error": err}), 503
        return jsonify({"ok": True}), 202

    @app.route("/api/waypoint", methods=["POST"])
    def waypoint_add():
        if modes.mode not in ("localize", "mapping"):
            return jsonify({"ok": False, "error": "start a mode first"}), 409
        b = body()
        try:
            x = float(b["x"]); y = float(b["y"]); yaw = float(b["yaw"])
        except (KeyError, TypeError, ValueError):
            return jsonify({"ok": False, "error": "need x, y and yaw"}), 400
        count = link.add_waypoint(x, y, yaw, bool(b.get("auto", False)))
        persist()
        return jsonify({"ok": True, "count": count})

    @app.route("/api/waypoint/undo", methods=["POST"])
    def waypoint_undo():
        link.drop_last_waypoint()
        persist()
        return jsonify({"ok": True, "count": len(link.waypoints)})

    @app.route("/api/waypoints/clear", methods=["POST"])
    def waypoints_clear():
        link.clear_waypoints()
        persist()
        return jsonify({"ok": True})

    @app.route("/api/route/start", methods=["POST"])
    def route_start():
        if modes.mode not in ("localize", "mapping"):
            return jsonify({"ok": False, "error": "start a mode first"}), 409
        if not modes.nav_ready(link):
            return jsonify({"ok": False,
                            "error": "navigation isn't ready yet"}), 409
        # explore_lite drives by sending its own goals; it would fight the
        # route exactly as it fights a single one.
        if modes.running("explore"):
            modes.stop_explore()
        b = body()
        ok, err = link.route_start(bool(b.get("loop", False)),
                                   bool(b.get("flow", False)))
        if not ok:
            return jsonify({"ok": False, "error": err}), 409
        return jsonify({"ok": True}), 202

    @app.route("/api/slam", methods=["POST"])
    def set_slam():
        """Choose the SLAM backend for the next mapping session.

        Refused while mapping: switching backends under a live map would mean
        tearing down the thing that is holding the pose graph, and a half-built
        map is not something to lose to a stray tap.
        """
        backend = (body().get("backend") or "").strip()
        if backend not in ModeStack.SLAM_LAUNCH:
            return jsonify({"ok": False,
                            "error": f"unknown backend {backend!r}"}), 400
        if modes.mode == "mapping":
            return jsonify({"ok": False,
                            "error": "stop mapping before switching backend"}), 409
        modes.slam_backend = backend
        return jsonify({"ok": True, "slam_backend": backend})

    @app.route("/api/speed", methods=["POST"])
    def set_speed():
        if not modes.running("nav2"):
            return jsonify({"ok": False,
                            "error": "navigation isn't running"}), 409
        try:
            value = body()["value"]
        except (KeyError, TypeError):
            return jsonify({"ok": False, "error": "need a value"}), 400
        ok, err = link.set_max_speed(value)
        if not ok:
            return jsonify({"ok": False, "error": err}), 409
        return jsonify({"ok": True, "max_speed": link.max_speed})

    @app.route("/api/route/stop", methods=["POST"])
    def route_stop():
        link.route_stop()
        return jsonify({"ok": True})

    @app.route("/api/goal/cancel", methods=["POST"])
    def goal_cancel():
        link.cancel_goal()
        return jsonify({"ok": True})

    @app.route("/api/takeover", methods=["POST"])
    def takeover():
        # The human has the wheel. Deciding here rather than in the client
        # avoids branching on state that is up to half a second stale —
        # exactly the case where it would fail to cancel.
        if modes.running("explore"):
            modes.stop_explore()
        link.route_stop()
        link.cancel_goal()
        return ("", 204)

    @app.route("/api/explore", methods=["POST"])
    def explore():
        on = bool(body().get("on", False))
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
        name = str(body().get("name", "")).strip()
        if not name or "/" in name or name.startswith("."):
            return jsonify({"ok": False, "error": "bad map name"}), 400
        if modes.mode != "mapping":
            return jsonify({"ok": False, "error": "only while mapping"}), 409
        if not modes.running("map") or not link.has_map():
            # finish_trajectory would block until its 120 s timeout, which
            # surfaces as a raw 500 the client cannot parse.
            return jsonify({"ok": False,
                            "error": f"{modes.slam_backend} isn't mapping"}), 409

        # Stop exploring first so the robot is still while the graph is sealed.
        modes.stop_explore()
        ok, detail = save_map(ws, name, modes.slam_backend)
        if ok and link.waypoints:
            # The route was placed on this map while mapping it, so it keeps
            # its meaning once the map has a name.
            try:
                save_waypoints(ws, name, link.waypoints)
            except OSError:
                pass
        return jsonify({"ok": ok, "detail": detail}), (200 if ok else 500)

    @app.route("/api/explore/resume", methods=["POST"])
    def explore_resume():
        link.resume_explore()
        return jsonify({"ok": True})

    @app.route("/api/logs")
    def logs():
        layer = request.args.get("layer", "map")
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
        b = body()
        try:
            link.set_drive(float(b.get("linear", 0.0)),
                           float(b.get("angular", 0.0)))
        except (TypeError, ValueError):
            return jsonify({"ok": False, "error": "linear and angular must be numbers"}), 400
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

    # supervisord captures stdout to logs/webui.log; without this the
    # warnings save_map emits never appear anywhere.
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname)s %(message)s")
    rclpy.init()
    link = RobotLink()
    modes = ModeStack()
    # build_app sets this too, but that runs after adopt() and adoption needs
    # it to find the map's saved waypoints. Setting it here rather than
    # reordering, so there is one obvious place the workspace is known.
    modes.ws = args.ws

    spin = threading.Thread(target=rclpy.spin, args=(link,), daemon=True)
    spin.start()

    # After spin, before serving: the layers outlive this process, so find out
    # what is already running rather than reporting idle over a live stack.
    # After spin because completing an interrupted transition needs a pose,
    # and a pose needs TF being processed.
    modes.adopt(link)

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
