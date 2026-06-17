#!/usr/bin/env python3
"""Loop a list of waypoints through Nav2's NavigateThroughPoses action,
using a rolling window so the robot never stops between laps.

YAML format matches the RViz Nav2 panel "Save Waypoints" output:

    waypoints:
      waypoint0:
        pose: [x, y, z]
        orientation: [w, x, y, z]    # NOTE: w first (RViz panel convention)
      waypoint1:
        ...

Behavior:
  * Initially send the next `window` waypoints (default 3) starting from
    the one after the robot's current pose.
  * Subscribe to action feedback; when `number_of_poses_remaining`
    decreases (Nav2's BT trims goals as they're passed), slide the
    window forward and send a new goal — this preempts the previous one
    so the robot continues without stopping.
  * The path endpoint is always a couple of waypoints ahead, so the
    controller's goal-checker can't shortcut to SUCCESS.

Run inside the picar2 container (rclpy + nav2_msgs + tf2_ros available):

    python3 scripts/loop_waypoints.py f1.yaml                  # forever
    python3 scripts/loop_waypoints.py f1.yaml --laps 3         # 3 cycles
    python3 scripts/loop_waypoints.py f1.yaml --window 4       # bigger lookahead

Ctrl-C cancels the in-flight goal cleanly.
"""

import argparse
import math
import re
import sys

import rclpy
import tf2_ros
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseArray, PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray


# Tunable: robot within this radius of a waypoint is considered "at" it.
# Used only at startup to pick where the first window starts.
ANCHOR_RADIUS_M = 0.5


_NUM_RE = re.compile(r'(\d+)')


def _natural_key(s: str):
    return [int(t) if t.isdigit() else t for t in _NUM_RE.split(s)]


def load_poses(yaml_path: str, frame_id: str) -> list[PoseStamped]:
    with open(yaml_path) as f:
        data = yaml.safe_load(f)
    raw = data.get('waypoints') or {}
    poses: list[PoseStamped] = []
    for key in sorted(raw, key=_natural_key):
        wp = raw[key]
        x, y, z = wp['pose']
        qw, qx, qy, qz = wp['orientation']
        ps = PoseStamped()
        ps.header.frame_id = frame_id
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.position.z = float(z)
        ps.pose.orientation.w = float(qw)
        ps.pose.orientation.x = float(qx)
        ps.pose.orientation.y = float(qy)
        ps.pose.orientation.z = float(qz)
        poses.append(ps)
    return poses


class WaypointLooper(Node):
    def __init__(self, poses: list[PoseStamped], laps: int, window: int,
                 max_retries: int, fail_limit: int):
        super().__init__('loop_waypoints')
        self.poses = poses
        self.n = len(poses)
        self.laps = laps                    # 0 = forever; otherwise N full cycles
        self.window = max(1, min(window, self.n))
        self.max_retries = max(0, max_retries)
        self.fail_limit = fail_limit        # 0 = no cap on skipped waypoints
        self.cycle_idx = 0                  # index of the NEXT waypoint to send
        self.total_passed = 0               # cumulative waypoints passed
        self.window_retries = 0             # retries on the CURRENT lead waypoint
        self.total_failures = 0             # cumulative ABORTED windows
        self.last_remaining = None          # feedback: poses_remaining tracker
        self.last_recoveries = 0            # feedback: recoveries triggered so far
        self.goal_handle = None
        # Monotonically-increasing send id. Every _send_window bumps it; the
        # feedback callback we register also captures it, so feedback from a
        # preempted (stale) goal can identify itself and bail out.
        self.goal_seq = 0

        self.client = ActionClient(self, NavigateThroughPoses,
                                   '/navigate_through_poses')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.map_frame = poses[0].header.frame_id
        self.robot_frame = 'base_footprint'

        latched = QoSProfile(depth=1,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.poses_pub = self.create_publisher(
            PoseArray, '/loop_waypoints/poses', latched)
        self.markers_pub = self.create_publisher(
            MarkerArray, '/loop_waypoints/markers', latched)
        self._publish_visualization()

    # ── lifecycle ──────────────────────────────────────────────────────────
    def start(self):
        self.get_logger().info('waiting for /navigate_through_poses action server...')
        self.client.wait_for_server()
        self._init_cycle()
        self._send_window()

    def cancel(self):
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()

    def _cancel_and_shutdown(self):
        """Cancel the in-flight goal (if any), then shutdown rclpy once the
        cancellation result lands. If we shutdown without cancelling,
        Nav2 keeps executing the active goal and the robot drives out the
        rest of the window."""
        if self.goal_handle is None:
            rclpy.shutdown()
            return
        self.get_logger().info('cancelling in-flight goal before shutdown...')
        # Bump goal_seq so any remaining feedback / result from this goal
        # gets dropped by the stale filter and we don't re-enter
        # _send_window from a slide.
        self.goal_seq += 1
        self.goal_handle.cancel_goal_async().add_done_callback(
            lambda _: rclpy.shutdown())

    def _init_cycle(self):
        """Pick the starting cycle_idx: if the robot is already at a
        waypoint, start with the next one; otherwise start at 0."""
        xy = self._robot_xy()
        if xy is None:
            return
        dists = [math.hypot(p.pose.position.x - xy[0],
                            p.pose.position.y - xy[1]) for p in self.poses]
        closest = min(range(self.n), key=lambda i: dists[i])
        if dists[closest] < ANCHOR_RADIUS_M:
            self.cycle_idx = (closest + 1) % self.n
            self.get_logger().info(
                f'robot at wp {closest}, starting window from wp {self.cycle_idx}')

    # ── goal sending ──────────────────────────────────────────────────────
    def _send_window(self):
        if self.laps and self.total_passed >= self.laps * self.n:
            self.get_logger().info(
                f'completed {self.laps} full cycle(s) '
                f'({self.total_passed} waypoints), shutting down')
            # Cancel the goal that's still active on Nav2's side — without
            # this the robot keeps driving the remaining poses of the last
            # window we sent (which is exactly `self.window` extra points).
            self._cancel_and_shutdown()
            return

        window = [self.poses[(self.cycle_idx + i) % self.n]
                  for i in range(self.window)]
        now = self.get_clock().now().to_msg()
        for p in window:
            p.header.stamp = now

        # Reset feedback trackers for the new goal. Stale feedback from a
        # preempted old goal can't make us double-advance because we ignore
        # any remaining value > the first one we see on the new goal.
        self.last_remaining = None
        self.last_recoveries = 0

        target_idxs = [(self.cycle_idx + i) % self.n for i in range(self.window)]
        retry_tag = f' [retry {self.window_retries}/{self.max_retries}]' \
            if self.window_retries else ''
        self.get_logger().info(
            f'sending window {target_idxs}{retry_tag} '
            f'(passed {self.total_passed}'
            f'{f"/{self.laps * self.n}" if self.laps else ""})')

        self.goal_seq += 1
        my_seq = self.goal_seq

        def _fb_cb(msg, seq=my_seq):
            # Drop feedback from any goal that's already been preempted.
            if seq != self.goal_seq:
                return
            self._on_feedback(msg)

        goal = NavigateThroughPoses.Goal(poses=window)
        self.client.send_goal_async(
            goal, feedback_callback=_fb_cb
        ).add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().error(
                'goal REJECTED by /navigate_through_poses server '
                '(server may be in shutdown / lifecycle inactive); retrying in 1 s')
            self.create_timer(1.0, self._send_window)
            return
        self.goal_handle = gh
        # Capture *this* handle in the closure so we can tell, when the
        # result eventually arrives, whether it's for the goal we still
        # care about. Preempted goals also fire their result callback
        # (Nav2 reports them as ABORTED with empty error info on jazzy),
        # and without this check we'd treat each preempt as a real
        # failure and start retrying — cascading instantly.
        gh.get_result_async().add_done_callback(
            lambda f, h=gh: self._on_result(f, h))

    # ── progress tracking ─────────────────────────────────────────────────
    def _on_feedback(self, msg):
        fb = msg.feedback
        rem = fb.number_of_poses_remaining
        recs = fb.number_of_recoveries

        # Surface Nav2's own recovery attempts (costmap clears, backups …).
        # They mean Nav2 is having trouble — useful breadcrumb when laps
        # later end up ABORTED.
        if recs > self.last_recoveries:
            self.get_logger().warn(
                f'nav2 triggered recovery #{recs} '
                f'(dist_remaining={fb.distance_remaining:.2f} m, '
                f'rem_poses={rem})')
            self.last_recoveries = recs

        if self.last_remaining is None:
            self.last_remaining = rem
            return
        if rem >= self.last_remaining:
            return
        # One or more waypoints just got trimmed by RemovePassedGoals — slide.
        n_passed = self.last_remaining - rem
        self.cycle_idx = (self.cycle_idx + n_passed) % self.n
        self.total_passed += n_passed
        self.window_retries = 0   # actual progress resets the retry counter
        self.get_logger().info(
            f'passed {n_passed} waypoint(s) (rem {self.last_remaining}→{rem}); '
            f'sliding window forward')
        # Preempts the in-flight goal. The cancelled old goal's result will
        # fire later — we ignore CANCELED in _on_result.
        self._send_window()

    def _on_result(self, future, my_handle):
        # If this result is for a goal we've already preempted with a newer
        # send, drop it. Nav2 jazzy fires preempted goals as ABORTED with
        # empty error info; without this check we'd mis-read that as a
        # real failure and start retrying — and each retry preempts the
        # new in-flight goal, cascading.
        if my_handle is not self.goal_handle:
            return

        result = future.result()
        status = result.status
        nav_result = result.result   # NavigateThroughPoses.Result
        error_code = getattr(nav_result, 'error_code', 0) or 0
        error_msg = getattr(nav_result, 'error_msg', '') or ''

        if status == GoalStatus.STATUS_CANCELED:
            # Server returned canceled (rare for bt_navigator on jazzy
            # but possible) — silent.
            return

        if status == GoalStatus.STATUS_SUCCEEDED:
            # The stale-handle filter above guarantees we only get here for
            # the *current* goal. SUCCEEDED on the current goal means Nav2
            # navigated through every pose in the window — no slides
            # happened mid-flight (those would have moved on to a newer
            # handle and this result would be stale). Advance by the full
            # window size; trying to deduce it from `last_remaining` is
            # off-by-one when the final rem→0 transition lands on SUCCESS
            # without a separate feedback message.
            self.cycle_idx = (self.cycle_idx + self.window) % self.n
            self.total_passed += self.window
            self.window_retries = 0
            self.get_logger().info(
                f'window completed cleanly (advanced {self.window}); next window')
            self._send_window()
            return

        # STATUS_ABORTED (6) or anything unexpected → real failure.
        lead_idx = self.cycle_idx
        self.get_logger().error(
            f'goal FAILED status={status} error_code={error_code} '
            f'msg="{error_msg}" at lead waypoint wp{lead_idx} '
            f'after {self.last_recoveries} nav2-internal recoveries')

        self.window_retries += 1
        if self.window_retries <= self.max_retries:
            self.get_logger().warn(
                f'retrying same window (attempt {self.window_retries}/{self.max_retries})')
            self._send_window()
            return

        # Give up on the lead waypoint, skip it, log loudly.
        self.total_failures += 1
        self.cycle_idx = (self.cycle_idx + 1) % self.n
        self.window_retries = 0
        self.get_logger().error(
            f'GIVING UP on wp{lead_idx} after {self.max_retries} retries; '
            f'skipping it (total skipped: {self.total_failures}'
            f'{f"/{self.fail_limit}" if self.fail_limit else ""})')

        if self.fail_limit and self.total_failures >= self.fail_limit:
            self.get_logger().error(
                f'reached --fail-limit={self.fail_limit}; shutting down')
            self._cancel_and_shutdown()
            return
        self._send_window()

    # ── helpers ───────────────────────────────────────────────────────────
    def _robot_xy(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.5))
            return t.transform.translation.x, t.transform.translation.y
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(
                f'TF {self.map_frame}→{self.robot_frame} unavailable: {e}')
            return None

    def _publish_visualization(self) -> None:
        frame_id = self.poses[0].header.frame_id
        now = self.get_clock().now().to_msg()

        pa = PoseArray()
        pa.header.frame_id = frame_id
        pa.header.stamp = now
        pa.poses = [p.pose for p in self.poses]
        self.poses_pub.publish(pa)

        ma = MarkerArray()
        clear = Marker()
        clear.header.frame_id = frame_id
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)
        for i, p in enumerate(self.poses):
            sphere = Marker()
            sphere.header.frame_id = frame_id
            sphere.header.stamp = now
            sphere.ns = 'waypoints'
            sphere.id = i
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose = p.pose
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.18
            sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = 0.2, 1.0, 0.2, 0.9
            ma.markers.append(sphere)

            label = Marker()
            label.header = sphere.header
            label.ns = 'waypoint_labels'
            label.id = i
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose = p.pose
            label.pose.position.z = p.pose.position.z + 0.25
            label.scale.z = 0.15
            label.color.r, label.color.g, label.color.b, label.color.a = 1.0, 1.0, 1.0, 0.95
            label.text = str(i)
            ma.markers.append(label)
        self.markers_pub.publish(ma)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('yaml', help='YAML file with waypoints')
    parser.add_argument('--frame', default='map',
                        help="Frame ID for the poses (default 'map')")
    parser.add_argument('--laps', type=int, default=0,
                        help='Stop after N full cycles through the list; '
                             '0 = forever (default)')
    parser.add_argument('--window', type=int, default=3,
                        help='Rolling-window size: how many waypoints to send '
                             'ahead of the robot (default 3)')
    parser.add_argument('--max-retries', type=int, default=2,
                        help='When a window ends ABORTED, retry the same '
                             'window this many times before skipping its lead '
                             'waypoint and continuing (default 2)')
    parser.add_argument('--fail-limit', type=int, default=0,
                        help='Abort the script after this many total skipped '
                             'waypoints. 0 = never give up (default)')
    args = parser.parse_args()

    poses = load_poses(args.yaml, args.frame)
    if not poses:
        print(f'no waypoints found in {args.yaml}', file=sys.stderr)
        sys.exit(1)
    print(f'loaded {len(poses)} waypoints from {args.yaml}')

    rclpy.init()
    node = WaypointLooper(poses, args.laps, args.window,
                          args.max_retries, args.fail_limit)
    try:
        node.start()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\ncancelling current goal...')
        node.cancel()
        rclpy.spin_once(node, timeout_sec=1.0)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
