#!/usr/bin/env python3
"""
IMU sensor verification — Layers 3–5 (ROS2).

Requires bringup running on the Pi (make bringup).
Run from inside the picar2 Docker container or with ROS2 sourced.

Layer 3 — /imu/data_raw      (axis remap in picar2_hardware.cpp; tilt visible)
Layer 3D — /imu/data_corrected (imu_calib accel scale+bias; tilt absorbed)
Layer 4 — /imu/data           (Madgwick quaternion)
Layer 5 — /odom / EKF         (motor-driven odometry checks)

Tests (--tests):
    3A  /imu/data_raw static — rate, accel, gyro, mag magnitude
    3B  /imu/data_raw gyro sign — motor spin CW/CCW        [motion]
    3C  /imu/data_raw mag circle — motor arc               [motion]
    3D  /imu/data_corrected static — accel.x/y after imu_calib tilt+bias correction
    4A  /imu/data static — orientation, roll/pitch, gyro passthrough
    4B  /imu/data yaw drift — 60 s static
    4C  /imu/data gyro sign — motor spin                   [motion]
    5A  /odom static drift — 60 s
    5B  /odom gyro.z sign — motor CW spin                  [motion]
    5C  /odom straight line — ~1 m                         [motion]
    5D  /odom 90° right turn accuracy                      [motion]

Magnetometer comparison:
    use_mag is read-only at runtime in imu_filter_madgwick. To compare:
    1. Set use_mag in bringup config, relaunch, run --tests 4B → note yaw drift
    2. Toggle use_mag, relaunch, run --tests 4B again → compare

Usage:
    python3 scripts/imu_verify.py                         # default: 3A only
    python3 scripts/imu_verify.py --tests 3A 3D           # raw + corrected static
    python3 scripts/imu_verify.py --tests 3A 3B 3C        # layer 3 all
    python3 scripts/imu_verify.py --tests 3A 4A 4B        # layers 3+4 static
    python3 scripts/imu_verify.py --tests 3A 3B 3C 3D 4A 4B 4C 5A 5B 5C 5D  # all
"""

import argparse
import math
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, MagneticField


# ── Motor speeds for motion tests ────────────────────────────────────────────
# Forward-arc avoids spin-in-place wheel slip (same approach as imu_verify_uart.py).
# At low cmd values the controller barely overcomes static friction;
# these match the Layer 2 UART test (500/200 DPS outer/inner wheel).
ARC_LINEAR   = 0.3   # m/s forward — gyro sign and odom sign tests
ARC_ANGULAR  = 1.5   # rad/s yaw   — gyro sign and odom sign tests
CIRCLE_LINEAR  = 0.1 # m/s forward — mag circle (slow arc)
CIRCLE_ANGULAR = 0.3 # rad/s yaw   — mag circle
DRIVE_LINEAR   = 0.2 # m/s         — straight-line test

# ── Pass / fail output ────────────────────────────────────────────────────────

_G, _R, _Y, _RST = '\033[32m', '\033[31m', '\033[33m', '\033[0m'
_all_passed = True


def _tag(ok: bool) -> str:
    return f'{_G}PASS{_RST}' if ok else f'{_R}FAIL{_RST}'


def _record(ok: bool, name: str, got: str, expected: str) -> bool:
    global _all_passed
    print(f'  [{_tag(ok)}] {name}: {got}  (expected {expected})')
    if not ok:
        _all_passed = False
    return ok


def check(name, val, lo, hi, fmt='.4f', unit=''):
    return _record(lo <= val <= hi, name,
                   f'{val:{fmt}}{unit}', f'{lo} – {hi}{unit}')


def check_lt(name, val, limit, fmt='.4f', unit=''):
    return _record(val < limit, name,
                   f'{val:{fmt}}{unit}', f'< {limit}{unit}')


def check_gt(name, val, limit, fmt='.4f', unit=''):
    return _record(val > limit, name,
                   f'{val:{fmt}}{unit}', f'> {limit}{unit}')


def check_warn(name, val, lo, hi, fmt='.4f', unit='', note='needs calibration'):
    """Range check that emits WARN (not FAIL) when out of range.
    Use for values fixable by calibration rather than code changes."""
    ok = lo <= val <= hi
    tag = f'{_G}PASS{_RST}' if ok else f'{_Y}WARN{_RST}'
    suffix = '' if ok else f'  ← {note}'
    print(f'  [{tag}] {name}: {val:{fmt}}{unit}  (expected {lo} – {hi}{unit}){suffix}')
    return ok


def warn(msg: str):
    print(f'  [{_Y}WARN{_RST}] {msg}')


# ── Statistics ────────────────────────────────────────────────────────────────

def mean(xs): return sum(xs) / len(xs)

def std(xs):
    if len(xs) < 2:
        return 0.0
    m = mean(xs)
    return math.sqrt(sum((x - m) ** 2 for x in xs) / (len(xs) - 1))


def quat_to_rpy(q):
    """Convert geometry_msgs quaternion to (roll, pitch, yaw) in radians."""
    x, y, z, w = q.x, q.y, q.z, q.w
    roll  = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    pitch = math.asin(max(-1.0, min(1.0, 2*(w*y - z*x))))
    yaw   = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return roll, pitch, yaw


def angle_diff(a, b):
    """Shortest angular distance a→b, result in (-π, π]."""
    d = (b - a) % (2 * math.pi)
    return d - 2 * math.pi if d > math.pi else d


# ── Data collector node ───────────────────────────────────────────────────────

class Collector(Node):
    def __init__(self, imu_raw='/imu/data_raw', imu_flt='/imu/data',
                 imu_corrected='/imu/data_corrected'):
        super().__init__('imu_verify')
        self._lock       = threading.Lock()
        self.imu_raw     = []   # (stamp_s, msg)
        self.imu_flt     = []   # (stamp_s, msg)
        self.imu_corrected = [] # (stamp_s, msg)
        self.mag         = []   # (stamp_s, msg)
        self.odom        = []   # (stamp_s, msg)

        qos = rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value
        self.create_subscription(Imu,          imu_raw,       self._cb_raw,       qos)
        self.create_subscription(Imu,          imu_flt,       self._cb_flt,       qos)
        self.create_subscription(Imu,          imu_corrected, self._cb_corrected, qos)
        self.create_subscription(MagneticField,'/imu/mag',    self._cb_mag,       qos)
        self.create_subscription(Odometry,     '/odom',       self._cb_odom,      qos)

        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    # -- callbacks
    def _cb_raw(self, msg):
        with self._lock:
            self.imu_raw.append((time.monotonic(), msg))

    def _cb_flt(self, msg):
        with self._lock:
            self.imu_flt.append((time.monotonic(), msg))

    def _cb_corrected(self, msg):
        with self._lock:
            self.imu_corrected.append((time.monotonic(), msg))

    def _cb_mag(self, msg):
        with self._lock:
            self.mag.append((time.monotonic(), msg))

    def _cb_odom(self, msg):
        with self._lock:
            self.odom.append((time.monotonic(), msg))

    # -- helpers
    def clear(self):
        with self._lock:
            self.imu_raw.clear()
            self.imu_flt.clear()
            self.imu_corrected.clear()
            self.mag.clear()
            self.odom.clear()

    def snap_raw(self):
        with self._lock: return list(self.imu_raw)

    def snap_flt(self):
        with self._lock: return list(self.imu_flt)

    def snap_corrected(self):
        with self._lock: return list(self.imu_corrected)

    def snap_mag(self):
        with self._lock: return list(self.mag)

    def snap_odom(self):
        with self._lock: return list(self.odom)

    def drive(self, linear_x: float, angular_z: float):
        msg = Twist()
        msg.linear.x  = linear_x
        msg.angular.z = angular_z
        self._cmd_pub.publish(msg)

    def stop(self):
        self.drive(0.0, 0.0)

    def drive_return(self, linear_x: float, angular_z: float, duration_s: float) -> None:
        """Approximately undo a prior motion: drive (-linear_x, -angular_z) for duration_s."""
        t0 = time.monotonic()
        while time.monotonic() - t0 < duration_s:
            self.drive(-linear_x, -angular_z)
            rclpy.spin_once(self, timeout_sec=0.02)
        self.stop()
        time.sleep(0.3)

    def spin_for(self, duration_s: float, collect_fn=None):
        """spin_once in a loop for duration_s, optionally calling collect_fn."""
        t0 = time.monotonic()
        while time.monotonic() - t0 < duration_s:
            rclpy.spin_once(self, timeout_sec=0.02)


def wait_collect(node: Collector, topic: str, count: int,
                 timeout_s: float = 15.0, motor_cmd=None) -> list:
    """
    Wait until `count` messages arrive on `topic` (or timeout).
    motor_cmd=(linear_x, angular_z): republish /cmd_vel at 10 Hz to keep
    AckermannSteeringController alive (reference_timeout = 0.5 s).
    Returns list of (timestamp, msg).
    """
    snap_fn = {
        'raw':       node.snap_raw,
        'corrected': node.snap_corrected,
        'flt':       node.snap_flt,
        'mag':       node.snap_mag,
        'odom':      node.snap_odom,
    }[topic]

    node.clear()
    t0         = time.monotonic()
    last_cmd_t = -1.0

    while time.monotonic() - t0 < timeout_s:
        rclpy.spin_once(node, timeout_sec=0.02)

        now = time.monotonic()
        if motor_cmd is not None and now - last_cmd_t >= 0.1:
            node.drive(*motor_cmd)
            last_cmd_t = now

        if len(snap_fn()) >= count:
            break

    node.stop()
    return snap_fn()


# ── Layer 3 — /imu/data_raw ───────────────────────────────────────────────────

def test_layer3_static(node: Collector) -> bool:
    print('\n── Layer 3A. /imu/data_raw static (10 s @ 50 Hz) ───────────────────')

    print('  Collecting 500 messages…', end='', flush=True)
    raw = wait_collect(node, 'raw', 500, timeout_s=15.0)
    print(f' got {len(raw)}')

    if len(raw) < 50:
        global _all_passed
        _all_passed = False
        print(f'  [{_R}FAIL{_RST}] too few messages — is bringup running?')
        return False

    ts  = [t for t, _ in raw]
    ax  = [m.linear_acceleration.x for _, m in raw]
    ay  = [m.linear_acceleration.y for _, m in raw]
    az  = [m.linear_acceleration.z for _, m in raw]
    gx  = [m.angular_velocity.x    for _, m in raw]
    gy  = [m.angular_velocity.y    for _, m in raw]
    gz  = [m.angular_velocity.z    for _, m in raw]
    amag = [math.sqrt(ax[i]**2 + ay[i]**2 + az[i]**2) for i in range(len(raw))]

    if len(ts) >= 2:
        rate = (len(ts) - 1) / (ts[-1] - ts[0])
        check('rate',              rate,      48.0,  52.0,  '.1f', ' Hz')

    # After axis remap: robot X=fwd, Y=left, Z=up — gravity in +Z.
    # accel.x/y are WARN here: physical mount tilt produces offsets in raw data;
    # imu_calib corrects them — check /imu/data_corrected with test 3D.
    check     ('accel.z mean  (robot Z=up, gravity)',  mean(az),  9.50, 10.10, unit=' m/s²')
    check_warn('accel.x mean  (robot X=fwd)',          mean(ax), -0.15,  0.15, unit=' m/s²',
               note='mount tilt in raw — run test 3D to verify imu_calib correction')
    check_warn('accel.y mean  (robot Y=left)',         mean(ay), -0.15,  0.15, unit=' m/s²',
               note='mount tilt in raw — run test 3D to verify imu_calib correction')
    check     ('accel magnitude',                      mean(amag), 9.51, 10.11, unit=' m/s²')

    check   ('gyro.x mean',  mean(gx), -0.020,  0.020, unit=' rad/s')
    check   ('gyro.y mean',  mean(gy), -0.020,  0.020, unit=' rad/s')
    check   ('gyro.z mean',  mean(gz), -0.020,  0.020, unit=' rad/s')
    check_lt('gyro.x std',   std(gx),   0.010,         unit=' rad/s')
    check_lt('gyro.y std',   std(gy),   0.010,         unit=' rad/s')
    check_lt('gyro.z std',   std(gz),   0.010,         unit=' rad/s')

    # orientation_covariance[0] == -1 means no orientation estimate (correct for data_raw)
    cov0 = raw[-1][1].orientation_covariance[0]
    _record(abs(cov0 - (-1.0)) < 1e-6, 'orientation_covariance[0] == -1 (no orientation)',
            f'{cov0:.1f}', '-1.0')

    # Magnetometer
    mag_msgs = node.snap_mag()
    if mag_msgs:
        mmag = [math.sqrt(m.magnetic_field.x**2 +
                          m.magnetic_field.y**2 +
                          m.magnetic_field.z**2) * 1e6   # T → µT
                for _, m in mag_msgs]
        check_warn('mag magnitude', mean(mmag), 20.0, 65.0, unit=' µT',
                   note='calibrate: hard-iron distortion — use ROS imu_tools magnetometer_calibration')
    else:
        warn('/imu/mag: no messages received')

    return True


def test_layer3_corrected(node: Collector) -> None:
    print('\n── Layer 3D. /imu/data_corrected static (imu_calib) ────────────────')

    print('  Collecting 500 messages…', end='', flush=True)
    corrected = wait_collect(node, 'corrected', 500, timeout_s=15.0)
    print(f' got {len(corrected)}')

    if len(corrected) < 50:
        global _all_passed
        _all_passed = False
        print(f'  [{_R}FAIL{_RST}] too few messages — is apply_calib_node running?')
        return

    ts  = [t for t, _ in corrected]
    ax  = [m.linear_acceleration.x for _, m in corrected]
    ay  = [m.linear_acceleration.y for _, m in corrected]
    az  = [m.linear_acceleration.z for _, m in corrected]
    gx  = [m.angular_velocity.x    for _, m in corrected]
    gy  = [m.angular_velocity.y    for _, m in corrected]
    gz  = [m.angular_velocity.z    for _, m in corrected]
    amag = [math.sqrt(ax[i]**2 + ay[i]**2 + az[i]**2) for i in range(len(corrected))]

    if len(ts) >= 2:
        rate = (len(ts) - 1) / (ts[-1] - ts[0])
        check('rate', rate, 48.0, 52.0, '.1f', ' Hz')

    # After imu_calib 3×3 matrix: tilt absorbed → accel.x/y near zero
    check('accel.z mean  (gravity, post-calib)', mean(az),  9.50, 10.10, unit=' m/s²')
    check('accel.x mean  (should be ≈0)',        mean(ax), -0.15,  0.15, unit=' m/s²')
    check('accel.y mean  (should be ≈0)',        mean(ay), -0.15,  0.15, unit=' m/s²')
    check('accel magnitude',                     mean(amag), 9.51, 10.11, unit=' m/s²')

    # Gyro bias removed by apply_calib_node calibrate_gyros=true
    check   ('gyro.x mean', mean(gx), -0.020,  0.020, unit=' rad/s')
    check   ('gyro.y mean', mean(gy), -0.020,  0.020, unit=' rad/s')
    check   ('gyro.z mean', mean(gz), -0.020,  0.020, unit=' rad/s')
    check_lt('gyro.x std',  std(gx),   0.010,         unit=' rad/s')
    check_lt('gyro.y std',  std(gy),   0.010,         unit=' rad/s')
    check_lt('gyro.z std',  std(gz),   0.010,         unit=' rad/s')


def test_layer3_gyro_sign(node: Collector) -> None:
    print('\n── Layer 3B. Gyro sign (motor-driven arc) ───────────────────────────')
    DUR = 2.0

    print(f'  CCW arc  linear={ARC_LINEAR} angular_z=+{ARC_ANGULAR} for {DUR} s…', end='', flush=True)
    raw_ccw = wait_collect(node, 'raw', 100, timeout_s=DUR + 2.0,
                           motor_cmd=(ARC_LINEAR, ARC_ANGULAR))
    node.stop()
    time.sleep(0.5)
    print(f' got {len(raw_ccw)} msgs')

    print(f'  CW  arc  linear={ARC_LINEAR} angular_z=-{ARC_ANGULAR} for {DUR} s…', end='', flush=True)
    raw_cw = wait_collect(node, 'raw', 100, timeout_s=DUR + 2.0,
                          motor_cmd=(ARC_LINEAR, -ARC_ANGULAR))
    node.stop()
    print(f' got {len(raw_cw)} msgs')

    if raw_ccw:
        gz_ccw = mean([m.angular_velocity.z for _, m in raw_ccw])
        _record(gz_ccw > 0.05, f'CCW arc (angular_z=+{ARC_ANGULAR}) → data_raw gyro.z positive',
                f'{gz_ccw:.4f} rad/s', '> +0.05 rad/s')

    if raw_cw:
        gz_cw = mean([m.angular_velocity.z for _, m in raw_cw])
        _record(gz_cw < -0.05, f'CW  arc (angular_z=-{ARC_ANGULAR}) → data_raw gyro.z negative',
                f'{gz_cw:.4f} rad/s', '< -0.05 rad/s')

    print('  Returning to origin…', end='', flush=True)
    node.drive_return(ARC_LINEAR, -ARC_ANGULAR, DUR)  # undo CW arc
    node.drive_return(ARC_LINEAR,  ARC_ANGULAR, DUR)  # undo CCW arc
    print(' done')


def test_layer3_mag_circle(node: Collector) -> None:
    print('\n── Layer 3C. Mag XY circle (motor-driven arc) ───────────────────────')
    print('  Driving slow circle for 6 s…', end='', flush=True)
    t_circle = time.monotonic()
    mag_msgs = wait_collect(node, 'mag', 120, timeout_s=8.0,
                            motor_cmd=(CIRCLE_LINEAR, CIRCLE_ANGULAR))
    circle_dur = time.monotonic() - t_circle
    node.stop()
    print(f' got {len(mag_msgs)} msgs')

    if len(mag_msgs) < 20:
        warn(f'too few mag messages ({len(mag_msgs)}) — skip')
        return

    mx = [m.magnetic_field.x * 1e6 for _, m in mag_msgs]
    my = [m.magnetic_field.y * 1e6 for _, m in mag_msgs]
    mz = [m.magnetic_field.z * 1e6 for _, m in mag_msgs]
    mmag = [math.sqrt(mx[i]**2 + my[i]**2 + mz[i]**2) for i in range(len(mag_msgs))]

    cx, cy = mean(mx), mean(my)
    radii  = [math.sqrt((mx[i] - cx)**2 + (my[i] - cy)**2) for i in range(len(mag_msgs))]

    check_warn('mag XY circle radius',               mean(radii), 5.0,  65.0, unit=' µT',
               note='calibrate: hard-iron — use ROS imu_tools magnetometer_calibration')
    check_warn('mag XY circle residual (std)',       std(radii),  0.0,  10.0, unit=' µT',
               note='calibrate: soft-iron — use ROS imu_tools magnetometer_calibration')
    check_warn('mag magnitude during rotation',      mean(mmag),  20.0, 65.0, unit=' µT',
               note='calibrate: hard-iron — use ROS imu_tools magnetometer_calibration')
    check_warn('mag.z std (constant during rotation)', std(mz),   0.0,   5.0, unit=' µT',
               note='calibrate: soft-iron — use ROS imu_tools magnetometer_calibration')

    print('  Returning to origin…', end='', flush=True)
    node.drive_return(CIRCLE_LINEAR, CIRCLE_ANGULAR, circle_dur)
    print(' done')


# ── Layer 4 — /imu/data (Madgwick) ───────────────────────────────────────────

def test_layer4_static(node: Collector) -> None:
    print('\n── Layer 4A. /imu/data static (Madgwick orientation) ───────────────')

    print('  Collecting 500 messages…', end='', flush=True)
    flt = wait_collect(node, 'flt', 500, timeout_s=15.0)
    print(f' got {len(flt)}')

    if len(flt) < 50:
        warn('too few /imu/data messages')
        return

    cov0 = flt[-1][1].orientation_covariance[0]
    _record(cov0 >= 0.0, 'orientation_covariance[0] >= 0 (valid orientation)',
            f'{cov0:.6f}', '>= 0')

    rpys = [quat_to_rpy(m.orientation) for _, m in flt]
    rolls  = [math.degrees(r[0]) for r in rpys]
    pitches= [math.degrees(r[1]) for r in rpys]
    check_lt('|roll|  when flat', abs(mean(rolls)),   2.0, '.2f', '°')
    check_lt('|pitch| when flat', abs(mean(pitches)), 2.0, '.2f', '°')

    # Gyro passthrough: data_raw and data angular_velocity must match
    raw = node.snap_raw()
    if raw and flt:
        gz_raw = mean([m.angular_velocity.z for _, m in raw[-50:]])
        gz_flt = mean([m.angular_velocity.z for _, m in flt[-50:]])
        diff = abs(gz_raw - gz_flt)
        check_lt('gyro.z passthrough raw↔flt delta', diff, 0.005, unit=' rad/s')


def test_layer4_yaw_drift(node: Collector) -> None:
    print('\n── Layer 4B. Yaw drift (60 s static) ───────────────────────────────')
    print('  Recording 60 s yaw…', end='', flush=True)

    node.clear()
    t0 = time.monotonic()
    while time.monotonic() - t0 < 62.0:
        rclpy.spin_once(node, timeout_sec=0.05)
    flt = node.snap_flt()
    print(f' got {len(flt)} msgs')

    if len(flt) < 100:
        warn('too few messages for drift test')
        return

    yaws = [math.degrees(quat_to_rpy(m.orientation)[2]) for _, m in flt]
    # unwrap to handle ±180° crossing
    yaw_unwrapped = [yaws[0]]
    for i in range(1, len(yaws)):
        d = angle_diff(math.radians(yaw_unwrapped[-1]),
                       math.radians(yaws[i]))
        yaw_unwrapped.append(yaw_unwrapped[-1] + math.degrees(d))

    drift = abs(yaw_unwrapped[-1] - yaw_unwrapped[0])
    check_lt('yaw drift over 60 s', drift, 5.0, '.2f', '°')


def test_layer4_gyro_sign(node: Collector) -> None:
    print('\n── Layer 4C. Gyro sign passthrough via /imu/data ───────────────────')
    print(f'  CCW arc  linear={ARC_LINEAR} angular_z=+{ARC_ANGULAR} for 2 s…', end='', flush=True)
    flt = wait_collect(node, 'flt', 100, timeout_s=4.0, motor_cmd=(ARC_LINEAR, ARC_ANGULAR))
    node.stop()
    print(f' got {len(flt)} msgs')

    if flt:
        gz = mean([m.angular_velocity.z for _, m in flt])
        _record(gz > 0.05, 'CCW → /imu/data angular_velocity.z positive',
                f'{gz:.4f} rad/s', '> +0.05 rad/s')

    print('  Returning to origin…', end='', flush=True)
    node.drive_return(ARC_LINEAR, ARC_ANGULAR, 2.0)
    print(' done')



# ── Layer 5 — EKF /odom ───────────────────────────────────────────────────────

def test_layer5_static_drift(node: Collector) -> None:
    print('\n── Layer 5A. /odom static drift (60 s) ─────────────────────────────')
    print('  Recording /odom for 60 s…', end='', flush=True)
    node.clear()
    t0 = time.monotonic()
    while time.monotonic() - t0 < 62.0:
        rclpy.spin_once(node, timeout_sec=0.05)
    odom = node.snap_odom()
    print(f' got {len(odom)} msgs')

    if len(odom) < 50:
        warn('too few /odom messages')
        return

    x0, y0 = odom[0][1].pose.pose.position.x, odom[0][1].pose.pose.position.y
    yaw0    = math.degrees(quat_to_rpy(odom[0][1].pose.pose.orientation)[2])
    xN, yN  = odom[-1][1].pose.pose.position.x, odom[-1][1].pose.pose.position.y
    yawN    = math.degrees(quat_to_rpy(odom[-1][1].pose.pose.orientation)[2])

    xy_drift  = math.sqrt((xN - x0)**2 + (yN - y0)**2)
    yaw_drift = abs(angle_diff(math.radians(yaw0), math.radians(yawN)))
    yaw_drift = math.degrees(yaw_drift)

    check_lt('XY drift  over 60 s', xy_drift,  0.02, '.4f', ' m')
    check_lt('yaw drift over 60 s', yaw_drift, 5.0,  '.2f', '°')


def test_layer5_gyro_sign(node: Collector) -> None:
    print('\n── Layer 5B. /odom gyro.z sign (CW spin) ───────────────────────────')
    print(f'  CW  arc  linear={ARC_LINEAR} angular_z=-{ARC_ANGULAR} for 2 s…', end='', flush=True)
    odom = wait_collect(node, 'odom', 100, timeout_s=4.0, motor_cmd=(ARC_LINEAR, -ARC_ANGULAR))
    node.stop()
    print(f' got {len(odom)} msgs')

    if odom:
        yaw0 = quat_to_rpy(odom[0][1].pose.pose.orientation)[2]
        yawN = quat_to_rpy(odom[-1][1].pose.pose.orientation)[2]
        delta = math.degrees(angle_diff(yaw0, yawN))
        _record(delta < -2.0, 'CW spin → /odom yaw decreasing',
                f'{delta:.2f}°', '< -2°')

    print('  Returning to origin…', end='', flush=True)
    node.drive_return(ARC_LINEAR, -ARC_ANGULAR, 2.0)
    print(' done')


def test_layer5_straight(node: Collector) -> None:
    print('\n── Layer 5C. Straight-line ~1 m ─────────────────────────────────────')
    print(f'  Driving forward 5 s at {DRIVE_LINEAR} m/s…', end='', flush=True)
    t_fwd = time.monotonic()
    odom = wait_collect(node, 'odom', 250, timeout_s=7.0, motor_cmd=(DRIVE_LINEAR, 0.0))
    fwd_dur = time.monotonic() - t_fwd
    node.stop()
    print(f' got {len(odom)} msgs')

    if odom:
        x0, y0 = odom[0][1].pose.pose.position.x, odom[0][1].pose.pose.position.y
        xN, yN = odom[-1][1].pose.pose.position.x, odom[-1][1].pose.pose.position.y
        dist   = math.sqrt((xN - x0)**2 + (yN - y0)**2)
        check('distance travelled', dist, 0.5, 1.5, '.3f', ' m')

    print('  Returning to origin…', end='', flush=True)
    node.drive_return(DRIVE_LINEAR, 0.0, fwd_dur)
    print(' done')


def test_layer5_turn_90(node: Collector) -> None:
    print('\n── Layer 5D. 90° right turn ─────────────────────────────────────────')
    print(f'  Turning CW (linear={ARC_LINEAR} angular_z=-{ARC_ANGULAR}) until ~-π/2 rad…', end='', flush=True)

    node.clear()
    t0 = time.monotonic()
    odom_start = None

    while time.monotonic() - t0 < 15.0:
        rclpy.spin_once(node, timeout_sec=0.02)
        snap = node.snap_odom()
        if not snap:
            node.drive(ARC_LINEAR, -ARC_ANGULAR)
            continue
        if odom_start is None:
            odom_start = snap[0]
        yaw0 = quat_to_rpy(odom_start[1].pose.pose.orientation)[2]
        yaw  = quat_to_rpy(snap[-1][1].pose.pose.orientation)[2]
        delta = angle_diff(yaw0, yaw)
        if delta < -math.pi / 2 - 0.05:
            break
        node.drive(ARC_LINEAR, -ARC_ANGULAR)

    turn_dur = time.monotonic() - t0
    node.stop()
    snap = node.snap_odom()
    print(f' got {len(snap)} msgs')

    if snap and odom_start:
        yaw0 = quat_to_rpy(odom_start[1].pose.pose.orientation)[2]
        yaw  = quat_to_rpy(snap[-1][1].pose.pose.orientation)[2]
        delta = math.degrees(angle_diff(yaw0, yaw))
        check('90° right turn yaw delta', delta, -100.0, -80.0, '.2f', '°')

    print('  Returning to origin…', end='', flush=True)
    node.drive_return(ARC_LINEAR, -ARC_ANGULAR, turn_dur)
    print(' done')


# ── Test registry ─────────────────────────────────────────────────────────────

TESTS = {
    '3A': '/imu/data_raw static — rate, accel, gyro, mag magnitude',
    '3B': '/imu/data_raw gyro sign — motor spin CW/CCW',
    '3C': '/imu/data_raw mag circle — motor arc',
    '3D': '/imu/data_corrected static — accel.x/y after imu_calib tilt+bias correction',
    '4A': '/imu/data static — orientation, roll/pitch, gyro passthrough',
    '4B': '/imu/data yaw drift — 60 s static',
    '4C': '/imu/data gyro sign — motor spin',
    '5A': '/odom static drift — 60 s',
    '5B': '/odom gyro.z sign — motor CW spin',
    '5C': '/odom straight line — ~1 m',
    '5D': '/odom 90° right turn accuracy',
}
MOTION_TESTS = {'3B', '3C', '4C', '5B', '5C', '5D'}


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(
        description='IMU verification — ROS2 layers 3–5',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='\n'.join(f'  {k}: {v}' for k, v in TESTS.items()))
    ap.add_argument('--tests', nargs='+', choices=sorted(TESTS), metavar='TEST',
                    default=['3A'],
                    help='tests to run (default: 3A); choices: ' + ' '.join(sorted(TESTS)))
    ap.add_argument('--imu-raw', default='/imu/data_raw', metavar='TOPIC',
                    help='unfiltered IMU topic (default: /imu/data_raw)')
    ap.add_argument('--imu-corrected', default='/imu/data_corrected', metavar='TOPIC',
                    help='imu_calib corrected topic (default: /imu/data_corrected)')
    ap.add_argument('--imu-flt', default='/imu/data', metavar='TOPIC',
                    help='filtered IMU topic (default: /imu/data)')
    args = ap.parse_args()

    tests         = set(args.tests)
    needs_motion  = bool(tests & MOTION_TESTS)

    rclpy.init()
    node = Collector(imu_raw=args.imu_raw, imu_flt=args.imu_flt,
                     imu_corrected=args.imu_corrected)

    print('═' * 62)
    print('  IMU Verification — ROS2 (Layers 3–5)')
    print(f'  Tests: {", ".join(sorted(tests))}')
    print(f'  raw:       {args.imu_raw}')
    print(f'  corrected: {args.imu_corrected}')
    print(f'  filtered:  {args.imu_flt}')
    print('═' * 62)
    print()
    print('  Assumes bringup is running on the Pi (make bringup).')

    input('\n[1] Place robot on a flat level surface, press Enter when ready: ')

    # Warm up subscriptions
    print('  Waiting for topics…', end='', flush=True)
    node.spin_for(2.0)
    print(' ready')

    do_motion = False
    if needs_motion:
        motion_list = ', '.join(
            f'{t} ({TESTS[t]})' for t in sorted(tests & MOTION_TESTS))
        ans = input(
            f'\n[2] Motion test(s): {motion_list}.\n'
            '    Robot will move — ensure 0.5 m clearance. Proceed? [y/N] ')
        do_motion = ans.strip().lower() == 'y'
        if not do_motion:
            print('  Motion tests skipped.')

    # ── Layer 3 ──
    if tests & {'3A', '3B', '3C', '3D'}:
        print('\n' + '─' * 62)
        print('  Layer 3 — /imu/data_raw + /imu/data_corrected')
        print('─' * 62)
        if '3A' in tests:
            test_layer3_static(node)
        if '3D' in tests:
            test_layer3_corrected(node)
        if do_motion:
            if '3B' in tests:
                test_layer3_gyro_sign(node)
            if '3C' in tests:
                test_layer3_mag_circle(node)

    # ── Layer 4 ──
    if tests & {'4A', '4B', '4C'}:
        print('\n' + '─' * 62)
        print('  Layer 4 — /imu/data (Madgwick)')
        r = subprocess.run(
            ['ros2', 'param', 'get', '/imu_filter_madgwick', 'use_mag'],
            capture_output=True, text=True)
        use_mag_str = r.stdout.strip() if r.returncode == 0 else 'unknown (node not running?)'
        print(f'  use_mag: {use_mag_str}')
        print('─' * 62)
        if '4A' in tests:
            test_layer4_static(node)
        if '4B' in tests:
            test_layer4_yaw_drift(node)
        if '4C' in tests and do_motion:
            test_layer4_gyro_sign(node)

    # ── Layer 5 ──
    if tests & {'5A', '5B', '5C', '5D'}:
        print('\n' + '─' * 62)
        print('  Layer 5 — EKF /odom')
        print('─' * 62)
        if '5A' in tests:
            test_layer5_static_drift(node)
        if do_motion:
            if '5B' in tests:
                test_layer5_gyro_sign(node)
            if '5C' in tests:
                test_layer5_straight(node)
            if '5D' in tests:
                test_layer5_turn_90(node)

    # ── Summary ──
    node.destroy_node()
    rclpy.shutdown()

    print('\n' + '═' * 62)
    if _all_passed:
        print(f'  Overall: {_G}PASS{_RST} — all checks passed')
    else:
        print(f'  Overall: {_R}FAIL{_RST} — one or more checks failed')
        print('  Tip: stop at first layer that fails — the bug is there or below.')
    print('═' * 62)
    sys.exit(0 if _all_passed else 1)


if __name__ == '__main__':
    main()
