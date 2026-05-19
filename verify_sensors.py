#!/usr/bin/env python3
"""
picar2 sensor verification.

Collects data from odom, IMU, and LiDAR topics for COLLECT_SECS seconds
then prints a PASS / WARN / FAIL report. Robot stack must be running.

Usage:
    source install/setup.bash
    python3 verify_sensors.py
"""

import math
import sys
import threading
import time

import rclpy
import rclpy.time
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float32
import tf2_ros

COLLECT_SECS = 5.0

# ── Terminal colours ──────────────────────────────────────────────────────────
G = '\033[92m'; R = '\033[91m'; Y = '\033[93m'; B = '\033[1m'; X = '\033[0m'

def _ok(s):   return f'  {G}PASS{X}  {s}'
def _fail(s): return f'  {R}FAIL{X}  {s}'
def _warn(s): return f'  {Y}WARN{X}  {s}'

# ── Verifier node ─────────────────────────────────────────────────────────────

class Verifier(Node):
    def __init__(self):
        super().__init__('picar2_verify')
        self.odom_msgs    = []
        self.imu_raw_msgs = []
        self.imu_msgs     = []
        self.scan_msgs    = []
        self.rpm_msgs     = []

        self.create_subscription(Odometry,  '/odom',         lambda m: self.odom_msgs.append(m),    10)
        self.create_subscription(Imu,       '/imu/data_raw', lambda m: self.imu_raw_msgs.append(m), 10)
        self.create_subscription(Imu,       '/imu/data',     lambda m: self.imu_msgs.append(m),     10)
        self.create_subscription(LaserScan, '/scan',         lambda m: self.scan_msgs.append(m),    10)
        self.create_subscription(Float32,   '/scan/rpm',     lambda m: self.rpm_msgs.append(m),     10)

        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _rate(self, msgs):
        """Compute Hz from a list of stamped messages."""
        if len(msgs) < 2:
            return 0.0
        t0 = msgs[0].header.stamp
        t1 = msgs[-1].header.stamp
        dt = (t1.sec - t0.sec) + (t1.nanosec - t0.nanosec) * 1e-9
        return (len(msgs) - 1) / dt if dt > 0 else 0.0

    def _check_rate(self, msgs, label, expected_hz, tol=0.25):
        if not msgs:
            return _fail(f'{label}: no messages received'), False
        hz = self._rate(msgs)
        lo = expected_hz * (1 - tol)
        if hz < lo:
            return _fail(f'{label}: {hz:.1f} Hz  (expected ≥ {lo:.0f} Hz)'), False
        return _ok(f'{label}: {hz:.1f} Hz'), True

    def _check_tf(self, parent, child):
        try:
            self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
            return _ok(f'TF  {parent} → {child}'), True
        except Exception as e:
            return _fail(f'TF  {parent} → {child}  ({e})'), False

    # ── Checks ────────────────────────────────────────────────────────────────

    def run_checks(self):
        results = []

        def record(line, passed):
            print(line)
            results.append(passed)

        # ── TF chain ──────────────────────────────────────────────────────────
        print(f'\n{B}TF chain{X}')
        for parent, child in [
            ('odom',          'base_footprint'),
            ('base_footprint','base_link'),
            ('base_link',     'laser_link'),
            ('base_link',     'imu_link'),
        ]:
            record(*self._check_tf(parent, child))

        # ── Odometry ──────────────────────────────────────────────────────────
        print(f'\n{B}Odometry{X}')
        record(*self._check_rate(self.odom_msgs, '/odom', expected_hz=50))

        if self.odom_msgs:
            p = self.odom_msgs[-1].pose.pose.position
            if all(math.isfinite(v) for v in (p.x, p.y, p.z)):
                record(_ok(f'position finite  ({p.x:.3f}, {p.y:.3f}, {p.z:.3f} m)'), True)
            else:
                record(_fail('position contains NaN/inf'), False)

            cov = self.odom_msgs[-1].pose.covariance
            if any(cov[i * 7] > 0 for i in range(6)):
                record(_ok('pose covariance non-zero'), True)
            else:
                record(_warn('pose covariance all-zero — EKF may not be running'), False)

        # ── IMU raw ───────────────────────────────────────────────────────────
        print(f'\n{B}IMU raw  (/imu/data_raw){X}')
        record(*self._check_rate(self.imu_raw_msgs, '/imu/data_raw', expected_hz=50))

        if self.imu_raw_msgs:
            tail = self.imu_raw_msgs[-20:]
            ax = sum(m.linear_acceleration.x for m in tail) / len(tail)
            ay = sum(m.linear_acceleration.y for m in tail) / len(tail)
            az = sum(m.linear_acceleration.z for m in tail) / len(tail)
            # imu_link rpy="0 π π/2" maps base_link -Z onto imu +Z
            # so gravity ≈ +9.8 on imu Z when robot is flat
            mag = math.sqrt(ax**2 + ay**2 + az**2)
            if 8.0 < mag < 11.0:
                record(_ok(f'accel magnitude: {mag:.2f} m/s²  (ax={ax:.2f} ay={ay:.2f} az={az:.2f})'), True)
            else:
                record(_fail(f'accel magnitude: {mag:.2f} m/s²  — expected 8–11  (ax={ax:.2f} ay={ay:.2f} az={az:.2f})'), False)

            if 7.0 < az < 11.0:
                record(_ok(f'gravity on IMU Z: {az:.2f} m/s²  (correct axis for this mounting)'), True)
            else:
                record(_warn(f'gravity not on IMU Z: {az:.2f} m/s²  — check mounting / axis convention'), False)

            gx = sum(m.angular_velocity.x for m in tail) / len(tail)
            gy = sum(m.angular_velocity.y for m in tail) / len(tail)
            gz = sum(m.angular_velocity.z for m in tail) / len(tail)
            gyro_mag = math.sqrt(gx**2 + gy**2 + gz**2)
            if gyro_mag < 0.1:
                record(_ok(f'gyro near zero when static: {gyro_mag:.4f} rad/s'), True)
            else:
                record(_warn(f'gyro not near zero: {gyro_mag:.4f} rad/s  — move robot or check bias'), False)

        # ── IMU filtered ──────────────────────────────────────────────────────
        print(f'\n{B}IMU filtered  (/imu/data){X}')
        record(*self._check_rate(self.imu_msgs, '/imu/data', expected_hz=50))

        if self.imu_msgs:
            q = self.imu_msgs[-1].orientation
            qmag = math.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
            if abs(qmag - 1.0) < 0.05:
                record(_ok(f'orientation quaternion unit  (|q|={qmag:.4f})'), True)
            else:
                record(_fail(f'orientation quaternion not unit  (|q|={qmag:.4f})'), False)

            cov = self.imu_msgs[-1].orientation_covariance
            if cov[0] < 0:
                record(_warn('orientation_covariance[0] = -1 — Madgwick not publishing orientation'), False)
            else:
                record(_ok('orientation covariance valid'), True)

        # ── LiDAR ─────────────────────────────────────────────────────────────
        print(f'\n{B}LiDAR{X}')
        record(*self._check_rate(self.scan_msgs, '/scan', expected_hz=5, tol=0.3))

        if self.rpm_msgs:
            rpm = self.rpm_msgs[-1].data
            if 250 < rpm < 380:
                record(_ok(f'motor RPM: {rpm:.0f}'), True)
            else:
                record(_fail(f'motor RPM: {rpm:.0f}  (expected 250–380)'), False)
        else:
            record(_fail('/scan/rpm: no messages'), False)

        if self.scan_msgs:
            scan = self.scan_msgs[-1]
            n = len(scan.ranges)
            if n == 360:
                record(_ok(f'scan size: {n} rays'), True)
            else:
                record(_fail(f'scan size: {n}  (expected 360)'), False)

            valid = sum(1 for r in scan.ranges
                        if math.isfinite(r) and scan.range_min <= r <= scan.range_max)
            pct = valid / max(n, 1) * 100
            if pct > 10:
                record(_ok(f'valid returns: {valid}/{n}  ({pct:.0f}%)'), True)
            elif pct > 2:
                record(_warn(f'few valid returns: {valid}/{n}  ({pct:.0f}%)  — open area or sensor issue?'), False)
            else:
                record(_fail(f'almost no valid returns: {valid}/{n}  ({pct:.0f}%)'), False)

            fid = scan.header.frame_id
            if fid == 'laser_link':
                record(_ok(f'frame_id: {fid}'), True)
            else:
                record(_fail(f'frame_id: {fid!r}  (expected "laser_link")'), False)

        # ── Summary ───────────────────────────────────────────────────────────
        passed = sum(results)
        total  = len(results)
        print(f'\n{B}{"─" * 44}{X}')
        if passed == total:
            print(f'{G}{B}All {total} checks passed{X}\n')
        else:
            print(f'{R}{B}{passed}/{total} checks passed{X}\n')
        return passed == total


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    node = Verifier()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print(f'Collecting for {COLLECT_SECS:.0f} s ...', flush=True)
    for i in range(int(COLLECT_SECS)):
        time.sleep(1.0)
        counts = (
            f'odom={len(node.odom_msgs)}'
            f'  imu_raw={len(node.imu_raw_msgs)}'
            f'  imu={len(node.imu_msgs)}'
            f'  scan={len(node.scan_msgs)}'
            f'  rpm={len(node.rpm_msgs)}'
        )
        print(f'  {i+1}/{int(COLLECT_SECS)}s  {counts}', end='\r', flush=True)
    print()

    all_ok = node.run_checks()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if all_ok else 1)


if __name__ == '__main__':
    main()
