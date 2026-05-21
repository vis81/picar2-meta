#!/usr/bin/env bash
# Quick diagnostic snapshot — node list, topic rates, key message samples.
# Output is printed to stdout and saved to bags/diag_<timestamp>.txt
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash

STAMP=$(date +%Y%m%d_%H%M%S)
mkdir -p /ws/bags
OUT=/ws/bags/diag_${STAMP}.txt

run() {
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "  $*"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
}

{
echo "══════════════════════════════════════════════════════"
echo "  PICAR-2 diagnostic snapshot — $(date)"
echo "══════════════════════════════════════════════════════"
echo ""

run "ros2 doctor"
ros2 doctor 2>&1 || true
echo ""

run "Active nodes"
ros2 node list 2>&1
echo ""

run "Active topics"
ros2 topic list 2>&1
echo ""

# ── Topic rates (parallel, 3 s window) ────────────────────────────────────
run "Topic rates (3 s window)"
KEY_TOPICS=(
    /imu/data_raw
    /imu/data
    /odom
    /ackermann_steering_controller/odometry
    /joint_states
    /scan
    /cmd_vel
    /tf
)
declare -A PIDS TMPS
for t in "${KEY_TOPICS[@]}"; do
    tmp=$(mktemp)
    TMPS[$t]=$tmp
    timeout 3 ros2 topic hz "$t" >"$tmp" 2>&1 &
    PIDS[$t]=$!
done
for t in "${KEY_TOPICS[@]}"; do
    wait "${PIDS[$t]}" 2>/dev/null || true
    rate=$(grep "average rate" "${TMPS[$t]}" | tail -1 | sed 's/.*average rate: //')
    printf "  %-55s %s\n" "$t" "${rate:-no data}"
    rm -f "${TMPS[$t]}"
done
echo ""

# ── Single-message samples ─────────────────────────────────────────────────
run "/imu/data_raw  (1 sample — check linear_acceleration and angular_velocity when stopped)"
timeout 3 ros2 topic echo --once /imu/data_raw 2>&1 || echo "  no data"
echo ""

run "/odom  (1 sample — check twist.twist.linear when stopped)"
timeout 3 ros2 topic echo --once /odom 2>&1 || echo "  no data"
echo ""

run "/joint_states  (1 sample — velocity should be ~0 when stopped)"
timeout 3 ros2 topic echo --once /joint_states 2>&1 || echo "  no data"
echo ""

run "/ackermann_steering_controller/odometry  (1 sample — wheel-only odometry)"
timeout 3 ros2 topic echo --once /ackermann_steering_controller/odometry 2>&1 || echo "  no data"
echo ""

} 2>&1 | tee "$OUT"

echo ""
echo "Saved → $OUT"
