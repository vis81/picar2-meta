#!/usr/bin/env bash
# Usage: debug.sh [FOCUS]
#   imu  — IMU, odometry, joint states        (default)
#   nav  — scan, map, costmaps, cmd_vel, odom
#   tf   — TF tree snapshot (PDF via view_frames)
#   all  — everything above combined
set -e
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash

FOCUS=${1:-imu}
STAMP=$(date +%Y%m%d_%H%M%S)
mkdir -p /ws/bags

case "$FOCUS" in
  imu)
    TOPICS="/imu/data_raw /imu/data /imu/mag \
            /odom /ackermann_steering_controller/odometry \
            /joint_states /tf /tf_static"
    ;;
  nav)
    TOPICS="/scan /map /cmd_vel /odom /imu/data \
            /local_costmap/costmap /global_costmap/costmap \
            /tf /tf_static"
    ;;
  tf)
    OUT=/ws/bags/tf_${STAMP}
    mkdir -p "$OUT"
    cd "$OUT"
    echo "Generating TF tree → $OUT/"
    ros2 run tf2_tools view_frames
    ls -lh "$OUT"
    exit 0
    ;;
  all)
    TOPICS="/imu/data_raw /imu/data /imu/mag \
            /odom /ackermann_steering_controller/odometry \
            /joint_states /scan /map /cmd_vel \
            /local_costmap/costmap /global_costmap/costmap \
            /tf /tf_static"
    ;;
  *)
    echo "Unknown FOCUS: $FOCUS"
    echo "Usage: make debug FOCUS=imu|nav|tf|all"
    exit 1
    ;;
esac

BAG=/ws/bags/${FOCUS}_${STAMP}
echo "FOCUS=$FOCUS  →  $BAG"
echo "Press Ctrl+C to stop recording."
echo ""
# shellcheck disable=SC2086
ros2 bag record -o "$BAG" $TOPICS
