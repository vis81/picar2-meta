# PICAR-2 ROS2 Workspace

## Overview

ROS2 Jazzy colcon workspace. Four packages in `src/`. Build runs inside Docker
(`picar2-ros2:jazzy`). The robot runs bringup on a Raspberry Pi 4; RViz / odom-cal
run on a PC — both in Docker containers on the same LAN via Cyclone DDS.

## Build & Run

```bash
# From ros2/
make build          # colcon build --symlink-install inside Docker
make bringup        # Pi: launch full robot stack (privileged, /dev access)
make rviz PI_IP=... PC_IFACE=...   # PC: RViz2
make odom-cal PI_IP=... PC_IFACE=... # PC: odometry calibration GUI
make slam           # attach to running picar2 container
make teleop         # keyboard teleop in running picar2 container
make shell          # interactive bash in picar2 container
make sync2pi        # rsync to pi@rpi4.local:~/picar_ws/ros2/
make clean          # rm build/ install/ log/
make image          # rebuild Docker image
```

Docker image: `picar2-ros2:jazzy`
- aarch64 (Pi): base `ros:jazzy-ros-base`
- amd64 (PC):  base `osrf/ros:jazzy-desktop`

DDS: Cyclone DDS (`rmw_cyclonedds_cpp`).
- Pi bringup uses `cyclonedds.xml`
- PC tools use `cyclonedds-pc.xml` and need `PI_IP=<pi-ip> PC_IFACE=<iface>` env vars

## Packages

### `picar2_bringup` (ament_cmake)
Launch files, controller config, calibration tools.

Key files:
- `launch/picar2.launch.py` — main bringup (see Nodes section)
- `launch/slam.launch.py` — slam_toolbox + nav2_lifecycle_manager
- `config/controllers.yaml` — AckermannSteeringController params
- `config/ekf.yaml` — robot_localization EKF config
- `config/rviz.rviz` — saved RViz layout (**never hand-edit**: use Save in RViz)
- `picar2_bringup/cmd_vel_relay.py` — Twist → TwistStamped relay
- `picar2_bringup/odom_cal.py` — tkinter GUI for straight-line + circle calibration

### `picar2_control` (ament_cmake)
`ros2_control` SystemInterface plugin for the STM32F103 board.

Key files:
- `src/picar2_hardware.cpp` — all logic: serial framing, joint I/O, IMU publish, timesync
- `include/picar2_control/picar2_hardware.hpp`
- `picar2_control.xml` — pluginlib descriptor

### `picar2_description` (ament_cmake)
URDF + meshes.

Key files:
- `urdf/picar2.urdf.xacro` — robot description + `<ros2_control>` block with hardware params

### `picar2_lidar` (ament_python)
Driver for LDS02RR (Neato XV-11) LiDAR.
- Entry point: `picar2_lidar.lidar_node:main`
- `launch/lidar.launch.py`

## Nodes Launched (picar2.launch.py)

| Node | Package | Role |
|------|---------|------|
| controller_manager | ros2_control | loads Picar2Hardware, 50 Hz loop |
| robot_state_publisher | — | TF from URDF + /joint_states |
| joint_state_broadcaster | — | /joint_states from hw interfaces |
| ackermann_steering_controller | — | Ackermann kinematics → steer + wheel cmds |
| cmd_vel_relay | picar2_bringup | /cmd_vel (Twist) → controller reference (TwistStamped) |
| imu_filter_madgwick | imu_filter_madgwick | /imu/data_raw → /imu/data (no mag, ENU) |
| ekf_node | robot_localization | odom + IMU → /odom + TF odom→base_footprint |
| lidar_node | picar2_lidar | LiDAR driver (target_rpm=300, angle_offset=-2.8 rad) |

Launch args: `port` (default `/dev/ttyYahboom0`), `baud` (460800), `lidar` (true).

## Hardware Interface (Picar2Hardware)

### URDF Hardware Params (`picar2.urdf.xacro` → `<hardware>` block)

| Param | Default | Meaning |
|-------|---------|---------|
| `port` | `/dev/ttyYahboom0` | Serial device |
| `baud` | `460800` | UART baud rate |
| `imu_rate_hz` | `50` | IMU stream rate (Hz) |
| `steer_us_per_rad` | `950.0` | Servo scale µs/rad — **calibration knob** |
| `imu_mount_roll` | `-0.1648` | IMU tilt correction (rad) — update after remount |
| `imu_mount_pitch` | `-0.0657` | IMU tilt correction (rad) — update after remount |

### Joint Interfaces

State: `back_{left,right}_joint` position+velocity; `front_{left,right}_steer_joint` position;
`front_{left,right}_wheel_joint` position (passive, held 0).

Command: `back_{left,right}_joint` velocity (rad/s); `front_{left,right}_steer_joint`
position (rad, ±0.6 rad — URDF limit).

### Serial Protocol (binary, 0xAA-framed, CRC-8 poly 0x31)

| Msg | ID | Direction | Content |
|-----|----|-----------|---------|
| MSG_CMD_VEL | 0x80 | Pi→STM32 | int16 LE: left_dps, right_dps, steer_delta_us |
| MSG_REQ | 0x81 | Pi→STM32 | stream id to request once |
| MSG_SET_RATE | 0x82 | Pi→STM32 | stream id + Hz (0=stop) |
| MSG_TIMESYNC | 0x84 | Pi→STM32 | T1 timestamp (µs) |
| MSG_SERVO_CENTER | 0x86 | Pi→STM32 | servo_id + center_us LE |
| STREAM_JOINT | 0x01 | STM32→Pi | enc_l, enc_r (int32), steer_delta_us (int16), seq, vel_l, vel_r, pi_time |
| STREAM_IMU | 0x02 | STM32→Pi | accel xyz + gyro xyz + mag xyz (int16, ×0.001) |

**Steer encoding**: `delta_us = round(-steer_rad × steer_us_per_rad)` — sent as int16 LE.
Firmware does `pwm = center_us + delta_us` (clamps to [min_pulse_us, max_pulse_us]).
The servo center is runtime-adjustable via `MSG_SERVO_CENTER` and persisted in STM32 settings.

### Key Implementation Details

- `read()` sends MSG_REQ(JOINT) and waits ≤5 ms for the frame via a condition_variable.
  If no frame arrives, stale data is used and a throttled WARN is logged.
- `write()` sends MSG_CMD_VEL every control cycle (50 Hz). Averages left/right steer angles.
- `reader_loop()` (background thread) decodes frames and publishes `/imu/data_raw` + `/imu/mag`.
- `timesync_loop()` (background thread) sends MSG_TIMESYNC at 1 Hz for latency measurement.
- IMU axes are remapped and tilt-corrected via precomputed rotation matrix `imu_R_[3][3]`
  (R = Ry(pitch) × Rx(roll)) in `dispatch_imu_frame()`.

## Robot Geometry (controllers.yaml)

| Param | Value |
|-------|-------|
| wheelbase | 0.235 m |
| traction_track_width | 0.1685 m |
| steering_track_width | 0.173 m |
| traction_wheels_radius | 0.033 m (**needs calibration**) |
| reference_timeout | 0.5 s (matches STM32 watchdog) |
| enable_odom_tf | false (EKF owns odom→base_footprint) |

## Calibration Parameters

| What | Where | How |
|------|-------|-----|
| Wheel radius | `controllers.yaml` `traction_wheels_radius` | straight-line test with odom_cal.py |
| Steer scale | `picar2.urdf.xacro` `steer_us_per_rad` | circle test with odom_cal.py |
| Servo center | STM32 settings (persistent) | `tools/servo_center.py` or shell `servo center 0 <us>` |
| IMU tilt | `picar2.urdf.xacro` `imu_mount_roll/pitch` | `imu cal accel` on STM32 shell, then update URDF |
| IMU gyro bias | STM32 settings | `imu cal gyro` on STM32 shell |

## Topics

| Topic | Type | Publisher | Notes |
|-------|------|-----------|-------|
| `/cmd_vel` | Twist | teleop/nav | relayed to controller |
| `/ackermann_steering_controller/odometry` | Odometry | controller | wheel odometry |
| `/imu/data_raw` | Imu | picar2_hardware | no orientation |
| `/imu/data` | Imu | imu_filter_madgwick | with orientation (no mag) |
| `/imu/mag` | MagneticField | picar2_hardware | soft-iron distortion present — mag fusion disabled |
| `/odom` | Odometry | robot_localization EKF | fused odom + IMU |
| `/scan` | LaserScan | lidar_node | LiDAR scan |
| `/joint_states` | JointState | joint_state_broadcaster | — |

## Pitfalls

- **RViz config**: never hand-edit `config/rviz.rviz` — missing `Views` fields break mouse input.
  Always save from within RViz.
- **Magnetometer**: `use_mag: False` in imu_filter config — motor soft-iron causes yaw drift
  when mag is enabled. Do not re-enable.
- **odom-cal on PC**: requires `PI_IP` and `PC_IFACE` — Cyclone DDS fails silently if empty.
- **Steer unit**: steer field in MSG_CMD_VEL is µs delta from servo center (not degrees).
  `steer_us_per_rad` is the sole conversion factor between rad and µs.
- **Symlink install**: `colcon build --symlink-install` is used — Python scripts and config
  files are symlinked, so edits in `src/` take effect without rebuild for those files.
  C++ changes still require a rebuild.
