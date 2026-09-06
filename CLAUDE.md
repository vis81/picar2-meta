# PICAR-2 Workspace

Meta-repo for the PICAR-2 robot: an Ackermann-steered car with a Raspberry Pi 4
(ROS 2 Jazzy) on top and a Yahboom STM32F103 board (Zephyr firmware) driving the
motors, servos and IMU. Bringup runs on the Pi; RViz / calibration / Gazebo run
on a PC. Both sides usually run in Docker on the same LAN over Cyclone DDS.

`README.md` covers first-time setup (vcs import, chrony time sync, PS4 pairing,
Bluetooth troubleshooting). This file is the map of the code.

## Layout

The repo root holds only build infrastructure. All source lives in `src/`, which
is **not committed** — it is populated by `make deps` (`vcs import < .repos`).

```
picar2_ws/                    ← this repo (Makefile, Dockerfile, DDS configs, scripts, etc/)
  src/
    picar2-ros2/              ← first-party ROS packages (own git repo)
      picar2_bringup/         ← launch files, configs, calibration tools
      picar2_control/         ← ros2_control SystemInterface for the STM32
      picar2_description/     ← URDF + meshes
    yahboom/                  ← Zephyr firmware (own git; COLCON_IGNORE'd by make deps)
    lds02rr_lidar/            ← LDS02RR (Neato XV-11) driver
    ldrobot_ld07/             ← LD07 structured-light depth sensor driver
    ldrobot-lidar-ros2/       ← LD19 lidar (external)
    tof_imager_ros/           ← SEN0628 matrix ToF driver
    vizanti/                  ← web mission planner (external, forked)
    explore_lite/             ← frontier exploration (external)
    imu_calib/                ← accel/gyro calibration (external)
    robotnik_gazebo_worlds/   ← sim worlds (external)
  maps/                       ← saved maps (pbstream / posegraph / pgm+yaml)
```

Each `src/*` entry is an independent git repo. `make status` / `make pull` /
`make push` operate on the meta-repo **and** all sub-repos (push only touches
`vis81`-owned remotes). Firmware work is documented separately in
`src/yahboom/CLAUDE.md` — read that before touching Zephyr code.

## Build & Run

Every target runs either directly on the host or inside Docker, selected by
`EXEC_ENV`:

```bash
make deps                     # vcs import into src/
make build                    # colcon build --symlink-install
make image                    # build picar2-ros2:jazzy locally
make docker-start             # persistent 'picar2' container (privileged, host net/ipc, /dev)
EXEC_ENV=docker make bringup  # any target, but inside that container
make docker-stop
```

- `EXEC_ENV=host` (default) → `bash -c`, build dirs `build/` + `install/`
- `EXEC_ENV=docker` → `docker exec -it picar2 …`, build dirs `build-docker/` + `install-docker/`

Both trees coexist; never mix them (a host-built `install/` won't run in the
container and vice versa).

**The Pi runs the stack in Docker**, despite `EXEC_ENV` defaulting to `host` —
so on the robot the workspace is `/ws`, the install tree is `install-docker/`,
and anything launched from systemd has to go through `docker run`, not a
sourced host ROS.

Image base is arch-dependent: `ros:jazzy-ros-base` on aarch64 (Pi),
`osrf/ros:jazzy-desktop` on amd64 (PC). Gazebo and GUI extras are installed on
amd64 only. `make image-pi` cross-builds the arm64 image via QEMU/buildx,
`make image-push` ships it to the Pi over ssh.

### Robot targets (Pi)

| Target | What it does |
|---|---|
| `make bringup` | Full stack — see Nodes below. Knobs: `LIDAR` (`ld19`\|`lds02rr`\|`none`), `USE_MAG`, `USE_SEN0628`, `USE_FOXGLOVE`, `USE_VIZANTI`, `USE_JOY`, `JOY_CONFIG` |
| `make cartographer` | Cartographer SLAM (**preferred**) |
| `make save-cartographer-map MAP=home` | `finish_trajectory` + `write_state` → `maps/home.pbstream`, plus a PGM/YAML via `map_saver_cli` |
| `make cartographer-resume MAP=home` | Resume mapping on a saved pbstream |
| `make cartographer-localize MAP=home` | Pure localization on a frozen pbstream |
| `make slam` / `slam-resume` / `slam-localize` / `save-map` | slam_toolbox equivalents |
| `make amcl MAP=home` | map_server + AMCL on a PGM/YAML — backend-agnostic |
| `make nav` | Nav2 (assumes bringup + a map source are already up) |
| `make explore` | explore_lite frontier exploration |
| `make teleop` / `make joystick` | keyboard / gamepad teleop |
| `make diag`, `make debug FOCUS=imu\|nav\|tf\|all` | snapshot + rosbag capture into `bags/` |
| `make firmware` / `make flash` | Zephyr build/flash via `src/yahboom/activate.sh` |
| `make install-uarts` | udev symlinks + socat TCP bridge to the Zephyr shell (port 4444) |
| `make fpv-setup` / `fpv` / `fpv-stop` | MediaMTX + WebXR FPV for Meta Quest 3 (host systemd, not Docker) |
| `make softap SSID=… PASS=…` | concurrent AP+STA on the Pi's wlan0 |

### PC targets

`make rviz`, `make rqt`, `make odom-cal`, `make mag-calib`, `make lidar-ld07-view`,
`make sen0628-view` — all use `cyclonedds-pc.xml` and **require `PI_IP` and
`PC_IFACE`**; Cyclone DDS fails silently when they are empty:

```bash
make rviz PI_IP=192.168.0.42 PC_IFACE=enp3s0
```

### Simulation (PC)

`make sim WORLD=room` launches Gazebo with `gz_ros2_control` instead of the real
hardware plugin (`use_sim:=true` in the xacro). `scripts/patch_gz_world.py`
injects missing system plugins into third-party world files and prints a patched
copy under `/tmp`. Companion targets: `slam-sim`, `nav-sim`, `explore-sim`.

## Nodes launched by `picar2.launch.py`

| Node | Package | Role |
|---|---|---|
| `ros2_control_node` | controller_manager | loads `Picar2Hardware`, 50 Hz loop |
| `robot_state_publisher` | — | TF from URDF; `publish_frequency` capped at 20 Hz |
| `joint_state_broadcaster` | — | `/joint_states` |
| `ackermann_steering_controller` | — | Ackermann kinematics + wheel odometry |
| `pan_tilt_controller` | forward_command_controller | camera pan/tilt servos |
| `cmd_vel_relay` | picar2_bringup | `/cmd_vel` (Twist) → controller reference (TwistStamped), clamps angular + reverse |
| `apply_calib_node` | imu_calib | `/imu/data_raw` → `/imu/data_corrected` (bias/scale, gyro auto-calib on start) |
| `mag_bias_observer` + `magnetometer_bias_remover` | magnetometer_pipeline | `/imu/mag` → `/imu/mag_unbiased` (only when `use_mag:=true`) |
| `imu_filter_madgwick` | imu_filter_madgwick | → `/imu/data`, ENU, `publish_tf:=false` |
| `ekf_node` | robot_localization | wheel odom + IMU yaw rate → `/odom` + `odom→base_footprint` TF |
| lidar | lds02rr_lidar **or** ldlidar_component (LD19, lifecycle + composable container + lifecycle_manager) | `/lidar_node/scan` |
| `tof_imager` | tof_imager_ros | SEN0628 front ToF → `/sen0628/pointcloud` (lifecycle, auto-configured/activated in the launch file) |
| `ldrobot_ld07_node` | ldrobot_ld07 | legacy front depth sensor, superseded by SEN0628 |
| `foxglove_bridge` | foxglove_bridge | `ws://<pi>:8765` |
| Vizanti stack + `waypoints_to_simple_goals` | vizanti_* | web UI on `:5000`; the bridge turns Vizanti's `/waypoints` PoseArray into one `/goal_pose` at a time |
| `joy_node` + `teleop_twist_joy` + `joy_feedback` | — | gamepad; `joy_feedback.py` drives DS4 rumble/lightbar over evdev |

Launch args: `port` (`/dev/ttyYahboom0`), `baud` (460800), `lidar` (`lds02rr`),
`use_mag` (false), `use_sen0628` (true), `sen0628_port`, `use_foxglove`,
`use_vizanti`, `use_joy`, `joy_dev`, `joy_config`, `calib_file`.

Note the Makefile's defaults differ from the launch file's in places — e.g.
`LIDAR ?= ld19` and `USE_MAG ?= true`. The Makefile wins for `make bringup`.

## Hardware interface (`picar2_control`)

Single file: `src/picar2-ros2/picar2_control/src/picar2_hardware.cpp`.

### Params (URDF `<hardware>` block)

| Param | Default | Meaning |
|---|---|---|
| `port` / `baud` | `/dev/ttyYahboom0` / 460800 | serial device |
| `imu_rate_hz` | 50 | IMU stream rate |
| `steer_lut_us` / `steer_lut_rad` | measured table | **steering calibration** — piecewise-linear µs↔rad LUT |
| `steer_us_per_rad` | — | legacy single-scale fallback, used only if no LUT is given |
| `steer_max_rate_rad_s` | 4.0 | slew limit applied to reported steer position |
| `pan_us_per_rad` / `tilt_us_per_rad` | 500.0 | camera servo scale |

The steering LUT is sorted ascending by µs and **descending** by rad (positive µs
= right turn = negative rad). `lut_rad_to_us()` / `lut_us_to_rad()` interpolate
between entries and clamp at the ends. The table in the xacro was measured from a
wheel-angle photo (2026-05-22).

### Joints

- State: `back_{left,right}_joint` position+velocity; `front_{left,right}_steer_joint`
  position; `front_{left,right}_wheel_joint` position (passive, estimated from the
  rear-wheel average); `pan_joint` / `tilt_joint` (open-loop, state = last command).
- Command: rear wheels velocity (rad/s); steer joints position (±0.6 rad);
  pan/tilt position (±1.57 rad).

### Serial protocol (`[0xAA][TYPE][LEN][PAYLOAD][CRC8]`, CRC-8 poly 0x31)

| Msg | ID | Direction | Payload |
|---|---|---|---|
| `MSG_CMD_VEL` | 0x80 | Pi→STM32 | int16 LE ×3: left_dps, right_dps, steer_delta_us |
| `MSG_SET_RATE` | 0x82 | Pi→STM32 | stream id + Hz (0 = stop) |
| `MSG_TIMESYNC` | 0x84 | Pi→STM32 | T1 timestamp |
| `MSG_SERVO_WRITE` | 0x87 | Pi→STM32 | servo id (1=pan, 2=tilt) + int16 LE delta µs |
| `STREAM_JOINT` | 0x01 | STM32→Pi | encoders, steer µs, seq, velocities, echoed Pi time |
| `STREAM_IMU` | 0x02 | STM32→Pi | accel + gyro (×0.001) + mag (×0.1 µT) as int16 |
| `MSG_TIMESYNC_RESP` | 0x05 | STM32→Pi | timesync reply |

Key behaviours:

- `on_activate()` sets JOINT to **100 Hz** and IMU to `imu_rate_hz`. Streams are
  pushed, not polled — `read()` just consumes the newest staged frame and warns
  (throttled) when none arrived. `on_deactivate()` sets both rates to 0.
- `write()` runs every control cycle (50 Hz): one `MSG_CMD_VEL` (steer = average
  of the two commanded steer angles, through the LUT) plus a `MSG_SERVO_WRITE`
  each for pan and tilt.
- `reader_loop()` (thread) decodes frames and publishes `/imu/data_raw` and
  `/imu/mag` from its own `picar2_imu` node. IMU axes are remapped chip→robot in
  `dispatch_imu_frame()`; there is no tilt-correction matrix any more — mounting
  error is handled by `imu_calib`.
- `timesync_loop()` (thread) sends `MSG_TIMESYNC` at 1 Hz; sensor lag is logged
  every 5 s.
- Firmware watchdog: 500 ms without `MSG_CMD_VEL` → motors stop and the board
  falls back to RC input.

## Geometry & tuning (`config/controllers.yaml`)

| Param | Value |
|---|---|
| wheelbase | 0.235 m |
| traction_track_width | 0.1685 m |
| steering_track_width | 0.173 m |
| traction_wheels_radius | 0.033 m (**still marked TODO: calibrate**) |
| update_rate | 50 Hz |
| reference_timeout | 0.5 s (matches firmware watchdog) |
| enable_odom_tf | false — EKF owns `odom→base_footprint` |
| cmd_vel_relay max_angular_vel | 1.2 rad/s |
| cmd_vel_relay max_reverse_vel | 0.25 m/s |

EKF (`config/ekf.yaml`): `two_d_mode`, 50 Hz. Takes x/y/yaw/vx from
`/ackermann_steering_controller/odometry` and **yaw rate only** from `/imu/data`.

## Navigation

Nav2 is configured for a car-like robot that **cannot spin in place**:

- Planner: `SmacPlannerHybrid`, `REEDS_SHEPP`, `minimum_turning_radius: 0.5`
  (measured; theoretical ≈0.34), `angle_quantization_bins: 36` — 72 caused ~42 s
  startup on the Pi 4.
- Controller: `RegulatedPurePursuitController`, `use_rotate_to_heading: false`,
  `allow_reversing: true`, `desired_linear_vel: 0.40`.
- Behaviors: backup / drive_on_heading / wait — **no spin**.
- BT: `navigate_through_poses` uses `config/nav_through_poses_ackermann.xml`
  (upstream tree with `<Spin>` stripped), wired in from `nav2.launch.py`.
- Costmaps: explicit rectangular `footprint` (base_footprint sits at the rear
  axle, chassis extends 40 mm behind it); obstacle sources are `/lidar_node/scan`
  and `/sen0628/pointcloud`.
- **Right turns need the patched Smac overlay.** Upstream's analytic-expansion
  scoring extrapolates one sample gap across the whole expansion; that gap
  collapses across a cusp, so a path starting with a reversal scores at a
  fraction of its true length and wins. Every right-hand turn cusped as a
  result (`corner_right`: 105 direction reversals vs `corner_left`'s 0).
  `.repos` points at our fork (`vis81/navigation2`, branch `picar2/1.3.12`),
  which carries the fix plus a `COLCON_IGNORE` in every other package, so only
  `nav2_smac_planner` is built and one branch covers the PC's nav2 1.3.12 and
  the Pi's 1.3.11. `make nav2-overlay` fetches it standalone (the Pi needs
  this — `sync2pi` skips it). See `docs/nav2-smac-right-turn.md`.
- `rcl_yaml_param_parser` does **not** support YAML anchors — an attempt to
  deduplicate the costmap blocks with them was reverted.

`scripts/loop_waypoints.py` drives `NavigateThroughPoses` with a rolling window so
the robot laps a waypoint list without stopping. Waypoint YAML matches the RViz
Nav2 panel "Save Waypoints" format (orientation is **w-first**).

## SLAM

`config/cartographer.lua`: `tracking_frame = imu_link`, `published_frame = odom`,
`use_odometry` and `use_imu_data` both on, online correlative scan matching,
range 0.06–5.0 m, `pose_publish_period_sec = 20e-3` (200 Hz was too much for the Pi).

Cartographer is the baseline that works — it feeds the IMU straight into scan
matching, whereas slam_toolbox leans on wheel odometry and suffers from Ackermann
calibration error. Don't regress to slam_toolbox without a reason. Note
`slam-localize` must use `localization_slam_toolbox_node`; `async_slam_toolbox_node`
ignores `/initialpose` regardless of its `mode` parameter.

## Calibration

| What | Where | How |
|---|---|---|
| Wheel radius | `controllers.yaml` `traction_wheels_radius` | straight-line test, `make odom-cal` |
| Steering LUT | `picar2.urdf.xacro` `steer_lut_us` / `steer_lut_rad` | measured wheel angles; refine with the odom_cal circle test |
| Accel/gyro | `config/imu_calib.yaml` | `make imu-calib` (6-position routine, needs bringup running) |
| Magnetometer hard-iron | `config/magnetometer_calib.yaml` | `make mag-calib` on the PC, then `ros2 service call /calibrate_magnetometer std_srvs/srv/Trigger {}` and rotate 360° for ~30 s |
| Servo centers, gyro bias | STM32 settings (persistent) | Zephyr shell on `/dev/ttyYahboom1` |
| IMU sanity check | — | `make imu-verify` (`imu_verify.py`) |

## Devices (udev, `etc/99-picar.rules`)

`/dev/ttyYahboom0` (STM32 protocol, 460800) · `/dev/ttyYahboom1` (Zephyr shell,
921600) · `/dev/ldlidar` (LD19) · `/dev/ld07` · `/dev/sen0628`.
`make install-uarts` also installs `zephyr-shell.service`, exposing the shell on
TCP 4444 (`putty -raw <pi-ip> 4444`).

## Topics

| Topic | Type | Source |
|---|---|---|
| `/cmd_vel` | Twist | teleop / Nav2 → relayed to `/ackermann_steering_controller/reference` |
| `/ackermann_steering_controller/odometry` | Odometry | wheel odometry |
| `/imu/data_raw` → `/imu/data_corrected` → `/imu/data` | Imu | hardware → imu_calib → Madgwick |
| `/imu/mag` → `/imu/mag_unbiased` | MagneticField | hardware → bias remover |
| `/odom` | Odometry | EKF (remapped from `/odometry/filtered`) |
| `/lidar_node/scan` | LaserScan | lidar driver (both models remap to this) |
| `/sen0628/pointcloud` | PointCloud2 | SEN0628 ToF |
| `/joint_states` | JointState | joint_state_broadcaster |
| `/pan_tilt_controller/commands` | Float64MultiArray | camera pan/tilt (also driven by the WebXR FPV client) |

## Pitfalls

- **Never hand-edit `.rviz` files.** A missing or incomplete `Views/Current` block
  makes the Ogre render panel silently ignore mouse input (Qt menus keep working).
  Configure in RViz and use File → Save Config As.
- **Never run `make sync2pi`** or any rsync to the Pi — syncing is done by hand.
- **PC tools need `PI_IP` + `PC_IFACE`.** Empty values fail silently.
- **Clock skew breaks TF** across Pi/PC. chrony setup is in the README.
- **Steer units**: the CMD_VEL steer field is a µs delta from the servo center, not
  degrees or radians. The LUT is the only rad↔µs conversion.
- **`--symlink-install`** means Python and config edits under `src/` take effect
  without rebuilding; C++ changes still need `make build`.
- **Docker vs host build trees** are separate (`install/` vs `install-docker/`).
  Sourcing the wrong one gives confusing "package not found" errors.
- Third-party Gazebo worlds usually need `scripts/patch_gz_world.py` before they
  will load.
