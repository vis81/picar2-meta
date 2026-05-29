# PICAR-2 Workspace

Meta-repo for the PICAR-2 robot. Manages the ROS 2 packages, firmware, and external dependencies via vcstool.

## Prerequisites

- ROS 2 Jazzy (or Docker — see below)
- vcstool: `sudo apt install python3-vcstool`
- Docker (optional): `sudo apt install docker.io`

## Initialization

```bash
# 1. Clone this repo
git clone https://github.com/vis81/picar2.git picar2_ws
cd picar2_ws

# 2. Import all source repos (ROS packages, firmware, external deps)
make deps

# 3. Build ROS packages
make build
```

## Workspace layout (after `make deps`)

```
picar2_ws/
├── src/
│   ├── picar2-ros2/          # ROS 2 packages (bringup, control, description)
│   ├── yahboom/              # Zephyr firmware for STM32F103 control board
│   ├── lds02rr_lidar/        # LDS02RR LiDAR driver
│   ├── ldrobot_ld07/         # LD07 structured-light depth sensor driver
│   ├── ldrobot-lidar-ros2/   # LD19 LiDAR driver (external)
│   ├── explore_lite/         # Frontier exploration (external)
│   └── imu_calib/            # IMU calibration (external)
├── Makefile
├── Dockerfile
├── cyclonedds.xml            # DDS config (robot)
└── cyclonedds-pc.xml         # DDS config (PC tools)
```

## Common targets

| Target | Description |
|--------|-------------|
| `make deps` | Import all source repos via vcstool |
| `make build` | Build ROS packages with colcon |
| `make bringup` | Launch full robot stack |
| `make firmware` | Build Zephyr firmware |
| `make flash` | Flash firmware to STM32 |
| `make slam` | Launch SLAM (attach to running bringup) |
| `make nav` | Launch Nav2 |
| `make explore` | Launch frontier exploration |
| `make teleop` | Keyboard teleoperation |
| `make rviz` | RViz2 (PC, needs `PI_IP` and `PC_IFACE`) |
| `make shell` | Interactive shell in workspace |

## Docker mode

```bash
make image                        # build Docker image (once)
make docker-start                 # start persistent container
EXEC_ENV=docker make bringup      # run targets inside container
make docker-stop                  # stop container
```

## PC visualisation

```bash
make rviz PI_IP=<pi-ip> PC_IFACE=<iface>
```
