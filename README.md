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

## Time synchronisation (chrony)

Accurate clock agreement between PC and Pi is required for TF lookups over LAN.
Without it, sensor timestamps arrive slightly in the future on the PC side, causing
`tf2` extrapolation errors and ghost points in RViz.

Docker containers inherit the host clock — no extra chrony config is needed inside containers.

### PC (NTP server)

```bash
sudo apt install chrony

# /etc/chrony/chrony.conf — add these lines:
#   allow 192.168.0.0/16   # allow Pi to query this PC
#   local stratum 8        # serve time even when not synced upstream

sudo systemctl restart chrony
chronyc clients             # verify Pi appears here after Pi is configured
```

### Pi (NTP client)

```bash
sudo apt install chrony

# /etc/chrony/chrony.conf — replace default pool lines with:
#   server <PC_IP> iburst prefer

sudo systemctl restart chrony
chronyc sources -v          # verify PC shows as * (selected source)
chronyc tracking            # check offset — should settle below ±5 ms
```

### Verify sync is working

```bash
# On Pi:
chronyc sources -v
# Should show one line with * (selected) pointing at your PC IP

# On PC — confirm Pi is a client:
chronyc clients

# Quick offset check from either side:
chronyc tracking | grep "System time"
```

## PS4 (DualShock 4) controller

The robot supports a PS4 controller via `teleop_twist_joy` for manual driving.
The R1 button is the deadman; right stick drives. Layout lives in
`src/picar2-ros2/picar2_bringup/config/ps4.yaml`.

Launch options:

```bash
make joystick                                  # standalone (ps4 layout default)
make bringup USE_JOY=true                      # joystick along with the rest
make bringup USE_JOY=true JOY_CONFIG=flysky    # use FlySky RC transmitter
make bringup USE_JOY=true JOY_ID=1             # second joystick device
```

### USB

Plug the controller into any USB port on the Pi. It enumerates as
`/dev/input/js0` and is immediately usable — no pairing needed.

### Bluetooth (one-time pairing)

#### 1. Enable Bluetooth on the Pi (one-time)

The default robot image disables on-board Bluetooth via
`dtoverlay=disable-bt` so the GPIO UART is available. To use Bluetooth,
swap that for `miniuart-bt` (BT on the slower mini-UART, GPIO UART
preserved):

```bash
# Verify GPIO UART is unused
sudo lsof /dev/serial0 /dev/ttyAMA0           # should return nothing

# Swap the overlay
sudo sed -i 's/^dtoverlay=disable-bt$/dtoverlay=miniuart-bt/' /boot/firmware/config.txt
grep -n 'miniuart\|disable-bt' /boot/firmware/config.txt    # verify

sudo reboot
```

After reboot, confirm BT is up:

```bash
systemctl status bluetooth      # active (running)
hciconfig hci0                  # UP RUNNING
```

#### 2. Pair the controller

Unplug any USB connection to the controller first (can't pair while
tethered). Then:

```bash
sudo bluetoothctl
```

Inside the `bluetoothctl` prompt:

```
agent on
default-agent
scan on
```

On the controller, **hold Share + PS** for ~3 s until the lightbar
flashes white rapidly. A line appears in `bluetoothctl`:

```
[NEW] Device A4:53:85:XX:XX:XX Wireless Controller
```

Copy the MAC and run (in the same session):

```
pair A4:53:85:XX:XX:XX
trust A4:53:85:XX:XX:XX
connect A4:53:85:XX:XX:XX
scan off
quit
```

`trust` is critical — without it the controller would need to be paired
again after every disconnect.

#### 3. Verify

```bash
ls /dev/input/js0               # exists when connected
hciconfig hci0                  # ACL/sco RX bytes increasing
```

### Day-to-day use

- **Connect**: press the PS button briefly. Lightbar flashes then goes
  solid. `/dev/input/js0` appears within a second.
- **Disconnect**: hold the PS button for 10 s, or
  `bluetoothctl disconnect <MAC>`.
- The Pi remembers the controller across reboots once trusted.
- DualShock 4 only remembers **one host at a time** — re-pairing is
  required if you connect it to your phone or another PC in between.

### Troubleshooting

| Symptom | Fix |
|---|---|
| `bluetooth.service` is `inactive (dead)` | `dtoverlay=disable-bt` is still in `config.txt` — swap to `miniuart-bt` and reboot |
| `hciconfig hci0` shows `DOWN` | `sudo hciconfig hci0 up` |
| `Failed to pair (org.bluez.Error.AlreadyExists)` | In `bluetoothctl`: `remove <MAC>` first, then re-pair |
| `/dev/input/js0` exists but `make joystick` says no /joy | Container missing `input` group access — see Makefile `--group-add input`; restart container with `make docker-stop && make docker-start` |
| `Package 'teleop_twist_joy' not found` | `ros-jazzy-joy` + `ros-jazzy-teleop-twist-joy` aren't in the running image — run `make image` to bake them in, or `docker exec -u root picar2 apt-get install -y ros-jazzy-joy ros-jazzy-teleop-twist-joy` for a one-off |
