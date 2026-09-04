IMAGE   := picar2-ros2:jazzy
WS      := $(CURDIR)
DISPLAY ?= :0
ARCH    := $(shell uname -m)
ifeq ($(ARCH),aarch64)
BASE_IMAGE := ros:jazzy-ros-base
else
BASE_IMAGE := osrf/ros:jazzy-desktop
endif

PI_IP    ?=
PC_IFACE ?=
FOCUS    ?= imu
FORCE    ?=
LIDAR    ?= ld19   # ld19 | lds02rr | none
# base name (no extension) under maps/ for save-map / slam-resume
MAP      ?= home
# Available worlds (from src/robotnik_gazebo_worlds/):
#   room | electrical_substation | opil_factory | rubber_factory | warehouse |
#   photovoltaic_station | robotnik_lab | robotnik_lab_simplifyed
WORLD    ?= room
SPAWN_X  ?= 0
SPAWN_Y  ?= 0
HEADLESS ?= false   # true = server-only (no Gazebo GUI window)
USE_MAG     ?= true
USE_LD07    ?= false
USE_SEN0628 ?= true
USE_FOXGLOVE ?= true
FOXGLOVE_PORT ?= 8765
USE_VIZANTI ?= true
VIZANTI_PORT ?= 5000
VIZANTI_ROSBRIDGE_PORT ?= 5001
SEN0628_PORT ?= /dev/sen0628
IMU_ARGS ?=
LD07_PORT ?= /dev/ttyUSB0
USE_JOY    ?= false
JOY_ID     ?= 0
# picar2_bringup/config/<name>.yaml — ps4 | flysky
JOY_CONFIG ?= ps4

# host: run commands directly (ROS must be installed and sourced on the host)
# docker: wrap each command in an appropriate Docker container
EXEC_ENV       ?= host      # host | docker
CONTAINER_NAME ?= picar2   # persistent container name for docker-start / docker-stop

# ── Path and build directories (differ inside docker vs. on host) ────────────
ifeq ($(EXEC_ENV),docker)
WS_PATH      := /ws
BUILD_BASE   := build-docker
INSTALL_BASE := install-docker
else
WS_PATH      := $(WS)
BUILD_BASE   := build
INSTALL_BASE := install
endif

# ── World file path resolution ───────────────────────────────────────────────
ifeq ($(WORLD),room)
_WORLD_PATH      := $(WS_PATH)/$(INSTALL_BASE)/picar2_bringup/share/picar2_bringup/worlds/room.sdf
_GZ_RESOURCE_PATH := $(WS_PATH)/$(INSTALL_BASE)/picar2_description/share
else
_WORLD_PATH      := $(WS_PATH)/$(INSTALL_BASE)/$(WORLD)_world/share/$(WORLD)_world/worlds/$(WORLD).world
_GZ_RESOURCE_PATH := $(WS_PATH)/$(INSTALL_BASE)/$(WORLD)_world/share/$(WORLD)_world/models:$(WS_PATH)/$(INSTALL_BASE)/picar2_description/share
endif

# ── ROS setup strings (inlined into every bash -c command) ───────────────────
ROS_SETUP    := export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && \
                export CYCLONEDDS_URI=file://$(WS_PATH)/cyclonedds.xml && \
                source /opt/ros/jazzy/setup.bash && \
                source $(WS_PATH)/$(INSTALL_BASE)/setup.bash

ROS_SETUP_PC := export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && \
                export CYCLONEDDS_URI=file://$(WS_PATH)/cyclonedds-pc.xml && \
                export PI_IP=$(PI_IP) && export PC_IFACE=$(PC_IFACE) && \
                source /opt/ros/jazzy/setup.bash && \
                source $(WS_PATH)/$(INSTALL_BASE)/setup.bash

# ── Navigation benchmark ─────────────────────────────────────────────────────
# Variables first: BENCH_SETUP is := (immediate), so anything it references has
# to exist already. Comments go on their own lines — a trailing comment leaves
# its preceding whitespace in the value, which then reaches the command line.
BENCH_DOMAIN ?= 42
# open_straight | doorway | dead_end_reverse
SCENARIO     ?= open_straight
# ground_truth | slam | amcl   (only ground_truth has been validated)
BENCH_MODE   ?= ground_truth
# config name from picar2_benchmark/configs, e.g. mppi_ackermann
OVERLAY      ?=
# 0.0 removes all simulated sensor noise
NOISE        ?= 1.0
RUNS         ?= 1
BENCH_OUT    ?= /tmp/picar2_bench/results

# Deliberately NOT ROS_SETUP: the benchmark runs on its own DDS domain with no
# cyclonedds.xml, so a sim on this machine can never talk to the robot or to a
# second benchmark run.
BENCH_SETUP  := export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && \
                unset CYCLONEDDS_URI && \
                export ROS_DOMAIN_ID=$(BENCH_DOMAIN) && \
                source /opt/ros/jazzy/setup.bash && \
                source $(WS_PATH)/$(INSTALL_BASE)/setup.bash

_BENCH_SHARE := $(WS_PATH)/$(INSTALL_BASE)/picar2_benchmark/share/picar2_benchmark
_SCENARIO_YML := $(_BENCH_SHARE)/scenarios/$(SCENARIO).yaml
_OVERLAY_ARG := $(if $(OVERLAY),--overlay $(_BENCH_SHARE)/configs/$(OVERLAY).yaml,)

# ── docker-start flags ───────────────────────────────────────────────────────
# gpio group name is not in the ROS base image — resolve its GID from
# /dev/gpiomem on the host at make time. Same idea for the `input` group
# (needed for /dev/input/event* under SDL2 in joy_node).
_GID_GPIO  := $(shell stat -c '%g' /dev/gpiomem 2>/dev/null)
_GID_INPUT := $(shell getent group input | awk -F: '{print $$3}')

_DOCKER_FLAGS := --privileged --network host --ipc host \
                 -u $(shell id -u):$(shell id -g) \
                 --group-add dialout \
                 --group-add plugdev \
                 --group-add kmem \
                 $(if $(_GID_GPIO),--group-add $(_GID_GPIO)) \
                 $(if $(_GID_INPUT),--group-add $(_GID_INPUT)) \
                 -e DISPLAY=$(DISPLAY) -e QT_XCB_NO_XI2=1 \
                 -v /tmp/.X11-unix:/tmp/.X11-unix \
                 -v /dev:/dev \
                 -v $(WS):/ws -w /ws

# Flags for the boot services (webui-setup). Same device/network access as
# _DOCKER_FLAGS but without the X11 and DISPLAY bits, which are useless
# headless and would bake a stale :0 into the unit file.
_SVC_DOCKER_FLAGS := --privileged --network host --ipc host \
                     -u $(shell id -u):$(shell id -g) \
                     --group-add dialout --group-add plugdev --group-add kmem \
                     $(if $(_GID_GPIO),--group-add $(_GID_GPIO)) \
                     $(if $(_GID_INPUT),--group-add $(_GID_INPUT)) \
                     -v /dev:/dev -v $(WS):/ws -w /ws

# ── Execution wrapper ────────────────────────────────────────────────────────
# host:   run commands directly via bash -c
# docker: exec into the persistent $(CONTAINER_NAME) container started by
#         `make docker-start` (which has all required flags pre-applied)
ifeq ($(EXEC_ENV),docker)
  CMD   := docker exec -it $(CONTAINER_NAME) bash -c
  XHOST := xhost +local:docker 2>/dev/null || true
else
  CMD   := bash -c
  XHOST := true
endif

.PHONY: all image image-pi image-push build deps pull status push firmware flash rviz rqt bringup sim slam slam-sim slam-resume slam-localize save-map cartographer cartographer-resume cartographer-localize save-cartographer-map amcl nav nav-sim explore explore-sim bench bench-keep bench-gen bench-report bench-gui bench-rviz teleop joystick \
        odom-cal imu-calib imu-verify mag-calib lidar-ld19 lidar-ld07 lidar-ld07-view sen0628 sen0628-view foxglove vizanti debug diag shell docker-shell \
        docker-start docker-stop sync2pi softap softap-down install-uarts webui webui-setup webui-stop fpv-setup fpv fpv-stop clean

all: build

# ── Docker image ─────────────────────────────────────────────────────────────
image:
	docker build --build-arg BASE_IMAGE=$(BASE_IMAGE) -t $(IMAGE) .

# Build arm64 image on PC via QEMU emulation, then push to Pi.
# One-time setup: sudo apt-get install docker-buildx-plugin
#                 docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
#                 docker buildx create --name multiarch && docker buildx use multiarch && docker buildx inspect --bootstrap
image-pi:
	docker buildx build --platform linux/arm64 --build-arg BASE_IMAGE=ros:jazzy-ros-base -t $(IMAGE) --load .

image-push:
	docker save $(IMAGE) | gzip | ssh pi@$(PI_IP) 'docker load'

# ── Persistent container (use with EXEC_ENV=docker) ──────────────────────────
# Starts a long-lived container with all necessary flags. All make targets
# with EXEC_ENV=docker exec into it. Run docker-stop when done.
docker-start:
	$(XHOST)
	docker run -d --name $(CONTAINER_NAME) $(_DOCKER_FLAGS) $(IMAGE) sleep infinity
	@echo "Container '$(CONTAINER_NAME)' started. Run targets with EXEC_ENV=docker."

docker-stop:
	docker rm -f $(CONTAINER_NAME)

# ── Build & source management ────────────────────────────────────────────────
deps:
	bash -c "mkdir -p $(WS)/src && vcs import $(WS)/src < $(WS)/.repos && touch $(WS)/src/yahboom/COLCON_IGNORE"

ifeq ($(FORCE),)
pull:
	git -C $(WS) pull
	vcs pull $(WS)/src
else
pull:
	git -C $(WS) fetch --all && git -C $(WS) reset --hard @{u}
	@for d in $(WS)/src/*/; do \
	  git -C "$$d" fetch --all 2>/dev/null && \
	  git -C "$$d" reset --hard @{u} 2>/dev/null || true; \
	done
endif

status:
	git -C $(WS) status
	vcs status $(WS)/src --hide-empty

push:
	git -C $(WS) push $(if $(FORCE),--force-with-lease)
	@for d in $(WS)/src/*/; do \
	  git -C "$$d" remote get-url origin 2>/dev/null | grep -q 'vis81' && \
	    echo "=== $$d ===" && git -C "$$d" push $(if $(FORCE),--force-with-lease) || true; \
	done

build:
	$(CMD) "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --build-base $(BUILD_BASE) --install-base $(INSTALL_BASE) --packages-ignore multirobot_map_merge"

clean:
	$(CMD) "rm -rf $(WS_PATH)/$(BUILD_BASE) $(WS_PATH)/$(INSTALL_BASE) $(WS_PATH)/log"

# ── Firmware (Zephyr/west, host-only) ────────────────────────────────────────
firmware:
	bash -c "cd $(WS)/src/yahboom && source activate.sh && make"

flash:
	bash -c "cd $(WS)/src/yahboom && source activate.sh && make flash"

# ── Robot bringup (creates the named 'picar2' container in docker mode) ──────
bringup:
	$(XHOST)
	$(CMD) "$(ROS_SETUP) && ros2 launch picar2_bringup picar2.launch.py lidar:=$(LIDAR) use_mag:=$(USE_MAG) use_ld07:=$(USE_LD07) use_sen0628:=$(USE_SEN0628) sen0628_port:=$(SEN0628_PORT) use_foxglove:=$(USE_FOXGLOVE) foxglove_port:=$(FOXGLOVE_PORT) use_vizanti:=$(USE_VIZANTI) vizanti_port:=$(VIZANTI_PORT) vizanti_rosbridge_port:=$(VIZANTI_ROSBRIDGE_PORT) use_joy:=$(USE_JOY) joy_dev:=$(JOY_ID) joy_config:=$(JOY_CONFIG)"

sim:
	$(XHOST)
	$(eval _PATCHED_WORLD := $(shell python3 $(WS)/scripts/patch_gz_world.py $(_WORLD_PATH)))
	$(CMD) "export GZ_SIM_RESOURCE_PATH=$(_GZ_RESOURCE_PATH) && $(ROS_SETUP) && ros2 launch picar2_bringup sim.launch.py lidar:=$(LIDAR) use_mag:=$(USE_MAG) use_ld07:=$(USE_LD07) use_sen0628:=$(USE_SEN0628) headless:=$(HEADLESS) world:=$(_PATCHED_WORLD) spawn_x:=$(SPAWN_X) spawn_y:=$(SPAWN_Y)"

# ── Attach targets — exec into running bringup session (docker) / run directly (host) ──
slam:
	$(CMD) "$(ROS_SETUP) && ros2 launch picar2_bringup slam.launch.py"

slam-sim:
	$(CMD) "$(ROS_SETUP) && ros2 launch picar2_bringup slam.launch.py use_sim_time:=true"

# Resume from a previously-saved pose graph. Robot is assumed to be at the
# same physical pose where save-map ran (map_start_at_dock=true).
slam-resume:
	$(CMD) "$(ROS_SETUP) && ros2 launch picar2_bringup slam.launch.py map_file:=$(WS_PATH)/maps/$(MAP)"

# Load a saved map and run slam_toolbox in localization mode — the map is
# frozen, the robot tracks against it. After launch, set the initial pose
# in RViz with the '2D Pose Estimate' tool (publishes to /initialpose).
# Uses localization_slam_toolbox_node because async_slam_toolbox_node does
# NOT subscribe to /initialpose, regardless of the `mode` parameter.
slam-localize:
	$(CMD) "$(ROS_SETUP) && ros2 launch picar2_bringup slam.launch.py map_file:=$(WS_PATH)/maps/$(MAP) mode:=localization executable:=localization_slam_toolbox_node"

# Save both the slam_toolbox pose graph (.posegraph + .data — for resume)
# and a PGM/YAML occupancy grid (for Nav2 map_server). MAP=<name>, default 'home'.
save-map:
	$(CMD) "$(ROS_SETUP) && mkdir -p $(WS_PATH)/maps && \
	  ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \"{filename: '$(WS_PATH)/maps/$(MAP)'}\" && \
	  ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \"{name: {data: '$(WS_PATH)/maps/$(MAP)'}}\""

cartographer:
	$(CMD) "$(ROS_SETUP) && ros2 launch picar2_bringup cartographer.launch.py"

# Save cartographer state. Produces:
#   maps/$(MAP).pbstream            — cartographer's native serialization
#   maps/$(MAP).pgm + $(MAP).yaml   — Nav2 map_server occupancy grid
# finish_trajectory(0) seals the active trajectory so write_state writes a
# clean state. Trajectory id 0 is the first trajectory (fresh `make
# cartographer`); use a higher id if you've called start_trajectory yourself.
save-cartographer-map:
	$(CMD) "$(ROS_SETUP) && mkdir -p $(WS_PATH)/maps && \
	  ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory \"{trajectory_id: 0}\" && \
	  ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \"{filename: '$(WS_PATH)/maps/$(MAP).pbstream', include_unfinished_submaps: false}\" && \
	  ros2 run nav2_map_server map_saver_cli -f $(WS_PATH)/maps/$(MAP) --ros-args -p map_subscribe_transient_local:=true"

# Resume cartographer from a saved pbstream (keeps mapping on top).
cartographer-resume:
	$(CMD) "$(ROS_SETUP) && ros2 launch picar2_bringup cartographer.launch.py load_state:=$(WS_PATH)/maps/$(MAP).pbstream"

# Localize on a saved pbstream without adding to it (pure localization).
cartographer-localize:
	$(CMD) "$(ROS_SETUP) && ros2 launch picar2_bringup cartographer.launch.py load_state:=$(WS_PATH)/maps/$(MAP).pbstream load_frozen:=true"

# AMCL localization on a saved PGM/YAML. Backend-agnostic — works on
# maps produced by either save-map (slam_toolbox) or save-cartographer-map.
# After launch, set initial pose via RViz "2D Pose Estimate".
amcl:
	$(CMD) "$(ROS_SETUP) && ros2 launch picar2_bringup amcl.launch.py map_yaml:=$(WS_PATH)/maps/$(MAP).yaml"

nav:
	$(CMD) "$(ROS_SETUP) && ros2 launch picar2_bringup nav2.launch.py"

nav-sim:
	$(CMD) "$(ROS_SETUP) && ros2 launch picar2_bringup nav2.launch.py use_sim_time:=true"

explore:
	$(CMD) "$(ROS_SETUP) && ros2 launch picar2_bringup explore.launch.py"

explore-sim:
	$(CMD) "$(ROS_SETUP) && ros2 launch picar2_bringup explore.launch.py use_sim_time:=true"

# Run one measured navigation trial. Trials are serial by design: two sims on
# one machine contend for CPU, which silently becomes an uncontrolled variable.
#   make bench SCENARIO=dead_end_reverse
#   make bench SCENARIO=doorway BENCH_MODE=slam
#   make bench SCENARIO=dead_end_reverse OVERLAY=mppi_ackermann RUNS=4
bench:
	$(CMD) "$(BENCH_SETUP) && for i in \$$(seq 1 $(RUNS)); do \
	  echo \"--- $(SCENARIO) [$(BENCH_MODE)] run \$$i/$(RUNS)\" && \
	  ros2 run picar2_benchmark bench-run $(_SCENARIO_YML) \
	    --mode $(BENCH_MODE) --sensor-noise $(NOISE) $(_OVERLAY_ARG) \
	    -o $(BENCH_OUT); done"

# Leave the stack up afterwards so RViz/Gazebo can be attached.
bench-keep:
	$(CMD) "$(BENCH_SETUP) && ros2 run picar2_benchmark bench-run $(_SCENARIO_YML) \
	  --mode $(BENCH_MODE) --sensor-noise $(NOISE) $(_OVERLAY_ARG) \
	  -o $(BENCH_OUT) --keep-up"

# Write a scenario's world and static map without running anything.
bench-gen:
	$(CMD) "$(BENCH_SETUP) && ros2 run picar2_benchmark bench-generate $(_SCENARIO_YML) \
	  -o /tmp/picar2_bench"

# Summarise trials: median + IQR + range, outcomes as a distribution.
#   make bench-report GROUP_BY=config
GROUP_BY ?= config

bench-report:
	$(CMD) "$(BENCH_SETUP) && ros2 run picar2_benchmark bench-report $(BENCH_OUT) \
	  --group-by $(GROUP_BY)"

# Attach viewers to a `make bench-keep` session.
bench-gui:
	$(CMD) "$(BENCH_SETUP) && gz sim -g"

bench-rviz:
	$(CMD) "$(BENCH_SETUP) && rviz2 -d $(WS_PATH)/$(INSTALL_BASE)/picar2_bringup/share/picar2_bringup/config/rviz.rviz --ros-args -p use_sim_time:=true"

teleop:
	$(CMD) "$(ROS_SETUP) && ros2 run teleop_twist_keyboard teleop_twist_keyboard"

# JOY_CONFIG picks the controller layout: ps4 (DualShock 4) or flysky (RC TX).
# JOY_ID is the device index (usually 0 = /dev/input/js0). Override on the CLI:
#   make joystick                       # PS4 on js0  (default)
#   make joystick JOY_CONFIG=flysky     # FlySky RC
#   make joystick JOY_ID=1              # second joystick device
joystick:
	$(CMD) "$(ROS_SETUP) && ros2 launch teleop_twist_joy teleop-launch.py joy_dev:=$(JOY_ID) config_filepath:=$(WS_PATH)/$(INSTALL_BASE)/picar2_bringup/share/picar2_bringup/config/$(JOY_CONFIG).yaml"

# Requires bringup running. Guides through 6-position accel calibration.
# Output saved to src/picar2_bringup/config/imu_calib.yaml
imu-calib:
	$(CMD) "$(ROS_SETUP) && ros2 run imu_calib do_calib_node --ros-args -r imu:=/imu/data_raw -p calib_file:=$(WS_PATH)/src/picar2_bringup/config/imu_calib.yaml"

# Requires bringup running (Pi). Starts bias observer + remover on PC.
# Trigger calibration in another terminal:
#   ros2 service call /calibrate_magnetometer std_srvs/srv/Trigger {}
# Then rotate the robot 360° for ~30 s. Bias saved to picar2_bringup/config/magnetometer_calib.yaml
mag-calib:
	$(XHOST)
	$(CMD) "$(ROS_SETUP_PC) && ros2 launch magnetometer_pipeline bias_remover.launch \
		two_d_mode:=true \
		calibration_file_path:=$(WS_PATH)/src/picar2-ros2/picar2_bringup/config/magnetometer_calib.yaml"

diag:
	$(CMD) "bash $(WS_PATH)/scripts/diag.sh"

# ── Standalone runtime ───────────────────────────────────────────────────────
imu-verify:
	$(CMD) "$(ROS_SETUP) && ros2 run picar2_bringup imu_verify.py $(IMU_ARGS)"

debug:
	$(CMD) "bash $(WS_PATH)/scripts/debug.sh $(FOCUS)"

lidar-ld19:
	$(CMD) "$(ROS_SETUP) && ros2 launch ldlidar_node ldlidar_with_mgr.launch.py"

lidar-ld07:
	$(CMD) "$(ROS_SETUP) && ros2 launch ldrobot_ld07 ld07.launch.py serial_port:=$(LD07_PORT)"

lidar-ld07-view:
	$(XHOST)
	$(CMD) "$(ROS_SETUP_PC) && ros2 run rviz2 rviz2 -d $(WS_PATH)/install/ldrobot_ld07/share/ldrobot_ld07/config/ld07.rviz"

sen0628:
	$(CMD) "$(ROS_SETUP) && ros2 launch tof_imager_ros tof_imager.launch.py"

sen0628-view:
	$(XHOST)
	$(CMD) "$(ROS_SETUP_PC) && ros2 run rviz2 rviz2 -d $(WS_PATH)/install/tof_imager_ros/share/tof_imager_ros/config/sen0628.rviz"

# Standalone foxglove_bridge — connect from Foxglove Studio to ws://<host>:8765
foxglove:
	$(CMD) "$(ROS_SETUP) && ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=$(FOXGLOVE_PORT)"

# Standalone Vizanti — open http://<host>:5000 in a browser
vizanti:
	$(CMD) "$(ROS_SETUP) && ros2 launch vizanti_server vizanti_server.launch.py port:=$(VIZANTI_PORT) port_rosbridge:=$(VIZANTI_ROSBRIDGE_PORT) flask_debug:=False"

# ── PC visualisation / calibration tools ────────────────────────────────────
rviz:
	$(XHOST)
	$(CMD) "$(ROS_SETUP_PC) && ros2 run rviz2 rviz2 -d $(WS_PATH)/install/picar2_bringup/share/picar2_bringup/config/rviz.rviz"

rqt:
	$(XHOST)
	$(CMD) "$(ROS_SETUP_PC) && rqt"

odom-cal:
	$(XHOST)
	$(CMD) "$(ROS_SETUP_PC) && ros2 run picar2_bringup odom_cal.py"

# ── Shell access ─────────────────────────────────────────────────────────────
# shell: interactive bash inside the running container (host or docker mode)
shell:
	$(CMD) "$(ROS_SETUP) && exec bash"

# docker-shell: always execs into $(CONTAINER_NAME) regardless of EXEC_ENV
docker-shell:
	docker exec -it $(CONTAINER_NAME) bash -c "source /opt/ros/jazzy/setup.bash && source /ws/install-docker/setup.bash && exec bash"

# ── Sync to Pi ───────────────────────────────────────────────────────────────
sync2pi:
	rsync -avz --exclude '.git' --exclude 'build' --exclude 'install' --exclude 'log' \
		--exclude 'build-docker' --exclude 'install-docker' --exclude 'setenv.sh' \
		. pi@$(PI_IP):~/picar_ws/

# ── SoftAP (host-only — configures NetworkManager on the Pi, not in container) ─
# Concurrent AP+STA on wlan0 (creates uap0 virtual interface).
# Usage:
#   make softap SSID=picar PASS=mysecret123
#   make softap-down
SSID ?= picar-ap
PASS ?= changeme1234

softap:
	sudo SSID='$(SSID)' PASS='$(PASS)' bash $(WS)/etc/setup-softap.sh

softap-down:
	sudo bash $(WS)/etc/setup-softap.sh --teardown

# ── Host UART setup — udev symlinks + zephyr-shell socat bridge ──────────────
# Installs /etc/udev/rules.d/99-picar.rules (symlinks: ttyYahboom{0,1},
# ldlidar, ld07, sen0628) and /etc/systemd/system/zephyr-shell.service which
# exposes ttyYahboom1 over TCP 4444 via socat for `putty -raw <pi-ip> 4444`.
install-uarts:
	sudo apt-get install -y socat
	sudo cp $(WS)/etc/99-picar.rules /etc/udev/rules.d/
	sudo udevadm control --reload-rules && sudo udevadm trigger
	sudo cp $(WS)/etc/zephyr-shell.service /etc/systemd/system/
	sudo systemctl daemon-reload
	sudo systemctl enable --now zephyr-shell
	@echo "Done — unplug and replug USB adapters, then connect via: putty -raw <pi-ip> 4444"

# ── Phone web UI — map building ──────────────────────────────────────────────
# webui:       run the backend in the foreground (bringup must already be up).
# webui-setup: install picar-bringup + picar-webui as boot services, so the
#              phone is the only interface needed — power on, open the page.
WEBUI_PORT ?= 8080

webui:
	$(CMD) "$(ROS_SETUP) && PICAR_WS=$(WS_PATH) PICAR_WEBUI_PORT=$(WEBUI_PORT) python3 $(WS_PATH)/etc/webui/server.py"

# The services run in the picar2-ros2 image, so flask/waitress come from the
# image, not the host. INSTALL is install-docker because that is what the
# container builds into.
webui-setup:
	@for unit in picar-bringup picar-webui; do \
	  sed -e 's|@FLAGS@|$(_SVC_DOCKER_FLAGS)|g' \
	      -e 's|@IMAGE@|$(IMAGE)|g' \
	      -e 's|@INSTALL@|install-docker|g' \
	      -e 's|@LIDAR@|$(strip $(LIDAR))|g' \
	      -e 's|@USE_JOY@|$(strip $(USE_JOY))|g' \
	      -e 's|@PORT@|$(WEBUI_PORT)|g' \
	      $(WS)/etc/webui/$$unit.service | sudo tee /etc/systemd/system/$$unit.service >/dev/null; \
	done
	sudo systemctl daemon-reload
	sudo systemctl enable --now picar-bringup picar-webui
	@ip="$$(hostname -I | awk '{print $$1}')"; \
	 echo ""; \
	 echo "  Open on your phone:  http://$$ip:$(WEBUI_PORT)/"; \
	 echo ""

webui-stop:
	sudo systemctl stop picar-webui picar-bringup

# ── FPV (Meta Quest 3) — MediaMTX on host (not docker) + WebXR client ────────
# fpv-setup: one-time installer (libcamera-apps, MediaMTX, TLS cert, systemd unit).
# fpv:       starts the mediamtx service; ROS bringup must be up separately for
#            pan/tilt to actually move (rosbridge inside the picar2 container).
# fpv-stop:  stops the mediamtx service.
#
# Open https://<pi-ip>:8889/  on Quest 3 → accept self-signed cert →
#   click "Enter VR" → head pose drives /pan_tilt_controller/commands.
fpv-setup:
	sudo WS_DIR=$(WS) bash $(WS)/etc/fpv/install-mediamtx.sh

fpv:
	sudo systemctl start mediamtx picar-fpv-ui
	@ip="$$(hostname -I | awk '{print $$1}')"; \
	 echo ""; \
	 echo "  Quest WebXR client:    https://$$ip:8443/"; \
	 echo "  Camera stream (test):  https://$$ip:8889/cam/"; \
	 echo ""

fpv-stop:
	sudo systemctl stop mediamtx picar-fpv-ui
