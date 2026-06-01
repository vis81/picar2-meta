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
USE_MAG     ?= true
USE_LD07    ?= false
USE_SEN0628 ?= true
SEN0628_PORT ?= /dev/sen0628
IMU_ARGS ?=
LD07_PORT ?= /dev/ttyUSB0
JOY_ID   ?= 0

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

# ── docker-start flags ───────────────────────────────────────────────────────
# gpio group name is not in the ROS base image — resolve its GID from
# /dev/gpiomem on the host at make time.
_GID_GPIO := $(shell stat -c '%g' /dev/gpiomem 2>/dev/null)

_DOCKER_FLAGS := --privileged --network host --ipc host \
                 -u $(shell id -u):$(shell id -g) \
                 --group-add dialout \
                 --group-add plugdev \
                 --group-add kmem \
                 $(if $(_GID_GPIO),--group-add $(_GID_GPIO)) \
                 -e DISPLAY=$(DISPLAY) -e QT_XCB_NO_XI2=1 \
                 -v /tmp/.X11-unix:/tmp/.X11-unix \
                 -v /dev:/dev \
                 -v $(WS):/ws -w /ws

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

.PHONY: all image build deps pull status push firmware flash rviz rqt bringup sim slam slam-sim cartographer nav nav-sim explore teleop joystick \
        odom-cal imu-calib imu-verify mag-calib lidar-ld19 lidar-ld07 lidar-ld07-view sen0628 sen0628-view debug diag shell docker-shell \
        docker-start docker-stop sync2pi clean

all: build

# ── Docker image ─────────────────────────────────────────────────────────────
image:
	docker build --build-arg BASE_IMAGE=$(BASE_IMAGE) -t $(IMAGE) .

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
	$(CMD) "$(ROS_SETUP) && ros2 launch picar2_bringup picar2.launch.py lidar:=$(LIDAR) use_mag:=$(USE_MAG) use_ld07:=$(USE_LD07) use_sen0628:=$(USE_SEN0628) sen0628_port:=$(SEN0628_PORT)"

sim:
	$(XHOST)
	$(CMD) "export GZ_SIM_RESOURCE_PATH=$(WS_PATH)/$(INSTALL_BASE)/picar2_description/share && $(ROS_SETUP) && ros2 launch picar2_bringup sim.launch.py use_mag:=$(USE_MAG)"

# ── Attach targets — exec into running bringup session (docker) / run directly (host) ──
slam:
	$(CMD) "$(ROS_SETUP) && ros2 launch picar2_bringup slam.launch.py"

slam-sim:
	$(CMD) "$(ROS_SETUP) && ros2 launch picar2_bringup slam.launch.py use_sim_time:=true"

cartographer:
	$(CMD) "$(ROS_SETUP) && ros2 launch picar2_bringup cartographer.launch.py"

nav:
	$(CMD) "$(ROS_SETUP) && ros2 launch picar2_bringup nav2.launch.py"

nav-sim:
	$(CMD) "$(ROS_SETUP) && ros2 launch picar2_bringup nav2.launch.py use_sim_time:=true"

explore:
	$(CMD) "$(ROS_SETUP) && ros2 launch picar2_bringup explore.launch.py"

teleop:
	$(CMD) "$(ROS_SETUP) && ros2 run teleop_twist_keyboard teleop_twist_keyboard"

joystick:
	$(CMD) "$(ROS_SETUP) && ros2 launch teleop_twist_joy teleop-launch.py joy_dev:=$(JOY_ID) config_filepath:=$(WS_PATH)/$(INSTALL_BASE)/picar2_bringup/share/picar2_bringup/config/flysky.yaml"

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
		. pi@rpi4.local:~/picar_ws/
