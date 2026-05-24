IMAGE   := picar2-ros2:jazzy
WS      := $(CURDIR)
DISPLAY ?= :0
# osrf/ros:jazzy-desktop is amd64-only; Pi uses the multi-arch ros-base
ARCH    := $(shell uname -m)
ifeq ($(ARCH),aarch64)
BASE_IMAGE := ros:jazzy-ros-base
else
BASE_IMAGE := osrf/ros:jazzy-desktop
endif
PI_IP    ?=
PC_IFACE ?=
ROS_ENV    := -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
              -e CYCLONEDDS_URI=file:///ws/cyclonedds.xml
ROS_ENV_PC := -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
              -e CYCLONEDDS_URI=file:///ws/cyclonedds-pc.xml \
              -e PI_IP=$(PI_IP) \
              -e PC_IFACE=$(PC_IFACE)

FOCUS    ?= imu
WHERE    ?= pi
IMU_ARGS ?=
LIDAR    ?= true
USE_MAG  ?= false

# WHERE=pi  → exec into running picar2 container (default, used on the Pi)
# WHERE=pc  → spin up a throwaway container on the PC connected via LAN DDS
ifeq ($(WHERE),pc)
DEBUG_RUN = docker run --rm -it \
	--network host \
	--ipc host \
	$(ROS_ENV_PC) \
	-v $(WS):/ws \
	-w /ws \
	$(IMAGE)
else
DEBUG_RUN = docker exec -it picar2
endif

.PHONY: all image build deps rviz rqt bringup sim slam cartographer nav explore teleop odom-cal imu-calib imu-verify debug diag shell clean

all: build

image:
	docker build --build-arg BASE_IMAGE=$(BASE_IMAGE) -t $(IMAGE) .

deps:
	docker run --rm \
		-v $(WS):/ws \
		-w /ws \
		$(IMAGE) \
		bash -c "vcs import src < src/.repos"

build:
	docker run --rm \
		-v $(WS):/ws \
		-w /ws \
		$(IMAGE) \
		bash -c "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --packages-ignore multirobot_map_merge"

rviz:
	xhost +local:docker 2>/dev/null || true
	docker run --rm -it \
		--network host \
		--ipc host \
		$(ROS_ENV_PC) \
		-e DISPLAY=$(DISPLAY) \
		-e QT_XCB_NO_XI2=1 \
		-v /tmp/.X11-unix:/tmp/.X11-unix \
		-v $(WS):/ws \
		-w /ws \
		$(IMAGE) \
		bash -c "source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 run rviz2 rviz2 -d /ws/install/picar2_bringup/share/picar2_bringup/config/rviz.rviz"

rqt:
	xhost +local:docker 2>/dev/null || true
	docker run --rm -it \
		--network host \
		--ipc host \
		$(ROS_ENV_PC) \
		-e DISPLAY=$(DISPLAY) \
		-e QT_XCB_NO_XI2=1 \
		-v /tmp/.X11-unix:/tmp/.X11-unix \
		-v $(WS):/ws \
		-w /ws \
		$(IMAGE) \
		bash -c "source /opt/ros/jazzy/setup.bash && source install/setup.bash && rqt"

bringup:
	xhost +local:docker 2>/dev/null || true
	docker run --rm -it \
		--name picar2 \
		--privileged \
		--network host \
		$(ROS_ENV) \
		-e DISPLAY=$(DISPLAY) \
		-v /tmp/.X11-unix:/tmp/.X11-unix \
		-v /dev:/dev \
		-v $(WS):/ws \
		-w /ws \
		$(IMAGE) \
		bash -c "source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch picar2_bringup picar2.launch.py lidar:=$(LIDAR) use_mag:=$(USE_MAG)"

sim:
	xhost +local:docker 2>/dev/null || true
	docker run --rm -it \
		--name picar2 \
		--network host \
		--ipc host \
		$(ROS_ENV) \
		-e DISPLAY=$(DISPLAY) \
		-e QT_XCB_NO_XI2=1 \
		-v /tmp/.X11-unix:/tmp/.X11-unix \
		-v $(WS):/ws \
		-w /ws \
		$(IMAGE) \
		bash -c "source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch picar2_bringup sim.launch.py"

slam:
	docker exec -it picar2 \
		bash -c "source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && ros2 launch picar2_bringup slam.launch.py"

cartographer:
	docker exec -it picar2 \
		bash -c "source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && ros2 launch picar2_bringup cartographer.launch.py"

nav:
	docker exec -it picar2 \
		bash -c "source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && ros2 launch picar2_bringup nav2.launch.py"

explore:
	docker exec -it picar2 \
		bash -c "source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && ros2 launch picar2_bringup explore.launch.py"

teleop:
	docker exec -it picar2 \
		bash -c "source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && ros2 run teleop_twist_keyboard teleop_twist_keyboard"

odom-cal:
	xhost +local:docker 2>/dev/null || true
	docker run --rm -it \
		--network host \
		--ipc host \
		$(ROS_ENV_PC) \
		-e DISPLAY=$(DISPLAY) \
		-e QT_XCB_NO_XI2=1 \
		-v /tmp/.X11-unix:/tmp/.X11-unix \
		-v $(WS):/ws \
		-w /ws \
		$(IMAGE) \
		bash -c "source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 run picar2_bringup odom_cal.py"

imu-verify:
	$(DEBUG_RUN) bash -c "source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && ros2 run picar2_bringup imu_verify.py $(IMU_ARGS)"

# Requires bringup running. Guides through 6-position accel calibration.
# Output saved to src/picar2_bringup/config/imu_calib.yaml (used by apply_calib at bringup).
imu-calib:
	$(DEBUG_RUN) bash -c "source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && ros2 run imu_calib do_calib_node --ros-args -r imu:=/imu/data_raw -p calib_file:=/ws/src/picar2_bringup/config/imu_calib.yaml"

debug:
	$(DEBUG_RUN) bash /ws/scripts/debug.sh $(FOCUS)

diag:
	$(DEBUG_RUN) bash /ws/scripts/diag.sh

shell:
	docker exec -it picar2 \
		bash -c "source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && exec bash"

sync2pi:
	rsync -avz --exclude '.git' --exclude-from='.gitignore' . pi@rpi4.local:~/picar_ws/ros2/

clean:
	rm -rf build install log
