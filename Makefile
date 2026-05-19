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

.PHONY: all image build rviz bringup teleop shell clean

all: build

image:
	docker build --build-arg BASE_IMAGE=$(BASE_IMAGE) -t $(IMAGE) .

build:
	docker run --rm \
		-v $(WS):/ws \
		-w /ws \
		$(IMAGE) \
		bash -c "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install"

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
		bash -c "source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch picar2_bringup picar2.launch.py"

teleop:
	docker exec -it picar2 \
		bash -c "source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && ros2 run teleop_twist_keyboard teleop_twist_keyboard"

shell:
	docker exec -it picar2 \
		bash -c "source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && exec bash"

clean:
	rm -rf build install log
