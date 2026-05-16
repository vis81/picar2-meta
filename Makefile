IMAGE   := picar2-ros2:jazzy
WS      := $(abspath .)
DISPLAY ?= :0

.PHONY: image build rviz bringup shell clean

image:
	docker build -t $(IMAGE) .

build:
	docker run --rm \
		-v $(WS):/ws \
		-w /ws \
		$(IMAGE) \
		bash -c "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install"

rviz:
	xhost +local:docker 2>/dev/null || true
	docker run --rm -it \
		-e DISPLAY=$(DISPLAY) \
		-v /tmp/.X11-unix:/tmp/.X11-unix \
		-v $(WS):/ws \
		-w /ws \
		$(IMAGE) \
		bash -c "source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch picar2_bringup display.launch.py"

bringup:
	docker run --rm -it \
		--privileged \
		-v /dev:/dev \
		-v $(WS):/ws \
		-w /ws \
		$(IMAGE) \
		bash -c "source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch picar2_bringup picar2.launch.py"

shell:
	docker run --rm -it \
		-v $(WS):/ws \
		-w /ws \
		$(IMAGE) \
		bash

clean:
	rm -rf build install log
