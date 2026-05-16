IMAGE := picar2-ros2:jazzy
WS    := $(abspath .)

.PHONY: image build shell clean

image:
	docker build -t $(IMAGE) .

build:
	docker run --rm \
		-v $(WS):/ws \
		-w /ws \
		$(IMAGE) \
		bash -c "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install"

shell:
	docker run --rm -it \
		-v $(WS):/ws \
		-w /ws \
		$(IMAGE) \
		bash

clean:
	rm -rf build install log
