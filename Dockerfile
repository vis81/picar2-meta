FROM osrf/ros:jazzy-desktop

COPY src/picar2_control/package.xml     /tmp/src/picar2_control/package.xml
COPY src/picar2_bringup/package.xml     /tmp/src/picar2_bringup/package.xml
COPY src/picar2_description/package.xml /tmp/src/picar2_description/package.xml

RUN apt-get update \
 && rosdep update \
 && rosdep install --from-paths /tmp/src --ignore-src -y \
 && apt-get install -y ros-jazzy-joint-state-publisher-gui \
                       ros-jazzy-ackermann-steering-controller \
                       ros-jazzy-teleop-twist-keyboard \
                       ros-jazzy-rmw-cyclonedds-cpp \
 && rm -rf /var/lib/apt/lists/*
