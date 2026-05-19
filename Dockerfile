ARG BASE_IMAGE=osrf/ros:jazzy-desktop
FROM ${BASE_IMAGE}

COPY src/picar2_control/package.xml     /tmp/src/picar2_control/package.xml
COPY src/picar2_bringup/package.xml     /tmp/src/picar2_bringup/package.xml
COPY src/picar2_description/package.xml /tmp/src/picar2_description/package.xml
COPY src/picar2_lidar/package.xml       /tmp/src/picar2_lidar/package.xml

RUN apt-get update \
 && rosdep update \
 && rosdep install --from-paths /tmp/src --ignore-src -y \
 && apt-get install -y python3-serial \
                       ros-jazzy-joint-state-publisher-gui \
                       ros-jazzy-ackermann-steering-controller \
                       ros-jazzy-teleop-twist-keyboard \
                       ros-jazzy-rmw-cyclonedds-cpp \
                       ros-jazzy-imu-filter-madgwick \
                       ros-jazzy-rviz-imu-plugin \
                       ros-jazzy-robot-localization \
                       ros-jazzy-slam-toolbox \
                       ros-jazzy-nav2-lifecycle-manager \
 && arch=$(dpkg --print-architecture) \
 && if [ "$arch" = "arm64" ] || [ "$arch" = "armhf" ]; then \
        apt-get install -y python3-rpi.gpio; \
    fi \
 && rm -rf /var/lib/apt/lists/*
