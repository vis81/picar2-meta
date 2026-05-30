ARG BASE_IMAGE=osrf/ros:jazzy-desktop
FROM ${BASE_IMAGE}

COPY src/picar2_control/package.xml     /tmp/src/picar2_control/package.xml
COPY src/picar2_bringup/package.xml     /tmp/src/picar2_bringup/package.xml
COPY src/picar2_description/package.xml /tmp/src/picar2_description/package.xml
COPY src/picar2_lidar/package.xml       /tmp/src/picar2_lidar/package.xml

RUN apt-get update \
 && rosdep update \
 # workspace package dependencies (declared in package.xml files)
 && rosdep install --from-paths /tmp/src --ignore-src -y \
 # middleware (not in any package.xml)
 && apt-get install -y \
        ros-jazzy-rmw-cyclonedds-cpp \
 # SLAM (not in any package.xml)
 && apt-get install -y \
        ros-jazzy-cartographer-ros \
 # navigation (not in any package.xml)
 && apt-get install -y \
        ros-jazzy-navigation2 \
 # Gazebo Harmonic simulation (not in any package.xml)
 && apt-get install -y \
        ros-jazzy-ros-gz-sim \
        ros-jazzy-ros-gz-bridge \
        ros-jazzy-gz-ros2-control \
 # magnetometer calibration pipeline
 && apt-get install -y \
        ros-jazzy-magnetometer-pipeline \
 # desktop tools (not in any package.xml)
 && apt-get install -y \
        ros-jazzy-joint-state-publisher-gui \
        ros-jazzy-rviz-imu-plugin \
        ros-jazzy-teleop-twist-keyboard \
 # Pi-only GPIO library
 && arch=$(dpkg --print-architecture) \
 && if [ "$arch" = "arm64" ] || [ "$arch" = "armhf" ]; then \
        apt-get install -y python3-rpi.gpio; \
    fi \
 && rm -rf /var/lib/apt/lists/*
