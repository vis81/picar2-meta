ARG BASE_IMAGE=osrf/ros:jazzy-desktop
FROM ${BASE_IMAGE}

COPY src/picar2-ros2/picar2_control/package.xml     /tmp/src/picar2_control/package.xml
COPY src/picar2-ros2/picar2_bringup/package.xml     /tmp/src/picar2_bringup/package.xml
COPY src/picar2-ros2/picar2_description/package.xml /tmp/src/picar2_description/package.xml
COPY src/lds02rr_lidar/package.xml                  /tmp/src/lds02rr_lidar/package.xml

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
 # magnetometer calibration pipeline
 && apt-get install -y \
        ros-jazzy-magnetometer-pipeline \
 # teleop (used on both Pi and PC)
 && apt-get install -y \
        ros-jazzy-teleop-twist-keyboard \
 # point cloud processing (pcl_ros filter nodes for sen0628)
 && apt-get install -y \
        ros-jazzy-pcl-ros \
 # SEN0628 ToF sensor I2C support + Pi-only GPIO library
 && apt-get install -y python3-smbus \
 && arch=$(dpkg --print-architecture) \
 && if [ "$arch" = "arm64" ] || [ "$arch" = "armhf" ]; then \
        apt-get install -y python3-rpi.gpio; \
    fi \
 # PC-only: Gazebo simulation + GUI tools (pyside2/Qt crash under QEMU cross-build)
 && if [ "$arch" != "arm64" ] && [ "$arch" != "armhf" ]; then \
        apt-get install -y \
            ros-jazzy-ros-gz-sim \
            ros-jazzy-ros-gz-bridge \
            ros-jazzy-gz-ros2-control \
            ros-jazzy-joint-state-publisher-gui \
            ros-jazzy-rviz-imu-plugin; \
    fi \
 && rm -rf /var/lib/apt/lists/*
