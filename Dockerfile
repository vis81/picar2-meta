ARG BASE_IMAGE=osrf/ros:jazzy-desktop
FROM ${BASE_IMAGE}

COPY src/picar2-ros2/picar2_control/package.xml     /tmp/src/picar2_control/package.xml
COPY src/picar2-ros2/picar2_bringup/package.xml     /tmp/src/picar2_bringup/package.xml
COPY src/picar2-ros2/picar2_description/package.xml /tmp/src/picar2_description/package.xml
COPY src/lds02rr_lidar/package.xml                  /tmp/src/lds02rr_lidar/package.xml
COPY src/vizanti/vizanti/package.xml                /tmp/src/vizanti/package.xml
COPY src/vizanti/vizanti_cpp/package.xml            /tmp/src/vizanti_cpp/package.xml
COPY src/vizanti/vizanti_msgs/package.xml           /tmp/src/vizanti_msgs/package.xml
COPY src/vizanti/vizanti_server/package.xml         /tmp/src/vizanti_server/package.xml
COPY src/vizanti/vizanti_demos/package.xml          /tmp/src/vizanti_demos/package.xml
# explore_lite: picar2_bringup exec_depends on explore_lite_msgs, and the web UI
# subscribes to /explore/status. Without its package.xml here rosdep cannot see
# it as a workspace package and tries to resolve it as a system dependency,
# which fails the whole layer with "Cannot locate rosdep definition".
COPY src/explore_lite/explore/package.xml           /tmp/src/explore/package.xml
COPY src/explore_lite/explore_lite_msgs/package.xml /tmp/src/explore_lite_msgs/package.xml
COPY src/explore_lite/map_merge/package.xml         /tmp/src/map_merge/package.xml

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
        ros-jazzy-joy \
        ros-jazzy-teleop-twist-joy \
        python3-evdev \
 # point cloud processing (pcl_ros filter nodes for sen0628)
 && apt-get install -y \
        ros-jazzy-pcl-ros \
 # Foxglove WebSocket bridge — Foxglove Studio visualization (Pi + PC)
 && apt-get install -y \
        ros-jazzy-foxglove-bridge \
 # Vizanti runtime — Flask UI + WSGI server + scipy for vizanti_demos
 # (also pulled via rosdep from vizanti_server/package.xml; explicit for safety)
 && apt-get install -y \
        ros-jazzy-rosbridge-suite \
        python3-flask \
        python3-waitress \
        python3-scipy \
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

# Process supervisor. Its own layer, and last, so adding it does not invalidate
# the ROS install above — that layer takes tens of minutes to rebuild under
# QEMU, this one takes seconds.
#
# supervisord is PID 1 in the running container and owns every process: the
# bringup stack, the web UI, and the navigation layers the UI starts on demand.
# Before this they were children of the web UI, so restarting the UI to pick up
# a code change tore down AMCL and Nav2 with it and cost a re-localization
# every time.
RUN apt-get update \
 && apt-get install -y supervisor \
 && rm -rf /var/lib/apt/lists/*
