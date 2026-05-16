FROM osrf/ros:jazzy-desktop

COPY src/picar2_control/package.xml     /tmp/src/picar2_control/package.xml
COPY src/picar2_bringup/package.xml     /tmp/src/picar2_bringup/package.xml
COPY src/picar2_description/package.xml /tmp/src/picar2_description/package.xml

RUN apt-get update \
 && rosdep update \
 && rosdep install --from-paths /tmp/src --ignore-src -y \
 && rm -rf /var/lib/apt/lists/*
