#!/usr/bin/env python3
"""Republish /ld07/scan as a PointCloud2 in the map frame for Z-height debugging.

Run:  ros2 run picar2_bringup scan_to_cloud.py
  or: python3 scripts/scan_to_cloud.py

Then in RViz add a PointCloud2 display on /ld07/cloud and color by z-axis.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
import laser_geometry.laser_geometry as lg
import tf2_ros
import tf2_sensor_msgs.tf2_sensor_msgs  # noqa: registers PointCloud2 transform support


class ScanToCloud(Node):
    def __init__(self):
        super().__init__('scan_to_cloud')
        self.lp = lg.LaserProjection()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(PointCloud2, '/ld07/cloud', 10)
        self.create_subscription(LaserScan, '/ld07/scan', self._cb, 10)
        self.get_logger().info('Publishing /ld07/cloud — add PointCloud2 in RViz, color by Z')

    def _cb(self, msg: LaserScan):
        # Project to cloud in sensor frame
        cloud = self.lp.projectLaser(msg)
        # Transform into map frame so Z is relative to the floor
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', cloud.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1))
            cloud_map = tf2_sensor_msgs.tf2_sensor_msgs.do_transform_cloud(cloud, tf)
            self.pub.publish(cloud_map)
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}', throttle_duration_sec=2.0)


def main():
    rclpy.init()
    rclpy.spin(ScanToCloud())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
