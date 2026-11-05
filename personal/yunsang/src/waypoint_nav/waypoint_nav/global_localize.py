#!/usr/bin/env python3
"""Find where the car is on the saved map by matching one LiDAR scan against the whole map,
then hand the pose to AMCL on /initialpose. Runs once and exits.

The car must stand still while it runs (a few seconds).
  ros2 run waypoint_nav global_localize --ros-args -p map:=/path/map.yaml
"""
import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from waypoint_nav.scan_match import MapMatcher, scan_rays


def initial_pose_msg(node, x, y, yaw):
    m = PoseWithCovarianceStamped()
    m.header.frame_id = 'map'
    m.header.stamp = node.get_clock().now().to_msg()
    m.pose.pose.position.x, m.pose.pose.position.y = x, y
    m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    m.pose.covariance[0] = m.pose.covariance[7] = 0.04
    m.pose.covariance[35] = 0.02
    return m


class GlobalLocalize(Node):
    def __init__(self):
        super().__init__('global_localize')
        self.matcher = MapMatcher(self.declare_parameter('map', '').value,
                                  self.declare_parameter('laser_x', 0.267).value)
        self.min_score = self.declare_parameter('min_score', 0.6).value
        self.scan = None
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

    def on_scan(self, s):
        self.scan = s


def main():
    rclpy.init()
    node = GlobalLocalize()
    log = node.get_logger()
    deadline = time.time() + 30.0
    # wait for a scan and for AMCL to listen on /initialpose
    while rclpy.ok() and time.time() < deadline and (
            node.scan is None or node.pub.get_subscription_count() == 0):
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.scan is None or node.pub.get_subscription_count() == 0:
        log.error('no /scan or no AMCL listening on /initialpose, giving up')
    else:
        sc, x, y, yaw = node.matcher.global_search(*scan_rays(node.scan, step=6))
        if sc < node.min_score:
            log.warn(f'best fit only {sc:.2f} at ({x:.2f}, {y:.2f}); not sending it. '
                     'Is the car inside the mapped area? Use RViz "2D Pose Estimate".')
        else:
            for _ in range(3):
                node.pub.publish(initial_pose_msg(node, x, y, yaw))
                rclpy.spin_once(node, timeout_sec=0.2)
            log.info(f'car at ({x:.2f}, {y:.2f}) yaw {math.degrees(yaw):.0f} deg, '
                     f'fit {sc:.2f} -> sent to AMCL')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
