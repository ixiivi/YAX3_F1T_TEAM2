#!/usr/bin/env python3
"""Find where the car is on the saved map by matching one LiDAR scan against the whole map,
then hand the pose to AMCL on /initialpose. Runs once and exits.

On the Jetson the search can take half a minute. If the car drives meanwhile (2026-11-11: the
result arrived 23 s into a run and threw AMCL 10 m back), the pose is moved on by the odometry
from the scan's time to now before it is sent; after more than max_moved (1 m) it is not sent.
  ros2 run waypoint_nav global_localize --ros-args -p map:=/path/map.yaml
"""
import math
import threading
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

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
                                  self.declare_parameter('laser_x', 0.267).value,
                                  device=self.declare_parameter('match_device', 'cpu').value)
        self.get_logger().info(f'search on {self.matcher.device_note}')
        self.min_score = self.declare_parameter('min_score', 0.6).value
        self.scan = None
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.tf = Buffer(cache_time=rclpy.duration.Duration(seconds=120.0))
        self.tf_listener = TransformListener(self.tf, self)

    def on_scan(self, s):
        self.scan = s

    def odom_pose(self, stamp=None):
        """base_link in the odom frame, at stamp or latest; None without odometry."""
        try:
            t = self.tf.lookup_transform('odom', 'base_link', stamp or Time())
        except TransformException:
            return None
        q = t.transform.rotation
        return (t.transform.translation.x, t.transform.translation.y,
                math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))


def carry_forward(pose, then, now):
    """Move a map pose found for odom pose `then` on by the odometry motion to `now`."""
    dx, dy = now[0] - then[0], now[1] - then[1]
    c, s = math.cos(-then[2]), math.sin(-then[2])
    bx, by = c * dx - s * dy, s * dx + c * dy           # motion in the car frame at scan time
    dyaw = math.atan2(math.sin(now[2] - then[2]), math.cos(now[2] - then[2]))
    x, y, yaw = pose
    return (x + math.cos(yaw) * bx - math.sin(yaw) * by,
            y + math.sin(yaw) * bx + math.cos(yaw) * by, yaw + dyaw)


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
        scan = node.scan
        t_search = time.time()
        result = []
        worker = threading.Thread(
            target=lambda: result.append(node.matcher.global_search(*scan_rays(scan, step=6))))
        worker.start()
        while worker.is_alive():                    # keep the TF buffer filling meanwhile
            rclpy.spin_once(node, timeout_sec=0.1)
        sc, x, y, yaw = result[0]
        then, now = node.odom_pose(Time.from_msg(scan.header.stamp)), node.odom_pose()
        moved = 0.0
        if then is not None and now is not None:
            moved = math.hypot(now[0] - then[0], now[1] - then[1])
            x, y, yaw = carry_forward((x, y, yaw), then, now)
            if moved > 0.05:
                log.info(f'the car moved {moved:.2f} m during the search: pose carried forward')
        else:
            log.warn('no odometry at the scan time: the car must not have moved since')
        max_moved = node.declare_parameter('max_moved', 1.0).value
        if moved > max_moved:
            # driven far (wheel slip, seconds of dead reckoning): the carried pose is a guess,
            # and AMCL has been tracking the car meanwhile. Do not throw it away for this
            log.warn(f'the car drove {moved:.1f} m during the {time.time() - t_search:.0f} s search; '
                     'not sending the old result. Search again with the car standing still.')
        elif sc < node.min_score:
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
