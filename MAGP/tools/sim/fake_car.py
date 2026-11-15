#!/usr/bin/env python3
"""Fake car: kinematic bicycle, noisy raycast scan from a saved map, TF, /initialpose.

usage: fake_car.py <map.yaml> <x> <y> <yaw rad> <seconds>
Refuses to run unless ROS_DOMAIN_ID=87 and ROS_LOCALHOST_ONLY=1.
"""
import math
import os
import sys
import time

if os.environ.get('ROS_DOMAIN_ID') != '87' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
    sys.exit('REFUSED: not isolated from the real car (need ROS_DOMAIN_ID=87, ROS_LOCALHOST_ONLY=1)')

import numpy as np  # noqa: E402
import rclpy  # noqa: E402
import yaml  # noqa: E402
from ackermann_msgs.msg import AckermannDriveStamped  # noqa: E402
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped  # noqa: E402
from PIL import Image  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402
from tf2_ros import TransformBroadcaster  # noqa: E402

LASER_X, WB, FRONT = 0.267, 0.33, 0.42


class Car(Node):
    def __init__(self, map_yaml, x, y, yaw):
        super().__init__('fake_car')
        m = yaml.safe_load(open(map_yaml))
        img = np.array(Image.open(os.path.join(os.path.dirname(map_yaml), m['image'])).convert('L'))
        self.res, (self.ox, self.oy) = m['resolution'], m['origin'][:2]
        self.h, self.w = img.shape
        self.occ = img < 50
        if os.environ.get('OBST'):      # 'x,y,r' a round obstacle only the fake world has
            ox_, oy_, rr = (float(v) for v in os.environ['OBST'].split(','))
            yy, xx = np.mgrid[0:self.h, 0:self.w]
            wx, wy = xx * self.res + self.ox, (self.h - 1 - yy) * self.res + self.oy
            self.occ = self.occ | (np.hypot(wx - ox_, wy - oy_) < rr)
        self.x, self.y, self.yaw = x, y, yaw
        self.map_odom = (0.0, 0.0, 0.0)
        self.v = self.st = self.cmd_v = self.cmd_st = 0.0
        self.a = -2.356 + np.arange(1081) * math.radians(0.25)
        self.r = np.arange(0.05, 8.0, 0.02)
        self.rng = np.random.default_rng(1)
        self.tfb = TransformBroadcaster(self)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.create_subscription(AckermannDriveStamped, '/drive', self.on_drive, 10)
        if not os.environ.get('NO_MAP_TF'):
            self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.on_pose, 10)
        self.create_timer(0.02, self.tick)
        self.create_timer(0.05, self.scan)

    def occupied(self, px, py):
        u = int((px - self.ox) / self.res)
        v = self.h - 1 - int((py - self.oy) / self.res)
        return 0 <= u < self.w and 0 <= v < self.h and bool(self.occ[v, u])

    def on_drive(self, m):
        self.cmd_v, self.cmd_st = m.drive.speed, m.drive.steering_angle

    def on_pose(self, m):
        p = m.pose.pose
        yaw = math.atan2(2 * p.orientation.w * p.orientation.z, 1 - 2 * p.orientation.z ** 2)
        dy = yaw - self.yaw
        self.map_odom = (p.position.x - (math.cos(dy) * self.x - math.sin(dy) * self.y),
                         p.position.y - (math.sin(dy) * self.x + math.cos(dy) * self.y), dy)

    def tick(self):
        dt = 0.02
        self.v += max(-4 * dt, min(4 * dt, self.cmd_v - self.v))
        self.st += max(-3 * dt, min(3 * dt, self.cmd_st - self.st))
        ds = self.v * dt
        nx, ny = self.x + ds * math.cos(self.yaw), self.y + ds * math.sin(self.yaw)
        bx, by = nx + (FRONT + 0.01) * math.cos(self.yaw), ny + (FRONT + 0.01) * math.sin(self.yaw)
        if ds > 0 and self.occupied(bx, by):
            self.v = 0.0
        else:
            self.x, self.y = nx, ny
            self.yaw += ds * math.tan(self.st) / WB
        now = self.get_clock().now().to_msg()
        if not os.environ.get('NO_MAP_TF'):   # AMCL publishes map->odom itself
            self.send(now, 'map', 'odom', *self.map_odom)
        self.send(now, 'odom', 'base_link', self.x, self.y, self.yaw)

    def send(self, stamp, parent, child, x, y, yaw):
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, parent, child
        t.transform.translation.x, t.transform.translation.y = float(x), float(y)
        t.transform.rotation.z, t.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.tfb.sendTransform(t)

    def scan(self):
        lx = self.x + LASER_X * math.cos(self.yaw)
        ly = self.y + LASER_X * math.sin(self.yaw)
        ang = self.yaw + self.a
        px = lx + self.r[None, :] * np.cos(ang)[:, None]
        py = ly + self.r[None, :] * np.sin(ang)[:, None]
        u = ((px - self.ox) / self.res).astype(int)
        v = self.h - 1 - ((py - self.oy) / self.res).astype(int)
        ins = (u >= 0) & (u < self.w) & (v >= 0) & (v < self.h)
        hit = np.zeros(px.shape, bool)
        hit[ins] = self.occ[v[ins], u[ins]]
        rng = np.where(hit.any(1), self.r[hit.argmax(1)], np.inf)
        rng = rng + self.rng.normal(0.0, 0.015, rng.shape)
        m = LaserScan()
        m.header.stamp, m.header.frame_id = self.get_clock().now().to_msg(), 'laser'
        m.angle_min, m.angle_increment = -2.356, math.radians(0.25)
        m.angle_max = float(self.a[-1])
        m.range_min, m.range_max = 0.06, 10.0
        m.ranges = [float(x) for x in rng]
        self.scan_pub.publish(m)


def main():
    rclpy.init()
    node = Car(sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4]))
    end = time.monotonic() + float(sys.argv[5])
    while rclpy.ok() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.01)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
