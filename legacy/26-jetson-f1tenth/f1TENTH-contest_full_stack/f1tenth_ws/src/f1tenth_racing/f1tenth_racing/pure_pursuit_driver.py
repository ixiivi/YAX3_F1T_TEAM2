#!/usr/bin/env python3
"""
F1TENTH Pure Pursuit 드라이버
==============================
generate_waypoints.py로 생성된 waypoints.csv를 읽고
Pure Pursuit 알고리즘으로 주행합니다.

토픽:
  구독: /scan (LaserScan), /ego_racecar/odom (Odometry)
  발행: /drive (AckermannDriveStamped)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np
import csv
import os
from tf_transformations import euler_from_quaternion


# ── 파라미터 ─────────────────────────────────────────────────────────────
_THIS_DIR      = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT     = os.path.abspath(os.path.join(_THIS_DIR, '..', '..', '..', '..'))
_MAPS_DIR      = os.path.join(_REPO_ROOT, 'sim_ws', 'src', 'f1tenth_gym_ros', 'maps')
WAYPOINTS_PATH = os.path.join(_MAPS_DIR, 'real_track_waypoints.csv')

LOOKAHEAD_MIN  = 0.8    # 최소 전방주시거리 (m)
LOOKAHEAD_MAX  = 2.5    # 최대 전방주시거리 (m)
SPEED_SCALE    = 1.0    # 웨이포인트 속도에 곱하는 배율
WHEELBASE      = 0.33   # 차량 축간거리 (m)

# 비상 정지 파라미터 (LiDAR)
EMERGENCY_DIST = 0.4    # 전방 이 거리 이내면 정지 (m)
EMERGENCY_ANG  = 30.0   # 전방 ±30도 범위 검사


class PurePursuitDriver(Node):
    def __init__(self):
        super().__init__('pure_pursuit_driver')

        # 웨이포인트 로드
        self.waypoints = self._load_waypoints(WAYPOINTS_PATH)
        self.wp_tree   = None  # 빠른 탐색용 KDTree (첫 odom 수신 시 생성)
        self.get_logger().info(
            f'웨이포인트 {len(self.waypoints)}개 로드: {WAYPOINTS_PATH}')

        # 차량 상태
        self.x      = 0.0
        self.y      = 0.0
        self.yaw    = 0.0
        self.speed  = 0.0

        # LiDAR 비상정지 플래그
        self.emergency = False

        # 구독
        self.create_subscription(
            Odometry, '/ego_racecar/odom',
            self.odom_callback, 10)
        self.create_subscription(
            LaserScan, '/scan',
            self.scan_callback, 10)

        # 발행
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, '/drive', 10)

        self.get_logger().info('Pure Pursuit 드라이버 시작!')

    # ── 웨이포인트 로드 ───────────────────────────────────────────────────
    def _load_waypoints(self, path):
        if not os.path.exists(path):
            self.get_logger().error(f'웨이포인트 파일 없음: {path}')
            self.get_logger().error(
                'generate_waypoints.py를 먼저 실행하세요!')
            return np.zeros((1, 3))

        wps = []
        with open(path, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                wps.append([float(row['x']), float(row['y']), float(row['speed'])])
        return np.array(wps)

    # ── LiDAR 콜백: 비상정지 판단 ────────────────────────────────────────
    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        total  = len(ranges)

        # 전방 ±EMERGENCY_ANG 범위
        center    = total // 2
        half_pts  = int(np.radians(EMERGENCY_ANG) / msg.angle_increment)
        front     = ranges[max(0, center - half_pts):
                           min(total, center + half_pts)]
        front     = front[np.isfinite(front)]

        if len(front) > 0 and front.min() < EMERGENCY_DIST:
            self.emergency = True
        else:
            self.emergency = False

    # ── Odometry 콜백: Pure Pursuit 실행 ─────────────────────────────────
    def odom_callback(self, msg: Odometry):
        # 위치 + 헤딩 업데이트
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q      = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.speed = msg.twist.twist.linear.x

        if self.emergency:
            self._publish(0.0, 0.0)
            self.get_logger().warn('비상정지!')
            return

        if len(self.waypoints) < 2:
            return

        # Pure Pursuit 계산
        steering, target_speed = self._pure_pursuit()
        self._publish(steering, target_speed)

    # ── Pure Pursuit 알고리즘 ─────────────────────────────────────────────
    def _pure_pursuit(self):
        wps    = self.waypoints
        pos    = np.array([self.x, self.y])

        # 가장 가까운 웨이포인트 인덱스
        dists  = np.linalg.norm(wps[:, :2] - pos, axis=1)
        nearest_idx = int(np.argmin(dists))

        # 속도에 비례한 전방주시거리
        lookahead = np.clip(
            LOOKAHEAD_MIN + self.speed * 0.3,
            LOOKAHEAD_MIN, LOOKAHEAD_MAX)

        # 전방주시거리 안에 드는 목표 웨이포인트 탐색
        n = len(wps)
        target_idx = nearest_idx
        for i in range(1, n):
            idx  = (nearest_idx + i) % n
            dist = np.linalg.norm(wps[idx, :2] - pos)
            if dist >= lookahead:
                target_idx = idx
                break

        # 목표점을 차량 좌표계로 변환
        tx, ty = wps[target_idx, :2] - pos
        cos_y  = np.cos(-self.yaw)
        sin_y  = np.sin(-self.yaw)
        lx     =  tx * cos_y - ty * sin_y   # 전방
        ly     =  tx * sin_y + ty * cos_y   # 좌측

        # Pure Pursuit 조향각
        # δ = arctan(2L sinα / ld)
        if abs(lx) < 1e-6 and abs(ly) < 1e-6:
            steering = 0.0
        else:
            ld       = np.hypot(lx, ly)
            steering = np.arctan2(2.0 * WHEELBASE * ly, ld ** 2)
            steering = float(np.clip(steering, -0.4, 0.4))

        # 목표 속도 (웨이포인트 속도 × 배율)
        target_speed = float(wps[target_idx, 2]) * SPEED_SCALE

        # 조향각이 클수록 속도 감소
        abs_steer = abs(steering)
        if abs_steer > 0.3:
            target_speed *= 0.5
        elif abs_steer > 0.2:
            target_speed *= 0.7
        elif abs_steer > 0.1:
            target_speed *= 0.85

        return steering, target_speed

    # ── 드라이브 명령 발행 ────────────────────────────────────────────────
    def _publish(self, steering: float, speed: float):
        msg = AckermannDriveStamped()
        msg.drive.steering_angle = steering
        msg.drive.speed          = speed
        self.drive_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
