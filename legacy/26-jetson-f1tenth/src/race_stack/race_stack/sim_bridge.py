#!/usr/bin/env python3
"""
sim_bridge — f1tenth_gym_ros ↔ race_stack 어댑터.

시뮬레이터 ground-truth 위치(/ego_racecar/odom, frame_id='map')를
컨트롤러가 기대하는 /amcl_pose + /odom 으로 변환한다.

start_gate=False (기본): /race/enabled=True 즉시 발행 (mission_manager 없이 출발)
start_gate=True        : /race/enabled 를 발행하지 않고 mission_manager 에 위임.
                         대신 /race/localized=0.0 을 래치로 발행해
                         mission_manager 가 "위치 정렬 완료"로 인식하게 한다.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Bool, Float32


class SimBridge(Node):
    def __init__(self):
        super().__init__('sim_bridge')

        self.declare_parameter('start_gate', False)
        start_gate = bool(self.get_parameter('start_gate').value)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.amcl_pub    = self.create_publisher(PoseWithCovarianceStamped, '/amcl_pose',      10)
        self.odom_pub    = self.create_publisher(Odometry,                  '/odom',            10)
        self.enabled_pub = self.create_publisher(Bool,                      '/race/enabled', latched)

        self.create_subscription(Odometry, '/ego_racecar/odom', self._odom_cb, 10)

        if start_gate:
            # mission_manager 가 /race/enabled 를 관리한다.
            # 단, 시뮬에는 auto_localize 가 없으므로 /race/localized=0.0 을 대신 발행해
            # mission_manager 의 "위치 정렬 완료" 조건을 충족시킨다.
            localized_pub = self.create_publisher(Float32, '/race/localized', latched)
            localized_pub.publish(Float32(data=0.0))
            self.get_logger().info(
                'SimBridge: start_gate=True → mission_manager 에 위임 '
                '(/race/localized=0.0 발행)')
        else:
            # latched — 나중에 구독하는 컨트롤러도 True 를 받는다
            self.enabled_pub.publish(Bool(data=True))
            self.get_logger().info('SimBridge ready: /race/enabled=True 즉시 발행')

    def _odom_cb(self, msg: Odometry):
        self.odom_pub.publish(msg)

        amcl = PoseWithCovarianceStamped()
        amcl.header = msg.header   # frame_id='map' 그대로
        amcl.pose.pose = msg.pose.pose
        self.amcl_pub.publish(amcl)


def main():
    rclpy.init()
    node = SimBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
