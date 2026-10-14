#!/usr/bin/env python3
"""
Waypoint Logger — 레이싱라인 기록.

로컬라이제이션(race 모드 스택 또는 amcl)이 켜진 상태에서 키보드/조종기로
좋은 라인으로 한 바퀴 주행하면, /amcl_pose를 일정 간격마다 CSV(x,y)로 저장.
저장된 CSV는 pure_pursuit가 그대로 읽는다. speed 열은 비워두면
곡률 기반 자동 감속이 적용되고, 직접 3번째 열에 속도를 적어도 된다.

사용: ./race.sh log my_track_wp   (별도 터미널에서 race 스택 실행 중이어야 함)
"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


class WaypointLogger(Node):
    def __init__(self):
        super().__init__('waypoint_logger')
        self.declare_parameter('output', '/tmp/waypoints.csv')
        self.declare_parameter('min_gap', 0.15)   # 웨이포인트 간 최소 거리(m)
        self.out = open(self.get_parameter('output').value, 'w')
        self.last = None
        self.count = 0
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 self.cb, 10)
        self.get_logger().info(
            f"기록 시작 → {self.get_parameter('output').value} (Ctrl+C로 종료)")

    def cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self.last is not None:
            if math.hypot(x - self.last[0], y - self.last[1]) < \
                    self.get_parameter('min_gap').value:
                return
        self.out.write(f'{x:.4f},{y:.4f}\n')
        self.out.flush()
        self.last = (x, y)
        self.count += 1
        if self.count % 20 == 0:
            self.get_logger().info(f'{self.count} pts')


def main():
    rclpy.init()
    node = WaypointLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.out.close()
        print(f'\n총 {node.count}개 웨이포인트 저장 완료')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
