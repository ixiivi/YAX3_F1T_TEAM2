#!/usr/bin/env python3
"""
odom yaw 확인 — 휠베이스/조향 게인 캘리브레이션 검증용.

/odom 의 orientation 을 yaw(도)로 바꿔 찍고, 랩(wrap)을 풀어서
'출발 이후 누적 회전량'을 보여준다. 제자리에서 한 바퀴 돌렸을 때
누적값이 ±360° 에 가까우면 캘리브레이션이 맞는 것이다.

  누적 > 360  → odom 이 과대평가 (휠베이스가 실제보다 작거나 게인이 큼)
  누적 < 360  → odom 이 과소평가

사용법:
  python3 yaw_check.py          # 실행 → 차를 한 바퀴 돌린다 → Ctrl+C
  r 키 대신, 재시작하면 기준점이 다시 잡힌다.
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


def quat_to_yaw(q) -> float:
    """쿼터니언 → yaw (rad, -pi~pi)."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class YawCheck(Node):
    def __init__(self):
        super().__init__('yaw_check')
        self.create_subscription(Odometry, '/odom', self._cb, 10)
        self.yaw0 = None      # 시작 yaw (기준점)
        self.prev = None      # 직전 yaw — 랩 감지용
        self.total = 0.0      # 누적 회전량 (rad, 부호 유지)
        self.get_logger().info('/odom 대기 중 — 차를 제자리에서 한 바퀴 돌리세요. (Ctrl+C 종료)')

    def _cb(self, msg: Odometry):
        yaw = quat_to_yaw(msg.pose.pose.orientation)

        if self.yaw0 is None:
            self.yaw0 = yaw
            self.prev = yaw
            return

        # -pi/+pi 경계를 넘을 때 점프를 보정해서 누적한다
        d = yaw - self.prev
        if d > math.pi:
            d -= 2.0 * math.pi
        elif d < -math.pi:
            d += 2.0 * math.pi
        self.total += d
        self.prev = yaw

        p = msg.pose.pose.position
        print(f'\r누적 회전 {math.degrees(self.total):+8.1f}°   '
              f'현재 heading {math.degrees(yaw):+7.1f}°   '
              f'위치 ({p.x:+.2f}, {p.y:+.2f}) m   ',
              end='', flush=True)


def main():
    rclpy.init()
    node = YawCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        turns = node.total / (2.0 * math.pi)
        print(f'\n\n─── 결과 ───')
        print(f'누적 회전량 : {math.degrees(node.total):+.1f}°  ({turns:+.3f} 바퀴)')
        if abs(node.total) > 1e-6:
            ratio = abs(math.degrees(node.total)) / 360.0
            print(f'한 바퀴(360°) 기준 : {ratio:.3f} 배')
            if ratio > 1.05:
                print('→ odom 과대평가. 휠베이스가 실제보다 작거나 조향 게인이 큽니다.')
            elif ratio < 0.95:
                print('→ odom 과소평가. 휠베이스가 실제보다 크거나 조향 게인이 작습니다.')
            else:
                print('→ 캘리브레이션 양호 (±5% 이내).')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
