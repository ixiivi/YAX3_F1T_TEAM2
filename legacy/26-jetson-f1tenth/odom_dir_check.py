#!/usr/bin/env python3
"""odom_dir_check.py — /odom_imu 의 병진 속도 **부호**가 맞는지 눈으로 확인한다.

왜 필요한가
───────────
이 값이 뒤집혀 있으면 SLAM 이 전진을 후진으로 받아 **맵이 통째로 깨진다.**
그런데 조이스틱 주행은 멀쩡해 보인다 — 명령 경로와 오도메트리 경로의 부호
처리가 비대칭이기 때문이다:

    ackermann_to_vesc :  erpm = gain × v          (음수 없음)
    vesc_to_odom      :  v    = (−erpm) / gain    (음수 있음)

2호차는 모터 결선이 반대라 gain 이 음수인데, 그러면 명령은 맞고 오도메트리만
한 번 더 뒤집힌다. 그래서 "주행은 되는데 맵만 안 나오는" 상태가 된다.

사용법
──────
스택이 떠 있어야 한다 (`./race.sh mapping-joy` 또는 `joy`).

    python3 ~/yax/26-jetson-f1tenth/odom_dir_check.py

**차를 손으로 앞으로 밀면서** 화면을 본다. Ctrl+C 로 종료.

판정
────
    전진 →  양수   ✅ 그대로 진행
    전진 →  음수   ❌ config/vehicle_car2.yaml 의 localization.speed_sign 을
                      -1.0 으로 바꾸고 스택 재시작

  ※ 아주 천천히 밀면 VESC 가 센서리스라 erpm 을 놓쳐 0 만 나온다.
    걷는 속도 정도로 밀 것.
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from nav_msgs.msg import Odometry


class OdomDirCheck(Node):
    def __init__(self):
        super().__init__('odom_dir_check')
        self.create_subscription(Odometry, '/odom_imu', self._cb, 20)
        self.peak = 0.0
        self.n = 0
        print('\n차를 손으로 **앞으로** 미세요.   Ctrl+C 로 종료\n')
        print('  전진할 때 [전진] 이 뜨면 정상, [후진] 이 뜨면 부호가 반대입니다.\n')

    def _cb(self, msg: Odometry):
        v = msg.twist.twist.linear.x
        self.n += 1
        if abs(v) > abs(self.peak):
            self.peak = v
        if abs(v) < 0.05:
            tag = '  (정지)'
        elif v > 0:
            tag = '  [전진]  ← 부호 정상'
        else:
            tag = '  [후진]  ← 부호 반대'
        print(f'\r  속도 {v:+7.3f} m/s{tag:28s}  최대 {self.peak:+.3f}   ',
              end='', flush=True)


def main():
    rclpy.init()
    node = OdomDirCheck()
    try:
        rclpy.spin(node)
    # ★ Humble 의 rclpy 는 SIGINT 를 자체 처리해 ExternalShutdownException 을
    #   던진다. KeyboardInterrupt 만 잡으면 Ctrl+C 에 트레이스백이 쏟아지고
    #   정작 필요한 판정 문구가 안 보인다.
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        print('\n')
        if node.n == 0:
            print('  ✗ /odom_imu 를 한 건도 못 받았습니다.')
            print('    - 스택이 떠 있나요?  ./race.sh mapping-joy')
            print('    - imu_odom 노드가 살아 있나요? (기동 로그의 ImuOdom 줄)')
        elif abs(node.peak) < 0.05:
            print('  ✗ 움직임이 감지되지 않았습니다. 좀 더 빠르게(걷는 속도) 미세요.')
            print('    VESC 는 센서리스라 아주 느리면 erpm 을 놓칩니다.')
        elif node.peak > 0:
            print(f'  ✅ 전진 = 양수 (최대 {node.peak:+.3f} m/s). 부호 정상 — 그대로 진행하세요.')
        else:
            print(f'  ❌ 전진 = 음수 (최대 {node.peak:+.3f} m/s). 부호가 반대입니다.')
            print()
            print('     config/vehicle_car2.yaml 에서 아래를 고치고 스택을 재시작하세요:')
            print('         localization:')
            print('           speed_sign: -1.0')
            print('     재기동 로그에 "속도부호=-1" 이 뜨면 반영된 것입니다.')
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
