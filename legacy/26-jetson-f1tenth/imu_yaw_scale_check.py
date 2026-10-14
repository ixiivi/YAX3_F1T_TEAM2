#!/usr/bin/env python3
"""
imu_yaw_scale_check.py — imu_odom 이 적분한 yaw 가 실제 회전량과 맞는지 잰다.

축·부호는 imu_axis_check.py 로 확인했다. 남은 건 **크기(스케일)** 다.
자이로 감도 오차나 단위 변환 실수가 있으면 "방향은 맞는데 1.1배" 가 되는데,
이건 지금 서보 명령 기반 odom 이 겪는 문제와 정확히 같은 종류의 오차다.
360° 를 돌려서 적분값이 360° 로 나오는지 보면 배율이 바로 드러난다.

사전 조건: imu_odom 이 떠 있어야 한다 (`./race.sh box` 면 스택째로 뜬다).
          /odom_imu 를 구독한다.

    python3 ~/yax/imu_yaw_scale_check.py

차를 바닥에 두고 **제자리에서 한 바퀴(360°)** 돌린 뒤 Enter. 여러 바퀴도 된다
(--turns 로 알려주면 회당 오차까지 계산한다). 바닥의 타일 선 같은 기준을 잡고
출발 자세로 정확히 되돌아오는 게 정확도의 전부다 — 눈대중으로 어긋난 만큼이
그대로 측정 오차가 된다.

판정:
  오차 < 2%  → 그대로 써도 된다
  2~5%       → 쓸 수는 있지만 긴 주행에서 누적된다. 재측정 후 스케일 보정 검토
  > 5%       → 단위·축 매핑을 의심할 것 (deg/s 를 rad/s 로 착각하면 57배가 난다)
"""
import argparse
import math
import sys
import threading
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class YawWatch(Node):
    """토픽의 yaw 는 ±π 로 감겨 나온다. 누적 회전을 보려면 풀어서(unwrap) 더해야 한다."""

    def __init__(self):
        super().__init__('imu_yaw_scale_check')
        self.total = 0.0          # 누적 회전 (rad, 부호 있음)
        self.prev = None
        self.count = 0
        self.lock = threading.Lock()
        self.create_subscription(Odometry, '/odom_imu', self.cb, 20)

    def cb(self, msg):
        y = yaw_from_quat(msg.pose.pose.orientation)
        with self.lock:
            if self.prev is not None:
                d = y - self.prev
                # ±π 경계를 넘을 때의 점프를 되돌린다
                d = math.atan2(math.sin(d), math.cos(d))
                self.total += d
            self.prev = y
            self.count += 1

    def reset(self):
        with self.lock:
            self.total = 0.0
            self.prev = None
            self.count = 0

    def read(self):
        with self.lock:
            return self.total, self.count


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--turns', type=float, default=1.0,
                    help='실제로 돌릴 바퀴 수 (기본 1.0 = 360°)')
    args = ap.parse_args()

    rclpy.init()
    node = YawWatch()
    spin = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    spin.start()

    print(__doc__.split('사전 조건')[0])
    print('=' * 62)
    print('/odom_imu 수신 대기 중…')
    t0 = time.time()
    while time.time() - t0 < 5.0:
        if node.read()[1] > 5:
            break
        time.sleep(0.2)
    else:
        rclpy.shutdown()
        sys.exit('/odom_imu 에 데이터가 없습니다.\n'
                 'imu_odom 이 떠 있는지 확인하세요 (./race.sh box 로 스택째 기동).')
    print('수신 OK.\n')

    expect = args.turns * 360.0
    print(f'차를 바닥에 두고 **제자리에서 {args.turns:g}바퀴({expect:.0f}°)** 돌릴 겁니다.')
    print('출발 자세를 기억해 두세요 — 정확히 그 자세로 돌아와야 합니다.')
    print('반시계(왼쪽)로 돌리면 +, 시계(오른쪽)면 - 로 나옵니다. 어느 쪽이든 됩니다.')
    input('\n준비되면 Enter → 그때부터 측정 시작. ')
    node.reset()
    time.sleep(0.2)
    input('다 돌려서 출발 자세로 돌아왔으면 Enter. ')

    total, n = node.read()
    deg = math.degrees(total)
    if n < 20:
        rclpy.shutdown()
        sys.exit(f'샘플이 {n}개뿐입니다 — 측정이 제대로 안 됐습니다.')

    measured = abs(deg)
    err = measured - expect
    ratio = measured / expect if expect else float('nan')

    print('\n' + '=' * 62)
    print(f'적분된 회전량 : {deg:+.2f}°   ({n}샘플)')
    print(f'실제 회전량   : {expect:.0f}° ({args.turns:g}바퀴)')
    print(f'오차          : {err:+.2f}°   ({(ratio - 1) * 100:+.2f}%)')
    print(f'스케일 비     : {ratio:.4f}   (1.0 이 정답)')
    print(f'바퀴당 오차   : {err / args.turns:+.2f}°')
    print()

    e = abs(ratio - 1.0)
    if e < 0.02:
        print('✅ 오차 2% 미만 — 그대로 쓰면 된다. 서보 명령 기반보다 확실히 낫다.')
    elif e < 0.05:
        print('△ 오차 2~5% — 쓸 수는 있으나 긴 주행에서 누적된다.')
        print('  손으로 돌린 정확도 자체가 이 정도 오차를 만든다. 3바퀴(--turns 3)로')
        print('  다시 재보고, 그래도 같은 방향으로 치우치면 스케일 보정을 검토할 것.')
    else:
        print('⚠ 오차 5% 초과 — 단위·축 매핑을 의심할 것.')
        print('  deg/s 를 rad/s 로 착각하면 57배, 축을 잘못 잡으면 엉뚱한 값이 된다.')
        print(f'  imu_gyro_units / imu_yaw_axis / imu_yaw_sign 을 다시 확인하세요.')

    print('\n※ 손으로 돌린 각도가 곧 기준이다. 출발 자세로 정확히 못 돌아왔으면')
    print('  그 어긋남이 그대로 오차에 들어간다 — 바닥 선을 기준 삼아 다시 재는 게 낫다.')

    rclpy.shutdown()
    spin.join(timeout=2.0)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\n중단')
