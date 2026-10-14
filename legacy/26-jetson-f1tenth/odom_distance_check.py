#!/usr/bin/env python3
"""
odom_distance_check.py — 휠 오도메트리가 말하는 이동거리를 재고,
                          실측 거리와 비교해 speed_to_erpm_gain 을 보정한다.

왜 필요한가
───────────
yaw 는 IMU 로 해결했지만(imu_odom), **거리는 여전히 바퀴 회전수(VESC ERPM)** 에서
나온다. `config/vesc.yaml` 의

    speed = (erpm - speed_to_erpm_offset) / speed_to_erpm_gain

이 gain 은 바퀴 둘레·기어비·극수 가정에서 나온 값이라, 타이어가 닳거나 하중으로
눌려 유효 반경이 달라지면 거리가 **일정 비율로** 틀린다 (RC 타이어면 보통 2~5%).

★ 이 오차는 AMCL 이 가장 못 잡아주는 성분이다. LiDAR 스캔 매칭은 횡방향·각도는
  잘 관측하지만, 평행한 벽 사이에서 앞뒤로 미끄러지는 건 거의 관측하지 못한다
  (aperture problem). 긴 직선에서 계속 누적되다 코너에 가서야 교정된다.

★ IMU 가속도로 거리를 얻으려 하지 말 것. 두 번 적분하면 바이어스가 t² 로 커져
  자이로와는 비교가 안 되게 빨리 발산한다. 지상 로봇에서는 안 하는 일이다.
  스케일 오차는 필터로 못 없앤다 — 줄자 같은 **절대 기준**이 있어야 잡힌다.

사용법
──────
스택이 떠 있어야 한다 (`./race.sh mapping-joy` 또는 `joy` 로 수동 주행 가능한 상태).

    # 1) 지금 측정만 해두기 (줄자 없을 때)
    python3 ~/yax/odom_distance_check.py

    # 2) 실측 거리를 알면 보정 게인까지 계산
    python3 ~/yax/odom_distance_check.py --actual 10.0

Enter 로 시작 → 조이스틱으로 **직진** → 도착선에서 멈추고 Enter.

측정 요령 (오차를 줄이는 순서대로)
  · 길수록 정확하다. 10m 이상 권장 — 출발/도착 표시 오차가 그만큼 희석된다
  · **등속 중속**으로. 급가속은 바퀴가 헛돌아(슬립) 거리를 과대추정한다
  · 되도록 직진. 코너를 돌면 안쪽/바깥 바퀴 차이가 섞인다
  · 2~3회 반복해 평균낼 것. 매번 크게 다르면 슬립이 심한 것이다
"""
import argparse
import math
import os
import sys
import threading
import time

import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node

REPO_DIR = os.path.dirname(os.path.abspath(__file__))
VESC_YAML = os.path.join(REPO_DIR, 'config', 'vesc.yaml')


def current_gain():
    """config/vesc.yaml 의 speed_to_erpm_gain (못 읽으면 None)"""
    try:
        cfg = yaml.safe_load(open(VESC_YAML)) or {}
    except OSError:
        return None
    for key in ('/**', 'vesc_to_odom_node'):
        p = (cfg.get(key) or {}).get('ros__parameters') or {}
        if 'speed_to_erpm_gain' in p:
            return float(p['speed_to_erpm_gain'])
    return None


class DistWatch(Node):
    """세 가지를 동시에 잰다 — 셋을 비교하면 뭐가 문제인지 드러난다.

      1) ∫|v|dt   : twist.linear.x 적분. speed_to_erpm_gain 이 직접 좌우하는 값
      2) 경로 길이 : pose 를 따라 누적. yaw 가 섞인다
      3) 직선 변위 : 시작점→끝점. 정말 직진했다면 1,2 와 같아야 한다
    """

    def __init__(self, topic):
        super().__init__('odom_distance_check')
        self.lock = threading.Lock()
        self.reset()
        self.create_subscription(Odometry, topic, self.cb, 20)

    def reset(self):
        with self.lock:
            self.v_int = 0.0        # ∫|v|dt
            self.path = 0.0         # 경로 길이
            self.p0 = None          # 시작 위치
            self.p1 = None          # 현재 위치
            self.prev_p = None
            self.prev_t = None
            self.n = 0
            self.vmax = 0.0
            self.t0 = None

    def cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t <= 0.0:
            t = self.get_clock().now().nanoseconds * 1e-9
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        v = msg.twist.twist.linear.x

        with self.lock:
            if self.prev_t is not None:
                dt = t - self.prev_t
                if 0.0 < dt < 0.5:
                    self.v_int += abs(v) * dt
            if self.prev_p is not None:
                self.path += math.hypot(x - self.prev_p[0], y - self.prev_p[1])
            if self.p0 is None:
                self.p0 = (x, y)
                self.t0 = t
            self.prev_t = t
            self.prev_p = (x, y)
            self.p1 = (x, y)
            self.vmax = max(self.vmax, abs(v))
            self.n += 1

    def read(self):
        with self.lock:
            straight = (math.hypot(self.p1[0] - self.p0[0], self.p1[1] - self.p0[1])
                        if self.p0 and self.p1 else 0.0)
            dur = (self.prev_t - self.t0) if (self.t0 and self.prev_t) else 0.0
            return self.v_int, self.path, straight, self.n, self.vmax, dur


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--actual', type=float, default=None,
                    help='줄자로 잰 실제 이동거리 (m). 주면 보정 게인까지 계산한다')
    ap.add_argument('--topic', default='/odom_imu',
                    help='구독할 오도메트리 토픽 (기본 /odom_imu, 원본은 /odom)')
    args = ap.parse_args()

    rclpy.init()
    node = DistWatch(args.topic)
    spin = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    spin.start()

    print(__doc__.split('사용법')[0])
    print('=' * 62)
    gain = current_gain()
    print(f'현재 speed_to_erpm_gain : {gain if gain else "(vesc.yaml 을 못 읽음)"}')
    print(f'{args.topic} 수신 대기 중…')

    t0 = time.time()
    while time.time() - t0 < 5.0:
        if node.read()[3] > 5:
            break
        time.sleep(0.2)
    else:
        rclpy.shutdown()
        sys.exit(f'{args.topic} 에 데이터가 없습니다. 스택이 떠 있는지 확인하세요\n'
                 '(./race.sh mapping-joy 또는 ./race.sh joy).')
    print('수신 OK.\n')

    print('출발선에 차를 세우고, 도착 지점을 표시해 두세요.')
    print('급가속은 바퀴가 헛돌아 거리를 과대추정합니다 — 등속 중속으로 직진하세요.')
    input('\n준비되면 Enter → 측정 시작. ')
    node.reset()
    time.sleep(0.3)
    input('도착해서 완전히 멈췄으면 Enter. ')

    v_int, path, straight, n, vmax, dur = node.read()
    if n < 20:
        rclpy.shutdown()
        sys.exit(f'샘플이 {n}개뿐입니다 — 측정이 제대로 안 됐습니다.')

    print('\n' + '=' * 62)
    print(f'∫|v|dt (ERPM 기반)  : {v_int:.3f} m     ← 게인이 직접 좌우하는 값')
    print(f'경로 길이 (pose 누적): {path:.3f} m')
    print(f'직선 변위 (시작→끝)  : {straight:.3f} m')
    print(f'주행 시간 {dur:.1f}s, 최고속 {vmax:.2f} m/s, {n}샘플')

    # 세 값이 크게 다르면 "직진" 가정이 깨진 것 — 게인 계산의 전제가 무너진다
    if v_int > 0.1:
        bend = abs(path - straight) / v_int
        if bend > 0.02:
            print(f'\n⚠ 경로 길이와 직선 변위가 {bend * 100:.1f}% 다릅니다 — 직진이 아니었습니다.')
            print('  게인 보정에는 직진 주행이 필요합니다. 다시 재는 것을 권합니다.')

    if args.actual is None:
        print('\n줄자 실측값이 없어 게인 계산은 생략합니다.')
        print('나중에 실제 거리를 알면 다시 돌릴 필요 없이 아래로 계산하세요:')
        if gain:
            print(f'  new_gain = {gain:.1f} × ({v_int:.3f} / 실제거리)')
        print(f'  (이번 측정의 오도메트리 거리 = {v_int:.3f} m)')
    else:
        actual = args.actual
        ratio = v_int / actual
        print(f'\n실측 거리          : {actual:.3f} m')
        print(f'오차               : {v_int - actual:+.3f} m  ({(ratio - 1) * 100:+.2f}%)')
        print(f'스케일 비          : {ratio:.4f}   (1.0 이 정답)')
        if gain:
            # speed = erpm / gain 이므로, 거리를 k배 줄이려면 gain 을 k배 키운다
            new_gain = gain * ratio
            print(f'\n보정된 speed_to_erpm_gain : {gain:.1f} → **{new_gain:.1f}**')
            print(f'  config/vesc.yaml 의 speed_to_erpm_gain 을 이 값으로 바꾸세요.')
            print('  ※ vesc.yaml 은 f1tenth_stack 으로 심링크돼 있다. f1tenth_stack 을')
            print('    colcon build 하면 install 링크가 복사본으로 덮이니, 그 경우')
            print('    ./install_on_jetson.sh 로 재링크해야 값이 실제로 먹는다.')
        e = abs(ratio - 1.0)
        print()
        if e < 0.01:
            print('✅ 오차 1% 미만 — 보정할 필요 없다.')
        elif e < 0.05:
            print('△ 오차 1~5% — 전형적인 타이어 반경 오차 범위. 보정하면 좋다.')
            print('  2~3회 재서 같은 방향으로 치우치는지 먼저 확인할 것.')
        else:
            print('⚠ 오차 5% 초과 — 슬립이 심했거나 gain 이 많이 틀어져 있다.')
            print('  더 낮은 속도로 다시 재보고, 그래도 같으면 gain 을 보정할 것.')

    print('\n자세한 절차: docs/odometry_calibration.md')
    rclpy.shutdown()
    spin.join(timeout=2.0)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\n중단')
