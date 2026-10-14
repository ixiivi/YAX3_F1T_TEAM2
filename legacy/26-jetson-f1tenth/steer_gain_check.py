#!/usr/bin/env python3
"""steer_gain_check.py — 조향 게인·중립을 **바닥에 그려진 원**으로 실측한다.

원리
────
자전거 모델에서  δ = atan(wheelbase / R)  — 원의 반지름만 재면 조향각이 역산된다.
서보를 양쪽 끝에 고정해 두 점을 재면 servo↔δ 직선이 결정된다:

    gain   = (s₂ − s₁) / (δ₂ − δ₁)
    offset = s₁ − gain · δ₁

⚠ yaw_check.py 의 odom yaw 로 재면 안 된다. odom 각속도 자체가 지금 검증하려는
  게인으로 계산되므로 순환논리다. **바닥의 실제 원**을 자로 재야 한다.

왜 이 스크립트인가
──────────────────
원래는 터미널 두 개에 긴 명령을 직접 쳐야 했다 (vesc_driver + ros2 topic pub).
`--ros-args` 같은 플래그가 SSH 복사 과정에서 하이픈이 깨져 계속 실패했다
(2026-08-13). 여기서 드라이버 기동·서보 유지·계산을 다 한다.

**모터에는 아무 명령도 보내지 않는다.** 서보만 고정하고 차는 손으로 민다 —
타이어 슬립이 없어 더 정확하고 안전하다.

사용법
──────
    cd ~/yax/26-jetson-f1tenth
    source envsetup.sh
    ./race.sh stop          # ★ 스택이 떠 있으면 서보 명령을 덮어쓴다
    python3 steer_gain_check.py

R 을 어디서 재나 — **뒷바퀴 축 중심**
    간편법: 뒤축 중심을 바닥에 표시 → 정확히 반 바퀴(차 머리가 정반대) 민 뒤
            다시 표시 → 두 점 사이 거리 = 지름
    정확법: 뒤축 한가운데에 분필을 붙여 바닥에 선이 그려지게 한다
  ※ 천천히 밀 것. 빨리 밀면 슬립으로 원이 커진다.
"""
import math
import os
import subprocess
import sys
import threading
import time

import rclpy
import yaml
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64

REPO = os.path.dirname(os.path.abspath(__file__))
VESC_YAML = os.path.join(REPO, 'config', 'vesc.yaml')


def read_vesc():
    with open(VESC_YAML) as f:
        p = yaml.safe_load(f)['/**']['ros__parameters']
    return p


class ServoHold(Node):
    """서보를 지정 위치로 20Hz 유지. 드라이버는 명령이 끊기면 마지막 값을 잡고 있지만,
    autorepeat 로 계속 보내는 편이 다른 노드가 끼어들었을 때 티가 난다."""

    def __init__(self):
        super().__init__('steer_gain_check')
        self.pub = self.create_publisher(Float64, '/commands/servo/position', 10)
        self.value = None
        self.create_timer(0.05, self._tick)

    def _tick(self):
        if self.value is not None:
            self.pub.publish(Float64(data=float(self.value)))


def ask_diameter(label):
    while True:
        s = input(f'  {label} 지름(m) 입력 (s = 이 방향 건너뛰기): ').strip()
        if s.lower() == 's':
            return None
        try:
            d = float(s)
        except ValueError:
            print('    숫자로 입력하세요 (예: 1.72)')
            continue
        if not (0.2 <= d <= 20.0):
            print('    0.2~20 m 범위로 입력하세요')
            continue
        return d


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('--params', default=VESC_YAML,
                    help='드라이버에 넘길 params 파일. 소프트리밋 밖을 재려면 '
                         'config/vesc_steer_test.yaml (0.02~0.98) 을 쓴다')
    ap.add_argument('--servo', type=float, nargs=2, default=None,
                    metavar=('S1', 'S2'),
                    help='시험할 두 서보값 (기본: params 의 servo_min/max). '
                         '스토퍼 근처를 재려면 예: --servo 0.05 0.95')
    args = ap.parse_args()
    global VESC_YAML_USED
    VESC_YAML_USED = args.params
    if not os.path.exists(args.params):
        sys.exit(f'✗ {args.params} 이 없습니다.')
    if not os.path.exists(VESC_YAML):
        sys.exit(f'✗ {VESC_YAML} 이 없습니다.\n'
                 f'  python3 config/apply_vehicle.py 를 먼저 돌리세요.')
    # 게인·중립은 항상 실제 설정(vesc.yaml)에서 읽는다 — 비교 기준이므로.
    p = read_vesc()
    pd = yaml.safe_load(open(args.params))['/**']['ros__parameters']
    if args.servo:
        smin, smax = sorted(args.servo)
    else:
        smin, smax = float(pd['servo_min']), float(pd['servo_max'])
    wb = float(p.get('wheelbase', 0.33))
    cur_gain = float(p['steering_angle_to_servo_gain'])
    cur_off = float(p['steering_angle_to_servo_offset'])

    print(__doc__.split('사용법')[0])
    print('=' * 62)
    print(f'현재 설정  gain {cur_gain:+.4f}   offset {cur_off:.4f}   wheelbase {wb}m')
    print(f'시험 서보  {smin} / {smax}   (params: {os.path.basename(args.params)})')
    print(f'현재 값이 맞다면 예상 지름:')
    for s in (smin, smax):
        d = (s - cur_off) / cur_gain
        r = wb / math.tan(abs(d)) if abs(d) > 1e-6 else float('inf')
        print(f'    servo {s:.2f} → {math.degrees(d):+6.2f}°  지름 약 {2*r:.2f} m')
    print('=' * 62)

    # ── vesc_driver 기동 (플래그 타이핑 없이) ──────────────────────
    print('\nVESC 드라이버를 띄웁니다...')
    drv = subprocess.Popen(
        ['ros2', 'run', 'vesc_driver', 'vesc_driver_node',
         '--ros-args', '--params-file', args.params],
        stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
    time.sleep(5)
    if drv.poll() is not None:
        err = drv.stderr.read() if drv.stderr else ''
        sys.exit(f'✗ 드라이버가 죽었습니다.\n{err[-600:]}\n'
                 f'  ./race.sh stop 으로 다른 프로세스를 정리했는지 확인하세요.')
    print('  드라이버 기동 완료.')

    rclpy.init()
    node = ServoHold()
    th = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    th.start()

    results = {}
    try:
        for tag, s in (('① 한쪽 끝', smin), ('② 반대쪽 끝', smax)):
            print(f'\n{tag} — 서보 {s:.2f} 로 고정합니다.')
            node.value = s
            time.sleep(0.5)
            print('  차를 손으로 **천천히** 밀어 원을 그리세요.')
            print('  뒤축 중심 기준으로 지름을 재고 아래에 입력합니다.')
            d = ask_diameter(f'servo {s:.2f}')
            if d is not None:
                results[s] = d
        node.value = cur_off      # 중앙 복귀
        time.sleep(0.5)
    except (KeyboardInterrupt, EOFError):
        print('\n중단했습니다.')
    finally:
        try:
            node.value = cur_off
            time.sleep(0.3)
            rclpy.shutdown()
        except Exception:
            pass
        drv.terminate()
        try:
            drv.wait(timeout=5)
        except Exception:
            drv.kill()
        print('\n드라이버 종료, 서보 중앙 복귀.')

    # ── 계산 ───────────────────────────────────────────────────────
    if len(results) < 2:
        print('\n두 방향을 모두 재야 게인이 나옵니다. 잰 값:')
        for s, d in results.items():
            r = d / 2
            print(f'  servo {s:.2f}: 지름 {d:.3f}m → R {r:.3f}m → '
                  f'δ {math.degrees(math.atan(wb / r)):.2f}°')
        return

    (s1, d1), (s2, d2) = sorted(results.items())
    r1, r2 = d1 / 2, d2 / 2
    # servo_min 쪽이 +δ (좌), servo_max 쪽이 −δ (우) — gain 이 음수인 규약
    dl1 = +math.atan(wb / r1)
    dl2 = -math.atan(wb / r2)
    gain = (s2 - s1) / (dl2 - dl1)
    off = s1 - gain * dl1

    print('\n' + '=' * 62)
    print('실측 결과')
    print(f'  servo {s1:.2f}: 지름 {d1:.3f}m → R {r1:.3f}m → δ {math.degrees(dl1):+6.2f}°')
    print(f'  servo {s2:.2f}: 지름 {d2:.3f}m → R {r2:.3f}m → δ {math.degrees(dl2):+6.2f}°')
    print()
    print(f'  gain    {cur_gain:+.4f}  →  {gain:+.4f}   ({100*(gain-cur_gain)/abs(cur_gain):+.1f}%)')
    print(f'  offset  {cur_off:.4f}  →  {off:.4f}   (차이 {off-cur_off:+.4f})')
    print()
    print('  ★ offset 은 steer_trim 으로 잰 값과의 **교차 검증**이다.')
    if abs(off - cur_off) < 0.02:
        print(f'    차이 {abs(off-cur_off):.4f} < 0.02 — 두 방법이 일치한다. 신뢰할 만하다.')
    else:
        print(f'    차이 {abs(off-cur_off):.4f} 가 크다. 원을 다시 재거나, 밀 때')
        print('    슬립이 있었는지(빨리 밀면 원이 커진다) 확인할 것.')
    print()
    print('  config/vehicle_car2.yaml 에 반영:')
    print(f'    steering_angle_to_servo_gain:   {gain:.4f}')
    print(f'  ※ offset 은 steer_trim 값이 더 직접적이므로 그대로 두는 편이 낫다.')
    print(f'    (반영 후  python3 config/apply_vehicle.py  → max_steer 가 다시 계산된다)')
    print('=' * 62)


if __name__ == '__main__':
    main()
