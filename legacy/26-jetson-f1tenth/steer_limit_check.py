#!/usr/bin/env python3
"""
steer_limit_check.py — 조향의 **물리적 한계**를 실측한다.

왜 필요한가
───────────
`race_script.yaml` 의 `max_steer: 0.36` 과 `config/vesc.yaml` 의
`servo_min: 0.15 / servo_max: 0.85` 가 진짜 기구 스토퍼인지는 주석의 주장일 뿐이다
(2026-07-16 실측이라 적혀 있음). 실제로 레이싱라인이 요구하는 곡률 1.72(29.6°)를
좌회전에서 못 따라가는 문제가 있어, 좌측 한계가 정말 20.7° 인지 확인해야 한다.

이 도구는 Enter 를 칠 때마다 서보를 한 칸씩 더 꺾는다. **앞바퀴가 더 이상 안
움직이는 지점**이 진짜 기구 한계다.

★★ 반드시 바퀴를 공중에 띄운 상태(박스 위)에서 할 것. ★★
   모터에는 아무 명령도 보내지 않으므로 차가 나가지는 않는다.
   다만 스토퍼에 물린 채로 오래 두면 서보가 발열하므로, 한계에 닿으면 바로
   Enter 없이 q 로 빠져나올 것 (종료 시 자동으로 중앙 복귀).

사전 준비 — 다른 노드가 같은 토픽을 쓰면 안 된다
────────────────────────────────────────────────
`/commands/servo/position` 은 `ackermann_to_vesc` 와 `throttle_interpolator` 도
쓴다. 스택이 떠 있으면 명령이 서로 덮어써서 결과가 무의미하다.

    ./race.sh stop                       # 먼저 스택을 완전히 정리
    source /opt/ros/humble/setup.bash
    source ~/f1tenth_ws/install/setup.bash
    ros2 run vesc_driver vesc_driver_node --ros-args \
        --params-file ~/yax/config/vesc.yaml          # 터미널 A (드라이버만)

    python3 ~/yax/steer_limit_check.py --side left     # 터미널 B

사용
────
    --side left|right   어느 쪽으로 꺾을지 (기본 left)
    --step 0.01         Enter 한 번에 움직일 서보 량 (기본 0.01 ≈ 0.75°)
    --floor 0.02        서보 하한 (안전). 기본 0.02 / 상한 0.98

결과 읽는 법
────────────
바퀴가 멈춘 지점의 서보 값이 기구 한계다. 그 값을 `config/vesc.yaml` 의
`servo_min`(좌) / `servo_max`(우) 로 넣고, 대응하는 조향각을 `max_steer` 로 쓰면
된다. 좌우가 다르면 작은 쪽이 대칭 `max_steer` 의 상한이다.
"""
import argparse
import math
import os
import sys
import threading

import rclpy
import yaml
from rclpy.node import Node
from std_msgs.msg import Float64

REPO = os.path.dirname(os.path.abspath(__file__))
VESC_YAML = os.path.join(REPO, 'config', 'vesc.yaml')


def load_cal():
    """vesc.yaml 에서 서보 매핑 계수를 읽는다."""
    cfg = yaml.safe_load(open(VESC_YAML)) or {}
    p = (cfg.get('/**') or {}).get('ros__parameters') or {}
    return (float(p['steering_angle_to_servo_gain']),
            float(p['steering_angle_to_servo_offset']),
            float(p.get('servo_min', 0.15)),
            float(p.get('servo_max', 0.85)))


def main():
    gain, offset, smin, smax = load_cal()
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--side', choices=('left', 'right'), default='left')
    ap.add_argument('--step', type=float, default=0.01)
    ap.add_argument('--floor', type=float, default=0.02)
    ap.add_argument('--ceil', type=float, default=0.98)
    args = ap.parse_args()

    # servo = gain·δ + offset. gain 이 음수이므로 서보를 줄이면 δ 가 커진다(좌).
    to_angle = lambda s: (s - offset) / gain
    # 좌(+δ) 로 가려면 서보를 어느 방향으로 움직여야 하나
    dirn = -1.0 if (args.side == 'left') == (gain < 0) else +1.0
    limit = args.floor if dirn < 0 else args.ceil
    cfg_lim = smin if dirn < 0 else smax

    rclpy.init()
    node = Node('steer_limit_check')
    pub = node.create_publisher(Float64, '/commands/servo/position', 10)
    spin = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    spin.start()

    print(__doc__.split('사전 준비')[0])
    print('=' * 62)
    print(f'서보 매핑 : servo = {gain} × δ(rad) + {offset}')
    print(f'  중앙 δ=0      → 서보 {offset:.3f}')
    print(f'  yaml 한계     → 좌 servo_min {smin:.2f} = {math.degrees(to_angle(smin)):+.1f}° '
          f'/ 우 servo_max {smax:.2f} = {math.degrees(to_angle(smax)):+.1f}°')
    print(f'\n방향 {args.side} — Enter 마다 서보 {args.step:+.3f} '
          f'({abs(args.step/gain)*180/math.pi:.2f}°) 씩 이동, 하드리밋 {limit:.2f}')
    print('\n★ 바퀴를 공중에 띄웠는지 확인하세요. 모터 명령은 보내지 않습니다.')
    print('  Enter = 한 칸 더 / b = 되돌리기 / c = 중앙 / 숫자 = 서보값 직접 지정 / q = 종료\n')

    servo = offset
    pub.publish(Float64(data=servo))
    print(f'  중앙으로 이동: 서보 {servo:.3f}  (δ {math.degrees(to_angle(servo)):+.1f}°)')

    def show(s, note=''):
        d = to_angle(s)
        k = math.tan(abs(d)) / 0.33 if abs(d) > 1e-6 else 0.0
        r = (1 / k) if k > 1e-6 else float('inf')
        flag = ''
        if (dirn < 0 and s < cfg_lim) or (dirn > 0 and s > cfg_lim):
            flag = '  ← yaml 한계를 넘었음'
        print(f'  서보 {s:.3f}   조향 {math.degrees(d):+6.2f}°   '
              f'최소반경 {r:5.2f}m   곡률 {k:4.2f}{flag}{note}')

    try:
        while True:
            cmd = input('  > ').strip().lower()
            if cmd == 'q':
                break
            if cmd == 'c':
                servo = offset
                pub.publish(Float64(data=servo)); show(servo, '  (중앙)')
                continue
            if cmd and (cmd[0].isdigit() or cmd[0] == '.'):
                # 서보 값을 직접 입력해 바로 이동 (중립 찾을 때 편하다)
                try:
                    v = float(cmd)
                except ValueError:
                    print('  숫자를 못 읽었습니다'); continue
                if not (args.floor <= v <= args.ceil):
                    print(f'  {args.floor}~{args.ceil} 범위로 입력하세요'); continue
                servo = v
                pub.publish(Float64(data=servo)); show(servo, '  (직접 지정)')
                continue
            if cmd == 'b':
                servo = min(max(servo - dirn * args.step, args.floor), args.ceil)
            else:
                nxt = servo + dirn * args.step
                if (dirn < 0 and nxt < args.floor) or (dirn > 0 and nxt > args.ceil):
                    print(f'  하드리밋 {limit:.2f} 도달 — 더 못 갑니다 (안전 상한)')
                    continue
                servo = nxt
            pub.publish(Float64(data=servo))
            show(servo)
    except (EOFError, KeyboardInterrupt):
        pass
    finally:
        print(f'\n중앙 복귀: 서보 {offset:.3f}')
        pub.publish(Float64(data=offset))
        import time
        time.sleep(0.3)
        rclpy.shutdown()
        spin.join(timeout=1.0)
        print('\n바퀴가 멈춘 지점의 서보 값이 기구 한계입니다.')
        print('  → config/vesc.yaml 의 servo_min(좌)/servo_max(우) 에 넣고,')
        print('    그 조향각을 race_script.yaml 의 pure_pursuit.max_steer 로 쓰세요.')
        print('    좌우가 다르면 **작은 쪽**이 대칭 max_steer 의 상한입니다.')


if __name__ == '__main__':
    main()
