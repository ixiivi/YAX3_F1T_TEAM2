#!/usr/bin/env python3
"""
주행 방식 선택 런처
실행: ros2 run f1tenth_racing racing_node
"""

import os
import rclpy

# 선택한 컨트롤러 이름을 lap_timer_node 가 읽어 CSV 에 기록함
CONTROLLER_FILE = os.path.expanduser('~/.f1tenth_active_controller')


def main(args=None):
    print()
    print('┌─────────────────────────────────────────┐')
    print('│       F1TENTH  주행 방식 선택            │')
    print('├─────────────────────────────────────────┤')
    print('│  1. Pure Pursuit + FTG  (클래식)         │')
    print('│  2. Hybrid  (APP + Stanley + LQR + FTG) │')
    print('└─────────────────────────────────────────┘')

    while True:
        choice = input('선택 (1/2): ').strip()
        if choice in ('1', '2'):
            break
        print('  → 1 또는 2를 입력하세요.')

    if choice == '1':
        controller = 'pure_pursuit'
        from f1tenth_racing.pure_pursuit_node import PurePursuit as NodeClass
        print('\n[Pure Pursuit + FTG] 컨트롤러로 시작합니다.\n')
    else:
        controller = 'hybrid'
        from f1tenth_racing.hybrid_controller_node import HybridController as NodeClass
        print('\n[Hybrid Controller (APP + Stanley + LQR + FTG)] 컨트롤러로 시작합니다.\n')

    # lap_timer_node 가 읽을 컨트롤러 이름 저장
    with open(CONTROLLER_FILE, 'w') as f:
        f.write(controller)

    rclpy.init(args=args)
    node = NodeClass()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
