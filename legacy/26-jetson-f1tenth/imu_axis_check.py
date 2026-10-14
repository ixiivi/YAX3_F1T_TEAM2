#!/usr/bin/env python3
"""
imu_axis_check.py — VESC IMU 의 축 방향·부호가 ROS 규약(REP-103)과 맞는지 확인.

REP-103:  x = 전방,  y = 좌측,  z = 위,  회전은 반시계(위에서 볼 때)가 +
EKF 에 IMU 를 물리기 전에 이걸 먼저 확인해야 한다. 부호가 하나라도 뒤집혀 있으면
yaw rate 가 반대로 들어가 오히려 지금보다 더 크게 어긋난다.

──────────────────────────────────────────────────────────
사전 준비 — 터미널 2개
──────────────────────────────────────────────────────────
[터미널 A] 드라이버 (구동 명령 없음 = 바퀴 안 움직임)

    source /opt/ros/humble/setup.bash
    source ~/f1tenth_ws/install/setup.bash
    ros2 run vesc_driver vesc_driver_node --ros-args --params-file ~/yax/config/vesc.yaml

[터미널 B] 이 스크립트

    source /opt/ros/humble/setup.bash
    source ~/f1tenth_ws/install/setup.bash
    python3 ~/yax/imu_axis_check.py

──────────────────────────────────────────────────────────
차는 바닥에 내려놓고, 안내가 나오면 손으로 움직이면 된다.
모터는 전혀 안 도니 들어올려도 안전하다.

※ 단위는 VESC 원본 그대로 (가속도 g, 자이로 deg/s) — 변환은 나중에 리퍼블리셔가 한다.
"""
import math
import sys
import threading
import time

import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Imu
except ImportError:
    sys.exit('rclpy 가 없습니다. source /opt/ros/humble/setup.bash 먼저 하세요.')

TOPIC = '/sensors/imu/raw'
AXES = ('x', 'y', 'z')


class Probe(Node):
    def __init__(self):
        super().__init__('imu_axis_check')
        self.buf = []
        self.lock = threading.Lock()
        self.create_subscription(Imu, TOPIC, self.cb, 50)

    def cb(self, m):
        with self.lock:
            self.buf.append((
                m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z,
                m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z))

    def grab(self, seconds):
        """seconds 동안 수집한 샘플 → (N,6) 배열"""
        with self.lock:
            self.buf.clear()
        time.sleep(seconds)
        with self.lock:
            return np.array(self.buf) if self.buf else np.zeros((0, 6))

    def grab_until_enter(self, prompt):
        """Enter 를 누를 때까지 수집 → (N,6) 배열.

        시간 제한을 두면 동작을 끝내기 전에 창이 닫혀 판정이 흐려진다.
        사람이 동작을 마친 시점을 직접 알려주게 한다.
        """
        with self.lock:
            self.buf.clear()
        input(prompt)
        with self.lock:
            return np.array(self.buf) if self.buf else np.zeros((0, 6))


def countdown(msg, sec):
    for i in range(sec, 0, -1):
        print(f'\r  {msg} … {i}  ', end='', flush=True)
        time.sleep(1)
    print('\r' + ' ' * 60, end='\r')


def dominant(vec, names=AXES):
    """절댓값이 가장 큰 축의 (이름, 값, 부호문자)"""
    i = int(np.argmax(np.abs(vec)))
    return names[i], float(vec[i]), '+' if vec[i] > 0 else '-'


def main():
    rclpy.init()
    node = Probe()
    spin = threading.Thread(
        target=lambda: rclpy.spin(node), daemon=True)
    spin.start()

    print(__doc__.split('사전 준비')[0])
    print('=' * 62)

    # ── 발행자 확인 ────────────────────────────────────────
    print(f'{TOPIC} 수신 대기 중…')
    t0 = time.time()
    while time.time() - t0 < 5.0:
        with node.lock:
            n = len(node.buf)
        if n > 5:
            break
        time.sleep(0.2)
    else:
        sys.exit(f'\n{TOPIC} 에 데이터가 없습니다.\n'
                 '터미널 A 에서 vesc_driver_node 가 떠 있는지 확인하세요 '
                 '(파일 상단 사용법 참고).')
    print('수신 OK.\n')

    results = {}

    # ── STEP 1. 수평 정지 → 중력이 어느 축인가 ───────────────
    print('[STEP 1/4] 차를 평평한 바닥에 그대로 두세요. 건드리지 마세요.')
    print('          (여기서 잰 정지 자이로 값이 곧 바이어스다 — 길게 재야 정확하다)')
    input('          준비되면 Enter → 10초간 측정합니다. ')
    d = node.grab(10.0)
    if len(d) < 10:
        sys.exit('샘플이 너무 적습니다. 드라이버 상태를 확인하세요.')
    g_rest = d[:, 3:].mean(0)
    w_rest = d[:, :3].mean(0)
    w_std = d[:, :3].std(0)
    ax, val, sign = dominant(g_rest)
    print(f'  가속도 평균 = {np.round(g_rest, 3)}  |a| = {np.linalg.norm(g_rest):.3f}')
    print(f'  자이로 평균 = {np.round(w_rest, 3)}   (정지 바이어스)  σ = {np.round(w_std, 3)}')
    print(f'  → 중력이 걸린 축: {ax} ({sign}), 크기 {abs(val):.3f}')
    if abs(abs(val) - 1.0) < 0.15:
        print('  → 단위는 g (중력=1). ROS 규격(m/s²)이 아니므로 리퍼블리셔에서 ×9.80665 필요.')
    elif abs(abs(val) - 9.81) < 1.5:
        print('  → 단위는 m/s². ROS 규격과 일치.')
    if ax == 'z' and sign == '+':
        print('  ✅ z 축이 위 — REP-103 과 일치')
    else:
        print(f'  ⚠ 위쪽이 z(+) 가 아니라 {ax}({sign}) 입니다. IMU 장착 방향이 회전돼 있음')
    results['up'] = (ax, sign)
    print()

    # ── STEP 2. 앞을 들어올림 → 전방축 ──────────────────────
    print('[STEP 2/4] 차의 **앞쪽(라이다 쪽)을 20~30° 들어올려** 그 자세로 유지하세요.')
    print('          (뒷바퀴는 바닥에 둔 채 앞만 드는 겁니다)')
    input('          들어올린 상태에서 Enter → 3초간 측정합니다. ')
    d = node.grab(3.0)
    g_nose = d[:, 3:].mean(0)
    delta = g_nose - g_rest
    ax, val, sign = dominant(delta)
    print(f'  가속도 평균 = {np.round(g_nose, 3)}   변화량 = {np.round(delta, 3)}')
    print(f'  → 가장 많이 변한 축: {ax} ({sign}{abs(val):.3f})')
    print('  판정 규칙: 위로 들린 쪽을 가리키는 축이 + 로 읽힌다.')
    if ax == 'x' and sign == '+':
        print('  ✅ x(+) 가 전방 — REP-103 과 일치')
    elif ax == 'x':
        print('  ⚠ 전방축은 x 가 맞지만 부호가 반대입니다 (x(-) 가 전방) → 부호 반전 필요')
    else:
        print(f'  ⚠ 전방이 x 가 아니라 {ax}({sign}) 입니다 → 축 재매핑 필요')
    results['forward'] = (ax, sign)
    print('\n          차를 다시 바닥에 내려놓으세요.')
    countdown('안정화 대기', 3)
    print()

    # ── STEP 3. 제자리 좌회전 → yaw 부호 (가장 중요) ─────────
    print('[STEP 3/4] ★ 가장 중요 — 차를 바닥에 둔 채 **제자리에서 왼쪽으로**')
    print('          (위에서 볼 때 반시계 방향) 천천히 90도쯤 돌리세요.')
    print('          좌회전할 때와 같은 방향입니다. 시간 제한 없습니다.')
    input('          Enter 를 누르면 측정 시작 → 그때부터 돌리세요. ')
    d = node.grab_until_enter('          다 돌렸으면 Enter. ')
    w = d[:, :3]
    # 회전 중 실제로 움직인 성분만 — 정지 바이어스 제거 후 평균
    w_mean = (w - w_rest).mean(0)
    w_peak = w[np.argmax(np.abs(w - w_rest).max(1))] - w_rest
    ax, val, sign = dominant(w_mean)
    print(f'  자이로 평균(바이어스 제거) = {np.round(w_mean, 3)} deg/s')
    print(f'  최대 순간값               = {np.round(w_peak, 3)} deg/s')
    print(f'  → 가장 많이 움직인 축: {ax} ({sign}{abs(val):.3f} deg/s)')
    if abs(val) < 1.0:
        print('  ⚠ 회전이 거의 감지 안 됐습니다. 측정 창(4초) 안에 안 돌렸을 수 있습니다 — 다시 해보세요.')
    elif ax == 'z' and sign == '+':
        print('  ✅ 좌회전 = gyro z(+) — REP-103 과 일치. EKF 에 그대로 쓸 수 있음')
    elif ax == 'z':
        print('  ⚠ yaw 축은 z 가 맞지만 **부호가 반대**입니다.')
        print('     이대로 EKF 에 넣으면 회전이 정반대로 들어갑니다 → 리퍼블리셔에서 z 부호 반전 필수')
    else:
        print(f'  ⚠ yaw 가 z 가 아니라 {ax}({sign}) 축에 나옵니다 → 축 재매핑 필요')
    results['yaw'] = (ax, sign)
    print()

    # ── STEP 4. 오른쪽으로 롤 → 좌측축 ──────────────────────
    print('[STEP 4/4] 차의 **왼쪽 바퀴 쪽을 들어올려** (오른쪽으로 기울여) 유지하세요.')
    input('          기울인 상태에서 Enter → 3초간 측정합니다. ')
    d = node.grab(3.0)
    g_roll = d[:, 3:].mean(0)
    delta = g_roll - g_rest
    ax, val, sign = dominant(delta)
    print(f'  가속도 평균 = {np.round(g_roll, 3)}   변화량 = {np.round(delta, 3)}')
    print(f'  → 가장 많이 변한 축: {ax} ({sign}{abs(val):.3f})')
    if ax == 'y' and sign == '+':
        print('  ✅ y(+) 가 좌측 — REP-103 과 일치')
    elif ax == 'y':
        print('  ⚠ 좌우축은 y 가 맞지만 부호가 반대입니다 (y(-) 가 좌측) → 부호 반전 필요')
    else:
        print(f'  ⚠ 좌측이 y 가 아니라 {ax}({sign}) 입니다 → 축 재매핑 필요')
    results['left'] = (ax, sign)
    print('\n          차를 다시 바닥에 내려놓으세요.')

    # ── 요약 ───────────────────────────────────────────────
    print('\n' + '=' * 62)
    print('요약 — 물리 방향 → IMU 축')
    print(f"  위    (up)      : {results['up'][1]}{results['up'][0]}      (기대: +z)")
    print(f"  전방  (forward) : {results['forward'][1]}{results['forward'][0]}      (기대: +x)")
    print(f"  좌측  (left)    : {results['left'][1]}{results['left'][0]}      (기대: +y)")
    print(f"  좌회전(yaw+)    : {results['yaw'][1]}{results['yaw'][0]}      (기대: +z)")

    ok = (results['up'] == ('z', '+') and results['forward'] == ('x', '+')
          and results['left'] == ('y', '+') and results['yaw'] == ('z', '+'))
    print()
    if ok:
        print('✅ 전부 REP-103 과 일치. 리퍼블리셔는 단위 변환(deg/s→rad/s, g→m/s²)과')
        print('   frame_id·공분산만 채우면 됩니다. 축 재매핑 불필요.')
    else:
        print('⚠ 어긋난 항목이 있습니다. 위 표를 그대로 리퍼블리셔의 축 매핑에 반영하세요.')
        print('  특히 yaw 부호가 반대면 EKF 가 회전을 정반대로 적분합니다.')
    print('\n정지 상태 자이로 바이어스 (리퍼블리셔에서 빼줄 값, deg/s):')
    print(f'  {np.round(w_rest, 4)}   σ = {np.round(w_std, 4)}')

    # spin 스레드가 도는 중에 destroy_node() 를 부르면 abort(core dump) 한다.
    # shutdown → spin 반환 → join 순서로 끝낸다.
    rclpy.shutdown()
    spin.join(timeout=2.0)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\n중단')
