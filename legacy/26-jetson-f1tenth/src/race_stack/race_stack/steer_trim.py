#!/usr/bin/env python3
"""
steer_trim.py — 조향 중립값(트림) 자동 캘리브레이션 리그.

docs/steering_calibration.md 의 "3. 튜닝 절차" 루프(직진 → 눈으로 휘는 정도
확인 → vesc.yaml offset 수동 조정 → 반복)를 사람 눈 대신 IMU로 자동화한다.

■ 왜 IMU 기반인가 (odom 대신)
  차체 좌표계 lateral offset(y)을 거리로 재는 대신, 자이로 각속도(wz)를
  주행거리로 적분해 평균 곡률 κ_avg = Δψ/Δs 를 직접 구한다. 작은 각 근사
  (y ≈ κs²/2) 없이 바로 곡률이 나오고, odom 의 각속도 자체가 이 튜닝의
  검증 대상(조향 offset)에 의존하는 순환논리도 피한다 — steering_calibration.md
  6번 항목이 조향 "게인" 실측에서 지적한 것과 같은 함정을 offset 실측에서도
  피하려는 것.

■ 동작 (요청 스펙 그대로)
  1. 직진 방향 조향각(trim) 명령으로 test_dist(기본 2m) 주행
  2. IMU로 좌우 어느 쪽으로 치우쳤는지 확인 (Δψ/Δs = 그 trim에서의 실제 곡률)
     → 반대 방향으로 trim 이동: trim -= damp · wheelbase · κ_measured
        (자전거 모델 κ=tan(δ)/L ≈ δ/L 에서 나온 뉴턴 스텝 — 이론상 한 번에
         수렴하는 보정량, damp<1 로 여유를 둔다)
  3. 그 trim으로 후진 test_dist 주행 — 검증이자 동시에 원위치 복귀
     (같은 trim이 전진·후진에서 다른 κ를 내면 백래시 신호 — 끝에 진단 출력)
  4. 최근 두 다리(전진/후진)의 |κ| 가 모두 kappa_tol 이하일 때까지 반복

■ 안전
  전진 중엔 전방, 후진 중엔 후방 LiDAR 최소거리를 감시해 safety_m 미만이면
  즉시 정지(E-stop). 후방은 라이다 FOV 밖일 수 있어(장착 방향에 따라) 그
  구간에 유효 빔이 없으면 경고만 하고 거리 기반 정지에만 의존한다 —
  **뒤쪽 공간도 사람이 직접 확보해둘 것.** q 키로 언제든 즉시 중단.

■ 최종 반영
  수렴(또는 q로 중단 후 s로 저장 선택)하면 config/vesc.yaml 의
  steering_angle_to_servo_offset 을 offset_new = offset_old + gain·trim 로
  직접 계산해 그 줄만 치환(다른 주석/값은 안 건드림). servo_min/max 클리핑
  안전마진을 벗어나면 자동 반영을 거부하고 값만 출력한다.
  ★ ackermann_to_vesc 는 파라미터를 시작 시 한 번만 읽는다(docs 3번 함정) —
  적용하려면 스택 재시작 필요, 이 노드가 대신 해주지 않는다.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
"""

import math
import os
import re
import sys
import time


def _repo_root():
    if os.environ.get('RACE_REPO'):
        return os.environ['RACE_REPO']
    # <repo>/src/race_stack/race_stack/steer_trim.py 기준 4단계 위
    return os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__)))))


BOX_W = 62


def _row(s):
    pad = BOX_W - len(s)
    return '║' + s + ' ' * max(pad, 0) + '║'


def _run_node():
    try:
        sys.stdout.reconfigure(line_buffering=True)
    except Exception:
        pass

    import select
    import termios
    import threading
    import tty

    import numpy as np
    import rclpy
    from ackermann_msgs.msg import AckermannDriveStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu, LaserScan

    (BIAS, FWD_RUN, FWD_STOP, PAUSE_A,
     BWD_RUN, BWD_STOP, PAUSE_B, DONE) = range(8)
    STATE_NAME = {
        BIAS: '바이어스', FWD_RUN: '전진', FWD_STOP: '전진-정지',
        PAUSE_A: '대기', BWD_RUN: '후진', BWD_STOP: '후진-정지',
        PAUSE_B: '대기', DONE: '완료',
    }

    class SteerTrim(Node):
        def __init__(self):
            super().__init__('steer_trim')
            d = self.declare_parameter
            d('imu_topic', '/sensors/imu/raw')
            d('odom_topic', '/odom')
            d('scan_topic', '/scan')
            d('gyro_units', 'deg')
            d('wheelbase', 0.33)
            d('lpf_hz', 8.0)
            d('test_speed', 0.6)
            d('test_dist', 2.0)
            d('accel_limit', 0.4)
            d('bias_sec', 2.0)
            d('pause_sec', 1.0)
            d('max_trim', 0.15)
            d('kappa_tol', 0.01)
            d('correction_damp', 0.85)
            d('max_iters', 20)
            d('front_safety_m', 0.4)
            d('rear_safety_m', 0.4)
            d('min_speed', 0.15)
            d('auto_write', True)

            p = lambda n: self.get_parameter(n).value
            self.L = float(p('wheelbase'))
            self.to_rad = (math.pi / 180.0
                           if str(p('gyro_units')).lower().startswith('deg') else 1.0)
            self.lpf_hz = float(p('lpf_hz'))
            self.test_speed = float(p('test_speed'))
            self.test_dist = float(p('test_dist'))
            self.accel_limit = float(p('accel_limit'))
            self.bias_sec = float(p('bias_sec'))
            self.pause_sec = float(p('pause_sec'))
            self.max_trim = float(p('max_trim'))
            self.kappa_tol = float(p('kappa_tol'))
            self.damp = float(p('correction_damp'))
            self.max_iters = int(p('max_iters'))
            self.front_safety_m = float(p('front_safety_m'))
            self.rear_safety_m = float(p('rear_safety_m'))
            self.min_speed = float(p('min_speed'))
            self.auto_write = bool(p('auto_write'))

            self.state = BIAS
            self.t_state = time.time()
            self.trim = 0.0
            self.iter_n = 0
            self.speed_cmd = 0.0
            self.speed_target = 0.0
            self.steer_cmd = 0.0

            self.v = 0.0
            self.pos = None          # (x, y) 최신 odom
            self.leg_start_pos = None
            self.leg_dist = 0.0

            self.last_t = None
            self.bias_wz = 0.0
            self._warm = []
            self.psi_accum = 0.0     # 다리(leg) 누적 헤딩 변화

            self.front_min = float('inf')
            self.rear_min = float('inf')
            self._scan_has_rear = None   # None=아직 모름

            self.history = []        # (leg, trim_used, kappa, dist)
            self._msg = '정지 유지 — 자이로 바이어스 측정 중'
            self._seen = {'odom': 0.0, 'imu': 0.0, 'scan': 0.0}
            self._estop = False
            self._stop_event = threading.Event()
            self._save_now = False
            self._abort_no_save = False
            self._finalized = False

            self.pub = self.create_publisher(AckermannDriveStamped, '/teleop', 10)
            self.create_subscription(Odometry, p('odom_topic'), self._odom_cb, 20)
            self.create_subscription(Imu, p('imu_topic'), self._imu_cb,
                                     qos_profile_sensor_data)
            self.create_subscription(LaserScan, p('scan_topic'), self._scan_cb,
                                     qos_profile_sensor_data)
            self.create_timer(0.05, self._control_tick)
            self.create_timer(0.3, self._draw)

            self.get_logger().info(
                f'SteerTrim ready — test_dist={self.test_dist}m  '
                f'test_speed={self.test_speed}㎧  kappa_tol={self.kappa_tol}  '
                f'q:즉시중단  s:이번 다리 끝나면 저장하고 종료')

        # ── 입력 ──────────────────────────────────────────────────────
        def _odom_cb(self, msg: Odometry):
            self._seen['odom'] = time.time()
            self.v = float(msg.twist.twist.linear.x)
            self.pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
            if self.leg_start_pos is not None:
                dx = self.pos[0] - self.leg_start_pos[0]
                dy = self.pos[1] - self.leg_start_pos[1]
                self.leg_dist = math.hypot(dx, dy)

        def _scan_cb(self, msg: LaserScan):
            self._seen['scan'] = time.time()
            ranges = np.asarray(msg.ranges)
            n = len(ranges)
            if n == 0:
                return
            angles = msg.angle_min + np.arange(n) * msg.angle_increment
            finite = np.isfinite(ranges) & (ranges > msg.range_min) & (ranges < msg.range_max)

            front_mask = finite & (np.abs(angles) < math.radians(15))
            self.front_min = float(ranges[front_mask].min()) if front_mask.any() else float('inf')

            rear_mask = finite & (np.abs(np.abs(angles) - math.pi) < math.radians(15))
            if rear_mask.any():
                self.rear_min = float(ranges[rear_mask].min())
                if self._scan_has_rear is None:
                    self._scan_has_rear = True
            else:
                self.rear_min = float('inf')
                if self._scan_has_rear is None:
                    self._scan_has_rear = False

        def on_key(self, key):
            if key in ('q', '\x03'):
                self._msg = '즉시 중단 — 저장 안 함'
                self._abort_no_save = True
                self.state = DONE
                self.speed_target = 0.0
            elif key == 's':
                self._save_now = True
                self._msg = '이번 다리 끝나면 저장 후 종료합니다'

        # ── 제어 루프 (20Hz) ─────────────────────────────────────────
        def _control_tick(self):
            dt = 0.05

            if self._estop:
                self.speed_target = 0.0
                self.speed_cmd = 0.0
                self.steer_cmd = 0.0
                self._publish(0.0, 0.0)
                return

            if self.state == BIAS:
                self.steer_cmd = 0.0; self.speed_cmd = 0.0
                if time.time() - self.t_state >= self.bias_sec:
                    self.bias_wz = float(np.mean(self._warm)) if self._warm else 0.0
                    self._start_leg(FWD_RUN, +1)

            elif self.state in (FWD_RUN, BWD_RUN):
                direction = +1 if self.state == FWD_RUN else -1
                if direction > 0 and self.front_min < self.front_safety_m:
                    self._estop_now(f'전방 장애물 {self.front_min:.2f}m — 비상정지')
                    return
                if (direction < 0 and self._scan_has_rear
                        and self.rear_min < self.rear_safety_m):
                    self._estop_now(f'후방 장애물 {self.rear_min:.2f}m — 비상정지')
                    return

                self.steer_cmd = self.trim
                self.speed_target = direction * self.test_speed
                self._ramp(dt)

                if self.leg_dist >= self.test_dist:
                    self.speed_target = 0.0
                    self.state = FWD_STOP if self.state == FWD_RUN else BWD_STOP

            elif self.state in (FWD_STOP, BWD_STOP):
                self.steer_cmd = self.trim
                self.speed_target = 0.0
                self._ramp(dt)
                if abs(self.speed_cmd) < 0.02:
                    self.speed_cmd = 0.0
                    self._finish_leg(FWD_RUN if self.state == FWD_STOP else BWD_RUN)

            elif self.state in (PAUSE_A, PAUSE_B):
                self.steer_cmd = 0.0; self.speed_cmd = 0.0
                if time.time() - self.t_state >= self.pause_sec:
                    if self.state == PAUSE_A:
                        self._start_leg(BWD_RUN, -1)
                    else:
                        self._next_iteration()

            elif self.state == DONE:
                self.speed_cmd = 0.0; self.steer_cmd = 0.0
                self._publish(0.0, 0.0)
                if not self._finalized:
                    self._finalized = True
                    self._finalize()
                self._stop_event.set()
                return

            self._publish(self.steer_cmd, self.speed_cmd)

        def _ramp(self, dt):
            diff = self.speed_target - self.speed_cmd
            step = self.accel_limit * dt
            if abs(diff) <= step:
                self.speed_cmd = self.speed_target
            else:
                self.speed_cmd += step if diff > 0 else -step

        def _estop_now(self, msg):
            self._estop = True
            self._msg = msg
            self.get_logger().error(msg)

        def _start_leg(self, state, direction):
            self.state = state
            self.leg_start_pos = self.pos
            self.leg_dist = 0.0
            self.psi_accum = 0.0
            self.t_state = time.time()
            label = '전진' if direction > 0 else '후진'
            self._msg = f'[{self.iter_n+1}회차] {label} trim={self.trim:+.4f}rad 시작'

        def _finish_leg(self, which_run_state):
            s = self.leg_dist if self.leg_dist > 1e-3 else self.test_dist
            signed_s = s if which_run_state == FWD_RUN else -s
            kappa = self.psi_accum / signed_s if abs(signed_s) > 1e-3 else 0.0
            leg_name = 'fwd' if which_run_state == FWD_RUN else 'bwd'
            self.history.append((leg_name, self.trim, kappa, s))

            old_trim = self.trim
            self.trim -= self.damp * self.L * kappa
            self.trim = max(-self.max_trim, min(self.max_trim, self.trim))

            self._msg = (f'[{leg_name}] κ={kappa:+.4f} 1/m  '
                         f'trim {old_trim:+.4f} → {self.trim:+.4f}')

            if which_run_state == FWD_RUN:
                self.state = PAUSE_A
            else:
                self.state = PAUSE_B
            self.t_state = time.time()

        def _converged(self):
            if len(self.history) < 2:
                return False
            last2 = self.history[-2:]
            return all(abs(k) <= self.kappa_tol for _, _, k, _ in last2)

        def _next_iteration(self):
            self.iter_n += 1
            if self._save_now:
                self._msg = 's 입력으로 저장 후 종료'
                self.state = DONE
            elif self._converged():
                self._msg = f'{self.iter_n}회차만에 수렴 (|κ| ≤ {self.kappa_tol})'
                self.state = DONE
            elif self.iter_n >= self.max_iters:
                self._msg = f'최대 반복({self.max_iters}) 도달 — 수렴 못함, 마지막 trim 참고용'
                self.state = DONE
            else:
                self._start_leg(FWD_RUN, +1)

        def _publish(self, steer, speed):
            msg = AckermannDriveStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.drive.speed = float(speed)
            msg.drive.steering_angle = float(steer)
            self.pub.publish(msg)

        # ── IMU ───────────────────────────────────────────────────────
        def _imu_cb(self, msg: Imu):
            self._seen['imu'] = time.time()
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if t <= 0.0:
                t = self.get_clock().now().nanoseconds * 1e-9
            dt = (t - self.last_t) if self.last_t else 0.02
            self.last_t = t
            if not (1e-4 < dt < 0.5):
                dt = 0.02

            wz = +msg.angular_velocity.z * self.to_rad   # curvature_probe.py 와 동일 축 매핑

            if self.state == BIAS:
                self._warm.append(wz)
                return

            wz -= self.bias_wz
            if self.state in (FWD_RUN, BWD_RUN):
                self.psi_accum += wz * dt

        # ── 최종 반영 ─────────────────────────────────────────────────
        def _finalize(self):
            print()
            if self._abort_no_save:
                print(f'중단됨 — 마지막 trim {self.trim:+.4f} rad (저장 안 함)')
                return
            if not self.history:
                print('표본 없음 — 저장 생략'); return

            fwd_k = [k for l, _, k, _ in self.history if l == 'fwd']
            bwd_k = [k for l, _, k, _ in self.history if l == 'bwd']
            print(f'최종 trim = {self.trim:+.4f} rad  '
                  f'(iter {self.iter_n}, 전진 {len(fwd_k)}회/후진 {len(bwd_k)}회)')
            if fwd_k and bwd_k and (abs(fwd_k[-1]) > 1e-9 or abs(bwd_k[-1]) > 1e-9):
                asym = abs(fwd_k[-1] - bwd_k[-1])
                if asym > 2 * self.kappa_tol:
                    print(f'⚠ 전진/후진 마지막 κ 차이 {asym:.4f} 1/m — 백래시 의심'
                          f'(steering_calibration.md 4번 항목 참고)')

            if not self.auto_write:
                print('auto_write=false — vesc.yaml 은 건드리지 않음')
                return
            self._write_vesc_yaml()

        def _write_vesc_yaml(self):
            path = os.path.join(_repo_root(), 'config', 'vesc.yaml')
            if not os.path.exists(path):
                print(f'vesc.yaml 못 찾음: {path} — 수동 반영 필요 (trim={self.trim:+.4f})')
                return
            text = open(path).read()

            def _grab(pattern):
                m = re.search(pattern, text)
                return float(m.group(1)) if m else None

            gain = _grab(r'steering_angle_to_servo_gain:\s*([-\d.]+)')
            offset_old = _grab(r'steering_angle_to_servo_offset:\s*([-\d.]+)')
            servo_min = _grab(r'servo_min:\s*([-\d.]+)')
            servo_max = _grab(r'servo_max:\s*([-\d.]+)')
            if None in (gain, offset_old, servo_min, servo_max):
                print(f'vesc.yaml 파싱 실패 — 수동 반영 필요 (trim={self.trim:+.4f}, '
                      f'gain/offset/servo_min/servo_max 정규식 확인)')
                return

            offset_new = offset_old + gain * self.trim
            margin = 0.05
            if not (servo_min + margin <= offset_new <= servo_max - margin):
                print(f'⚠ 계산된 offset({offset_new:.4f})이 servo 안전범위'
                      f'({servo_min+margin:.3f}~{servo_max-margin:.3f}) 밖 — 자동 반영 거부.'
                      f' 수동 확인 요망 (trim={self.trim:+.4f}, gain={gain}, offset_old={offset_old})')
                return

            ts = time.strftime('%Y-%m-%d')
            new_text, n = re.subn(
                r'steering_angle_to_servo_offset:\s*[-\d.]+',
                f'steering_angle_to_servo_offset: {offset_new:.4f}',
                text, count=1)
            if n != 1:
                print('offset 치환 실패 — 수동 반영 필요'); return

            comment = (f'    # {ts} steer_trim.py 자동측정: {offset_old:.4f} → '
                       f'{offset_new:.4f} (trim={self.trim:+.4f}rad, iter={self.iter_n})\n')
            new_text = re.sub(
                r'(\n\s*steering_angle_to_servo_offset:)',
                '\n' + comment.rstrip('\n') + r'\1', new_text, count=1)

            open(path, 'w').write(new_text)
            print(f'저장: {path}')
            print(f'  offset {offset_old:.4f} → {offset_new:.4f}')
            print('  ★ ackermann_to_vesc 는 시작 시 파라미터를 한 번만 읽음 — '
                  '적용하려면 스택(race.sh) 재시작 필요')

        # ── 화면 ──────────────────────────────────────────────────────
        def _draw(self):
            now = time.time()
            bar = '═' * BOX_W
            L = ['\033[2J\033[H', '╔' + bar + '╗']
            A = L.append
            A(_row(f'  조향 트림 자동 캘리브레이션{" "*13}[{STATE_NAME[self.state]:^8}]'))
            A('╠' + bar + '╣')
            A(_row(f'  회차 {self.iter_n:2d}/{self.max_iters}   trim {self.trim:+.4f} rad'))
            A(_row(f'  v {self.v:+5.2f} m/s   leg_dist {self.leg_dist:4.2f}/{self.test_dist:.1f}m'))
            if self.history:
                l, tr, k, s = self.history[-1]
                A(_row(f'  마지막[{l}] κ={k:+.4f} 1/m  (허용 ±{self.kappa_tol})'))
            A('╠' + bar + '╣')
            A(_row(f'  전방최소 {self.front_min:5.2f}m (한계{self.front_safety_m})'))
            rear_s = f'{self.rear_min:5.2f}m' if self._scan_has_rear else '  라이다 FOV 밖'
            A(_row(f'  후방최소 {rear_s} (한계{self.rear_safety_m})'))
            for k, label in (('odom', 'odom'), ('imu', 'imu'), ('scan', 'scan')):
                age = now - self._seen[k]
                A(_row(f'  {label:5s} {"OK" if age < 1.0 else f"{age:.1f}s 전"}'))
            A('╠' + bar + '╣')
            A(_row('  q: 즉시중단(저장 안함)   s: 이번 다리 후 저장·종료'))
            if self._msg:
                A(_row(f'  {self._msg}'))
            A('╚' + bar + '╝')
            print('\n'.join(L), flush=True)

    rclpy.init()
    node = SteerTrim()
    settings = termios.tcgetattr(sys.stdin)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    tty.setraw(sys.stdin.fileno())
    try:
        while rclpy.ok() and not node._stop_event.is_set():
            r, _, _ = select.select([sys.stdin], [], [], 0.1)
            if r:
                node.on_key(sys.stdin.read(1))
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()
        print()


def main():
    _run_node()


if __name__ == '__main__':
    main()
