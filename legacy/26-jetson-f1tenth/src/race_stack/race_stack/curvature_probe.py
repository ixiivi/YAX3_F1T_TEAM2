#!/usr/bin/env python3
"""curvature_probe.py — 속도별 "풀조향 달성곡률" 캘리브레이션 리그.

★ slip_logger.py(언더스티어 구배 K_us 회귀)와 완전히 별개의 도구다.
  그 파일과 race_script.yaml 의 slip: 섹션은 건드리지 않는다 — 다른 팀원이
  그 리그를 계속 쓴다. 여기는 새 토픽 이름(/curvprobe/...), 새 설정 섹션
  (curv_probe:), 새 로그 폴더(logs/curv_tests/)를 따로 쓴다.

★ 목적
  gen_waypoints_kappa.py 는 조향각 기하한계만으로 KAPPA_MAX 를 잡는다
  (무슬립 가정). 실제로는 그 곡률을 낼 만큼 빠르면 타이어가 미끄러져
  계획보다 넓게 밀려난다(2026-08-06 q1_max=2 실주행에서 실측). 그래서
  "이 곡률에서 슬립 없이 낼 수 있는 최대속도" v_max(κ) 를 실측하려 한다.

★ 왜 반경을 여러 개 안 재고 속도만 스윕하는가
  조향을 풀락(좌 또는 우 끝까지)으로 고정하고 속도만 올리면서, 매 순간
  IMU 요레이트(ψ̇, 실측)와 휠 오도메트리 속도(v)로
      달성곡률 κ_achieved = ψ̇ / v
  를 직접 계산한다. 속도 v 에서 풀락(최대 요구 a_y)을 슬립 없이 버텼다면,
  그보다 작은 곡률(더 작은 a_y=v²κ 요구)은 같은 속도에서 당연히 더 여유
  있게 버틴다. 그래서 "속도 → 풀락 달성곡률" 곡선 하나만 실측하면, 뒤집어서
  전체 곡률 범위(0~KAPPA_MAX)의 v_max(κ) 를 얻을 수 있다 — 반경×속도 그리드를
  다 채울 필요가 없다.

★ 조작 방식 (2026-08-07 개편 — 조이스틱 수동 대신 프로그램이 직접 명령)
  사람이 스틱으로 조향/속도를 들고 있으면 "정말 풀락이었는지", "속도가 정말
  일정했는지"가 매번 달라져 측정이 안 깨끗했다(2026-08-07 1차 시도에서 저속
  구간이 고속 구간보다 오히려 달성곡률이 낮게 나오는 이상한 결과 확인).
  그래서 이 노드가 keyboard_teleop.py 와 같은 방식으로 /teleop(mux 최우선)에
  직접 명령을 낸다 — 조향은 항상 풀락 고정, 속도는 0.5㎧부터 0.25㎧씩 계단식
  으로 프로그램이 직접 램프한다. 사람은 스페이스바로 "다음 단계로" 신호만 준다.

  키보드 (터미널에 포커스 있어야 함):
    SPACE   다음 속도 단계로 (target_speed += speed_step, 부드럽게 램프)
    q       정지 → 저장 → 종료 (Ctrl+C 도 동일)

  흐름: 노드 시작 → 3초 정지 상태로 바이어스 자동측정 → 조향 풀락 고정 +
        speed_start 로 램프 시작 → min_hold_sec 이상 그 속도 유지하며 계속
        기록 → SPACE 누르면 다음 단계(+speed_step)로 램프 → ... → q 로 종료.
  방향은 curv_probe.steer_sign (+1=좌, -1=우) 로 고정 — 매 세션 한 방향만.

★ 사용법
  ./race.sh curvprobe                                    # 실차 리그 (foreground, 키보드 필요)
  ros2 run race_stack curvature_probe -- --analyze        # 수집 데이터 분석만
    (ROS 노드 안 띄우고 logs/curv_tests/*.csv 를 모아 v_max(κ) 표를 뽑는다)
"""

import argparse
import csv
import math
import os
import sys
import time
import unicodedata

import numpy as np

from race_stack.paths import repo_root

_REPO = repo_root()
_OUT = os.path.join(_REPO, 'logs', 'curv_tests')

BOX_W = 64


def _w(s: str) -> int:
    return sum(2 if unicodedata.east_asian_width(c) in 'WF' else 1 for c in s)


def _row(text: str) -> str:
    while _w(text) > BOX_W:
        text = text[:-1]
    return '║' + text + ' ' * (BOX_W - _w(text)) + '║'


# ══════════════════════════════════════════════════════════════════════════
# 오프라인 분석 (ROS 불필요) — 여러 세션 CSV를 모아 v_max(κ) 계산
# ══════════════════════════════════════════════════════════════════════════
def _load_steady_samples():
    """logs/curv_tests/*.csv 전부에서 steady==1 행만 모은다.
    반환: [(v, kappa_achieved), ...]"""
    out = []
    if not os.path.isdir(_OUT):
        return out
    for fn in sorted(os.listdir(_OUT)):
        if not fn.endswith('.csv'):
            continue
        path = os.path.join(_OUT, fn)
        try:
            with open(path) as f:
                rd = csv.reader(f)
                next(rd)  # 헤더 주석행
                header = next(rd)
                iv = header.index('v')
                ik = header.index('kappa')
                ist = header.index('steady')
                for row in rd:
                    if len(row) <= max(iv, ik, ist) or row[ist] != '1':
                        continue
                    out.append((float(row[iv]), float(row[ik])))
        except Exception:
            continue
    return out


def analyze(kappa_max: float = 1.1406, speed_bin: float = 0.25,
            grid_step: float = 0.1, save: bool = True):
    """수집된 데이터를 (방향, 속도구간)별로 묶어 median κ_achieved 를 구하고,
    그걸 뒤집어 v_max(κ) 표를 만든다. 콘솔에 출력하고, save 면 CSV로도 저장."""
    samples = _load_steady_samples()
    print(f'steady 샘플: {len(samples)}개')
    if len(samples) < 5:
        print('표본이 너무 적음 (5개 미만) — 리그를 더 돌려야 함')
        return

    for direction, sign in (('좌회전(κ>0)', 1), ('우회전(κ<0)', -1)):
        pts = [(v, abs(k)) for v, k in samples if (k > 0) == (sign > 0) and v > 0.2]
        print(f'\n=== {direction} — {len(pts)}개 표본 ===')
        if len(pts) < 5:
            print('  표본 부족 — 이 방향은 더 재야 함')
            continue

        # 속도 구간별 median achieved kappa
        vs = np.array([p[0] for p in pts])
        ks = np.array([p[1] for p in pts])
        vmin, vmax = vs.min(), vs.max()
        bins = np.arange(math.floor(vmin / speed_bin) * speed_bin,
                          vmax + speed_bin, speed_bin)
        curve_v, curve_k = [], []
        print('  속도구간(m/s)   n     median|κ_achieved|   κ_max대비')
        for b in bins:
            mask = (vs >= b) & (vs < b + speed_bin)
            if mask.sum() < 2:
                continue
            med = float(np.median(ks[mask]))
            curve_v.append(b + speed_bin / 2)
            curve_k.append(med)
            print(f'    {b:4.2f}~{b+speed_bin:4.2f}   {mask.sum():3d}   '
                  f'{med:6.3f} 1/m         {med/kappa_max*100:5.1f}%')

        if len(curve_v) < 2:
            print('  구간이 2개 미만이라 v_max(κ) 역산 불가')
            continue

        # 속도 오름차순 정렬, kappa_achieved 는 보통 내림차순(슬립 시작하면 감소)
        order = np.argsort(curve_v)
        cv = np.array(curve_v)[order]
        ck = np.array(curve_k)[order]

        # v_max(κ): 목표 κ 이상을 낼 수 있는 가장 큰 속도를 선형보간으로 역산
        kappa_grid = np.arange(grid_step, min(kappa_max, ck.max()) + 1e-6, grid_step)
        print('\n  → v_max(κ) 역산 (선형보간):')
        rows_out = []
        for kg in kappa_grid:
            # ck 가 v 증가에 따라 대체로 감소한다고 가정하고, ck>=kg 를 만족하는
            # 마지막 v(가장 큰 v)를 찾는다.
            ok = np.where(ck >= kg)[0]
            if len(ok) == 0:
                continue
            i_last = ok[-1]
            if i_last + 1 < len(cv) and ck[i_last + 1] < kg:
                # 그 다음 구간과 보간
                v0, v1 = cv[i_last], cv[i_last + 1]
                k0, k1 = ck[i_last], ck[i_last + 1]
                v_max = v0 + (kg - k0) * (v1 - v0) / (k1 - k0)
            else:
                v_max = cv[i_last]
            print(f'    κ={kg:.2f}  v_max≈{v_max:.2f} m/s')
            rows_out.append((sign, kg, v_max))

        if save and rows_out:
            os.makedirs(_OUT, exist_ok=True)
            out_path = os.path.join(_OUT, 'v_max_by_kappa.csv')
            mode = 'a' if os.path.exists(out_path) else 'w'
            with open(out_path, mode, newline='') as f:
                w = csv.writer(f)
                if mode == 'w':
                    w.writerow(['direction_sign', 'kappa', 'v_max',
                                'generated_at'])
                ts = time.strftime('%Y-%m-%d %H:%M:%S')
                for s, kg, vmax in rows_out:
                    w.writerow([s, f'{kg:.3f}', f'{vmax:.3f}', ts])
            print(f'\n  저장: {out_path}')


# ══════════════════════════════════════════════════════════════════════════
# ROS 노드 (실차 수집)
# ══════════════════════════════════════════════════════════════════════════
def _run_node():
    # ros2 run 이라도 stdout 이 파이프로 캡처될 수 있다 — 터미널이 아니면 파이썬이
    # 완전버퍼링으로 전환해 화면 패널(_draw)이 한참 늦게 나타난다(2026-08-07에
    # 실제로 이 버그로 아무것도 안 뜨는 것처럼 보였다). line_buffering 강제.
    try:
        sys.stdout.reconfigure(line_buffering=True)
    except Exception:
        pass

    import select
    import termios
    import threading
    import tty
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu
    from ackermann_msgs.msg import AckermannDriveStamped

    BIAS, RUN, STOP = 0, 1, 2
    STATE_NAME = {BIAS: '바이어스 측정', RUN: '측정중', STOP: '정지중'}

    class CurvatureProbe(Node):
        def __init__(self):
            super().__init__('curvature_probe')
            d = self.declare_parameter
            d('imu_topic', '/sensors/imu/raw')
            d('odom_topic', '/odom')
            d('wheelbase', 0.33)
            d('gyro_units', 'deg')
            d('lpf_hz', 8.0)
            d('kappa_max', 1.1406)
            d('mu', 0.7)
            d('full_lock_steer', 0.65)   # 실제 기구 락(0.361/0.558)보다 확실히 큼
            d('steer_sign', 1.0)         # +1=좌, -1=우 (세션 하나는 한 방향만)
            d('speed_start', 0.5)
            d('speed_step', 0.25)
            d('speed_max', 3.0)
            d('accel_limit', 0.4)        # m/s^2 — 프로그램이 직접 램프하는 가감속
            d('bias_sec', 3.0)
            d('min_hold_sec', 3.0)       # 목표속도 도달 후 이만큼 유지돼야 "다음단계 준비완료"
            d('ss_hold_sec', 0.5)
            d('ss_v_tol', 0.15)
            d('ss_wz_tol', 0.20)
            d('min_speed', 0.3)

            p = lambda n: self.get_parameter(n).value
            self.L = float(p('wheelbase'))
            self.to_rad = (math.pi / 180.0
                           if str(p('gyro_units')).lower().startswith('deg') else 1.0)
            self.lpf_hz = float(p('lpf_hz'))
            self.kappa_max = float(p('kappa_max'))
            self.mu = float(p('mu'))
            self.full_lock = float(p('full_lock_steer'))
            self.steer_sign = 1.0 if float(p('steer_sign')) >= 0 else -1.0
            self.speed_start = float(p('speed_start'))
            self.speed_step = float(p('speed_step'))
            self.speed_max = float(p('speed_max'))
            self.accel_limit = float(p('accel_limit'))
            self.bias_sec = float(p('bias_sec'))
            self.min_hold_sec = float(p('min_hold_sec'))
            self.min_speed = float(p('min_speed'))

            self.state = BIAS
            self.t_state = time.time()
            self.v = 0.0
            self.last_t = None
            self.wz_f = 0.0
            self.bias_wz = 0.0
            self._warm = []
            self.rows = []
            self._ss_buf = []
            self._msg = '정지 상태 유지 — 바이어스 측정 중...'
            self._seen = {'odom': 0.0, 'imu': 0.0}
            self._live_kappa = 0.0

            self.steer_cmd = 0.0
            self.speed_target = 0.0
            self.speed_cmd = 0.0
            self._t_reached = None       # 목표속도 도달 시각 (None=아직)
            self._stop_event = threading.Event()

            os.makedirs(_OUT, exist_ok=True)

            self.pub = self.create_publisher(AckermannDriveStamped, '/teleop', 10)
            self.create_subscription(Odometry, p('odom_topic'), self._odom_cb, 20)
            self.create_subscription(Imu, p('imu_topic'), self._imu_cb,
                                     qos_profile_sensor_data)
            self.create_timer(0.05, self._control_tick)
            self.create_timer(0.3, self._draw)

            self.get_logger().info(
                f'CurvatureProbe(계단식) ready — SPACE:다음단계  q:정지·저장·종료  '
                f'풀락={self.steer_sign*math.degrees(self.full_lock):+.1f}°  '
                f'{self.speed_start}~{self.speed_max}㎧ step {self.speed_step}')

        # ── 입력 ──────────────────────────────────────────────────────
        def _odom_cb(self, msg: Odometry):
            self._seen['odom'] = time.time()
            self.v = float(msg.twist.twist.linear.x)

        # ── 키보드 (메인 스레드에서 직접 호출) ───────────────────────────
        def on_key(self, key):
            if key == ' ':
                if self.state != RUN:
                    return
                nxt = self.speed_target + self.speed_step
                if nxt > self.speed_max + 1e-6:
                    self._msg = f'이미 최고속도({self.speed_max}㎧) — q로 종료'
                else:
                    self.speed_target = nxt
                    self._t_reached = None
                    self._msg = f'다음 단계 → {self.speed_target:.2f}㎧ 로 램프 중'
            elif key in ('q', '\x03'):
                if self.state != STOP:
                    self.state = STOP
                    self.speed_target = 0.0
                    self._msg = '정지 중 — 속도 0 도달 후 자동 저장'

        # ── 제어 루프 (20Hz, /teleop 직접 발행 — mux 최우선순위) ─────────
        def _control_tick(self):
            dt = 0.05
            if self.state == BIAS:
                self.steer_cmd = 0.0
                self.speed_cmd = 0.0
                if time.time() - self.t_state >= self.bias_sec:
                    self.bias_wz = float(np.mean(self._warm)) if self._warm else 0.0
                    self.state = RUN
                    self.t_state = time.time()
                    self.speed_target = self.speed_start
                    self._msg = f'바이어스 확정 — {self.speed_target:.2f}㎧ 로 램프 시작'
            else:
                self.steer_cmd = self.full_lock * self.steer_sign
                diff = self.speed_target - self.speed_cmd
                step = self.accel_limit * dt
                if abs(diff) <= step:
                    self.speed_cmd = self.speed_target
                else:
                    self.speed_cmd += step if diff > 0 else -step

                if self.state == RUN:
                    if abs(self.speed_cmd - self.speed_target) < 0.02:
                        if self._t_reached is None:
                            self._t_reached = time.time()
                    else:
                        self._t_reached = None
                elif self.state == STOP and abs(self.speed_cmd) < 0.02:
                    self.speed_cmd = 0.0
                    self._publish(0.0, 0.0)
                    self._save()
                    self._stop_event.set()
                    return

            self._publish(self.steer_cmd, self.speed_cmd)

        def _publish(self, steer, speed):
            msg = AckermannDriveStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.drive.speed = float(speed)
            msg.drive.steering_angle = float(steer)
            self.pub.publish(msg)

        # ── IMU 처리 ──────────────────────────────────────────────────
        def _imu_cb(self, msg: Imu):
            self._seen['imu'] = time.time()
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if t <= 0.0:
                t = self.get_clock().now().nanoseconds * 1e-9
            dt = (t - self.last_t) if self.last_t else 0.02
            self.last_t = t
            if not (1e-4 < dt < 0.5):
                dt = 0.02

            # 축 매핑: 전방(+x_car)=-y_imu, 좌측(+y_car)=+x_imu, 위=+z
            # (slip_logger.py 와 동일 — imu_axis_check.py 실측 기준)
            wz = +msg.angular_velocity.z * self.to_rad

            if self.state == BIAS:
                self._warm.append(wz)
                return

            wz -= self.bias_wz
            a_lpf = dt / (dt + 1.0 / (2 * math.pi * max(self.lpf_hz, 0.1)))
            self.wz_f += a_lpf * (wz - self.wz_f)

            v = self.v
            kappa = self.wz_f / v if abs(v) > self.min_speed else 0.0
            self._live_kappa = kappa
            wz_kin = v * math.tan(self.steer_cmd) / self.L

            steady = self._steady(t, v, self.wz_f)
            self.rows.append((t, v, self.steer_cmd, self.wz_f, wz_kin, kappa, int(steady)))

        def _steady(self, t, v, wz):
            if abs(v) < self.min_speed:
                self._ss_buf.clear(); return False
            self._ss_buf.append((t, v, wz))
            hold = float(self.get_parameter('ss_hold_sec').value)
            while self._ss_buf and t - self._ss_buf[0][0] > hold:
                self._ss_buf.pop(0)
            if not self._ss_buf or t - self._ss_buf[0][0] < hold * 0.9:
                return False
            a = np.asarray(self._ss_buf)
            v_tol = float(self.get_parameter('ss_v_tol').value)
            w_tol = float(self.get_parameter('ss_wz_tol').value)
            if a[:, 1].ptp() > v_tol or a[:, 2].ptp() > w_tol:
                return False
            # 램프가 끝나 목표속도에 실제로 도달해 있어야 진짜 정상원이다
            return abs(self.speed_cmd - self.speed_target) < 0.03

        # ── 저장 ──────────────────────────────────────────────────────
        def _save(self):
            if len(self.rows) < 20:
                self._msg = '표본이 너무 적어 버립니다 (20개 미만)'
                print(f'\n{self._msg}', flush=True)
                return
            ts = time.strftime('%Y%m%d_%H%M%S')
            path = os.path.join(_OUT, f'{ts}_curvprobe.csv')
            t0 = self.rows[0][0]
            n_steady = sum(r[6] for r in self.rows)
            with open(path, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['# curvature_probe', f'rows={len(self.rows)}',
                            f'steady={n_steady}'])
                w.writerow(['t', 'v', 'delta', 'wz', 'wz_kin', 'kappa', 'steady'])
                for row in self.rows:
                    w.writerow([f'{row[0]-t0:.4f}'] + [f'{x:.5f}' for x in row[1:6]]
                               + [row[6]])
            self._msg = f'저장: {os.path.basename(path)} — steady {n_steady}점'
            print(f'\n{self._msg}', flush=True)

        # ── 화면 ──────────────────────────────────────────────────────
        def _draw(self):
            now = time.time()
            bar = '═' * BOX_W
            L = ['\033[2J\033[H', '╔' + bar + '╗']
            A = L.append
            A(_row(f'  곡률-속도 계단 캘리브레이션{" "*17}[{STATE_NAME[self.state]:^10}]'))
            A('╠' + bar + '╣')
            dirlabel = '좌' if self.steer_sign > 0 else '우'
            A(_row(f'  방향 {dirlabel}   목표 {self.speed_target:5.2f}  '
                   f'명령 {self.speed_cmd:5.2f}  실측 v {self.v:5.2f} m/s'))
            A(_row(f'  달성곡률 κ  {self._live_kappa:+6.3f} 1/m '
                   f'(κ_max={self.kappa_max:.3f})'))
            if abs(self.v) > 0.3:
                pct = abs(self._live_kappa) / self.kappa_max * 100
                A(_row(f'  κ_max 대비  {pct:5.1f}%'
                       f'{"  ⚠ 슬립 의심" if pct < 70 else ""}'))
            if self.state == RUN and self._t_reached:
                held = now - self._t_reached
                ready = held >= self.min_hold_sec
                A(_row(f'  안정화 {held:4.1f}/{self.min_hold_sec:.0f}s'
                       f'{"  — SPACE로 다음 단계" if ready else ""}'))
            A('╠' + bar + '╣')
            for k, label in (('odom', 'odom'), ('imu', 'imu')):
                age = now - self._seen[k]
                A(_row(f'  {label:5s}  {"OK" if age < 1.0 else f"{age:.1f}s 전"}'))
            A('╠' + bar + '╣')
            A(_row('  SPACE: 다음 속도단계   q: 정지+저장+종료'))
            if self._msg:
                A(_row(f'  {self._msg}'))
            A(_row(f'  누적 표본 {len(self.rows)}개  (logs/curv_tests/, 분석: --analyze)'))
            A('╚' + bar + '╝')
            print('\n'.join(L), flush=True)

    rclpy.init()
    node = CurvatureProbe()
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
    if '--analyze' in sys.argv:
        p = argparse.ArgumentParser()
        p.add_argument('--analyze', action='store_true')
        p.add_argument('--kappa-max', type=float, default=1.1406)
        args = p.parse_args()
        analyze(kappa_max=args.kappa_max)
        return
    _run_node()


if __name__ == '__main__':
    main()
