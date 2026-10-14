#!/usr/bin/env python3
"""slip_logger — 타이어 슬립 데이터 수집 리그 (조이스틱으로 조작).

★ 왜 수동 주행인가
  슬립을 보려면 횡가속도가 마찰한계에 가까워야 하고, 우리 트랙 최급코너
  (κ 1.14) 기준으로 그건 **2.0 m/s 부터**다 (a_y 4.6 m/s² = μ0.7 한계의 66%).
  그런데 그 속도로 자율주행하려면 컨트롤러·위치추정·레이싱라인이 먼저 멀쩡해야
  한다. 지금은 셋 다 미완이다.

  하지만 수집 자체는 자율주행이 필요 없다. 정상원은 조이스틱으로 낼 수 있고
  필요한 건 IMU + 휠 오도메트리 + 서보 명령뿐이다. AMCL 도 Stanley 도 레이싱
  라인도 안 쓴다. 그래서 지금 막힌 것들과 **완전히 분리해서** 진행할 수 있다.

★ 무엇을 주된 지표로 삼는가 — β 적분이 아니라 언더스티어 구배
  요청받은 관측기 β̇ = a_y/v_x − ψ̇ 는 수식이 맞지만 우리 하드웨어에서 불리하다:
    · IMU 가 50 Hz 다 (vesc_driver.cpp:107 의 20ms 폴링 타이머 탓, 센서 한계 아님)
    · 타임스탬프가 샘플 시각이 아니라 **패킷 파싱 시각**이다 (같은 파일 :211 의
      `now()`). 지터가 그대로 적분에 쌓인다

  그래서 적분이 필요 없는 정상상태 관계를 주 산출물로 삼는다:

        δ  =  L·ψ̇/v  +  K_us · a_y
              └ 기구학 ┘   └ 타이어가 먹는 몫 ┘

  (δ − L·ψ̇/v) 를 a_y 로 회귀하면 기울기가 **언더스티어 구배 K_us** 다.
  표본이 독립이라 드리프트가 없다. β 도 함께 기록하지만 참고용이다.

★ 조작 (조이스틱)
      ↑ ↓   목표 속도 단계 변경
      ← →   목표 정상원 반경 변경
      B     테스트 시작 / 종료

  ★ 시험법은 **정조향 램프**(constant-steer ramp) 다. 조향 스틱을 한 자리에
    고정하고 속도만 천천히 올린다.

    바닥에 원을 그릴 필요가 없다. 회귀에 쓰는 (δ − L·ψ̇/v) vs a_y 는 δ·ψ̇·v 를
    **전부 측정**하므로, 차가 실제로 그린 원이 1.0m 든 1.23m 든 상관없다.
    반경은 목표가 아니라 결과다 — ←→ 로 고르는 "목표 반경" 은 그에 대응하는
    조향각을 안내하기 위한 것일 뿐이다.

    정반경(원을 따라가며 핸들을 계속 조정)보다 쉽고, 정상상태 판정의
    "δ 변동 < 2°" 가 자동으로 만족된다 — 스틱을 안 움직였으니까.
    슬립이 생기면 차가 저절로 바깥으로 벌어지는데, 그 벌어짐이 곧 슬립량이라
    화면의 "실제 반경" 이 커지는 것으로 바로 보인다.

  절차: ←→ 로 조향 목표를 고른다 → 빈 자리에 놓고 **정지** → B
        → 3초 바이어스 자동측정 → "출발하세요!"
        → 화면의 δ 가 목표와 맞도록 스틱을 잡고 **그대로 고정**
        → 조향은 그대로 둔 채 **속도만 천천히** 올린다 (0.3 ㎧² 이하.
           1.0→2.5 ㎧ 를 10초 정도. 더 빠르면 정상상태 판정에서 걸러진다)
        → 공간이 부족해지거나 미끄러지면 정지 후 B → 자동 저장
        ↑↓ 는 램프 목표 속도(안내용)일 뿐이다. 측정 칸은 **실측 속도로 자동
        분류**되므로, 한 번의 램프가 여러 속도 칸을 한꺼번에 채운다.

★ 축·단위 (2026-08-02 imu_axis_check.py 실측)
      차량 전방 = IMU -y,  차량 좌측 = IMU +x,  위 = +z
      자이로 deg/s,  가속도 g   ← REP-145 아님. 드라이버가 변환 없이 대입한다
  가속도까지 쓰는 건 이 노드가 처음이라 x/y 스왑이 여기서 처음 문제가 된다
  (yaw 만 쓰던 imu_odom 은 회전축이 z 라 스왑이 무해했다).

★ 남은 오차 요인 (알고 쓸 것)
  · 레버암: `lever_x` 를 재서 넣지 않으면 a_y 에 ψ̈·r 항이 남는다. 기본 0.
  · 모터 진동: 1.5 m/s 에서 모터 회전이 이미 50 Hz 대라 **샘플링 주파수보다
    높다**. 에일리어싱이라 저역통과로 못 지운다. IMU 100 Hz 화가 근본 대책.

★ 이 노드는 차를 조종하지 않는다 — /drive 도 서보도 발행하지 않는다.
  주행은 전적으로 조이스틱(다른 터미널의 ./race.sh joy)이 한다.
"""
import csv
import json
import unicodedata
import math
import os
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Imu, Joy
from std_msgs.msg import Float64

# 슬립 CSV 도 팀 공유 대상이라 **git repo** 안에 쌓여야 한다.
# 경로 규칙은 race_stack.paths 하나로 유지한다 — 두 군데서 따로 정하면 로그가
# 두 곳으로 갈린다 (2026-08-06 에 실제로 갈려 있었다).
from race_stack.paths import repo_root

_REPO = repo_root()
_OUT = os.path.join(_REPO, 'logs', 'slip_tests')

BOX_W = 64          # 테두리 안쪽 표시 폭(칸). 한글은 2칸이라 len() 으로는 못 센다


def _w(s: str) -> int:
    """터미널 표시 폭. 한중일 문자와 ㎧·° 같은 기호는 2칸을 차지한다."""
    return sum(2 if unicodedata.east_asian_width(c) in 'WF' else 1 for c in s)


def _row(text: str) -> str:
    """내용을 BOX_W 칸에 맞춰 채운 한 줄. 넘치면 잘라서 테두리를 지킨다."""
    while _w(text) > BOX_W:
        text = text[:-1]
    return '║' + text + ' ' * (BOX_W - _w(text)) + '║'


IDLE, BIAS, REC = 0, 1, 2
STATE_NAME = {IDLE: '대기', BIAS: '바이어스 측정', REC: '측정중'}


class SlipLogger(Node):
    def __init__(self):
        super().__init__('slip_logger')
        d = self.declare_parameter
        d('imu_topic', '/sensors/imu/raw')
        d('odom_topic', '/odom')          # 병진 속도는 휠 오도메트리가 정확하다
        d('servo_topic', '/commands/servo/position')
        d('joy_topic', '/joy')
        # ★ 시험할 조향각을 joystick_teleop 에 상한으로 넘긴다. 그러면 L스틱을
        #   끝까지 밀었을 때 차량 최대각이 아니라 **이 각도**가 나온다.
        #   사람이 화면을 보며 손으로 맞추는 것보다 재현성이 비교가 안 된다.
        #   teleop 쪽은 max_steer 를 넘는 값은 무시하고, 1초 안 오면 원래
        #   상한으로 되돌아간다 (이 노드가 죽어도 조종은 정상).
        d('steer_limit_topic', '/slip/steer_limit')
        # ↑↓ 로 고른 속도를 RT 최고속도로 넘긴다. 트리거 전체 행정이 그 속도에
        # 배분되므로 같은 손놀림이 훨씬 정밀해진다 — 2.5㎧ 상한에서는 트리거
        # 1% 가 0.025㎧ 라 정상상태 허용치(0.15㎧)를 지키기가 어렵다.
        d('speed_limit_topic', '/slip/speed_limit')
        d('wheelbase', 0.33)
        d('servo_gain', -0.761)           # config/vesc.yaml 과 같아야 한다
        d('servo_offset', 0.425)
        d('gyro_units', 'deg')
        d('accel_units', 'g')
        d('lever_x', 0.0)                 # IMU 가 무게중심보다 앞이면 + (m)
        d('lpf_hz', 8.0)
        d('mu', 0.7)                      # 마찰계수 추정 — 화면 표시용
        # 목표 격자
        d('speeds', [1.0, 1.5, 2.0, 2.5, 3.0])
        d('radii', [1.0, 1.5, 2.0, 3.0])
        d('target_runs', 3)               # 설정 하나당 목표 측정 횟수
        d('bias_sec', 3.0)
        # 조이스틱 (F710: A=0 B=1 X=2 Y=3, D-pad = axes 6(좌우) 7(상하))
        d('btn_toggle', 1)
        d('axis_dpad_x', 6)          # F710 실측: 6=Hat0X, 7=Hat0Y, 버튼 1=B
        d('axis_dpad_y', 7)
        # ★ 부호는 드라이버마다 다르다. 리눅스 규약은 Hat0Y 위=-1 인데 ROS
        #   joy_node 가 뒤집는지가 확실치 않다. 화면의 "D-pad" 값을 보고
        #   반대로 움직이면 이 값을 true 로 바꾸면 된다.
        d('dpad_x_invert', False)
        d('dpad_y_invert', False)
        # 정상상태 판정
        d('ss_hold_sec', 0.5)
        d('ss_v_tol', 0.15)
        d('ss_delta_tol', 0.035)
        d('ss_wz_tol', 0.20)
        d('ss_min_ay', 0.8)
        d('min_speed', 0.5)

        p = lambda n: self.get_parameter(n).value
        self.L = float(p('wheelbase'))
        self.sg, self.so = float(p('servo_gain')), float(p('servo_offset'))
        self.to_rad = (math.pi / 180.0
                       if str(p('gyro_units')).lower().startswith('deg') else 1.0)
        self.to_ms2 = 9.80665 if str(p('accel_units')).lower() == 'g' else 1.0
        self.lever_x = float(p('lever_x'))
        self.lpf_hz = float(p('lpf_hz'))
        self.mu = float(p('mu'))
        self.min_speed = float(p('min_speed'))
        self.speeds = [float(x) for x in p('speeds')]
        self.radii = [float(x) for x in p('radii')]
        self.target_runs = int(p('target_runs'))
        self.iv, self.ir = 0, 0           # 선택 인덱스

        self.state = IDLE
        self.t_state = 0.0
        self.v = 0.0
        self.delta = 0.0
        self.last_t = None
        self.wz_f = self.ay_f = self.ax_f = 0.0
        self.wz_prev = 0.0
        self.beta = 0.0
        self.bias = {'wz': 0.0, 'ax': 0.0, 'ay': 0.0}
        self._warm = []
        self.rows = []
        self.ss = []
        self._ss_buf = []
        self._prev_joy = None
        self._msg = ''
        # 속도 대역별 (실제 ψ̇)/(기구학 ψ̇). 1.00 에서 떨어지는 대역이 슬립 시작점.
        self._band = {}
        # 연결 상태 — teleop 로그를 화면에서 뺐으므로(slip.launch.py 의 quiet)
        # "왜 차가 안 나가지" 를 여기서 판단할 수 있어야 한다.
        self._seen = {'joy': 0.0, 'odom': 0.0, 'imu': 0.0, 'teleop': 0.0}
        self._dpad = (0.0, 0.0, False)      # 화면 진단용 (부호 확인)

        os.makedirs(_OUT, exist_ok=True)
        self.counts = self._load_counts()

        self.lim_pub = self.create_publisher(Float64, p('steer_limit_topic'), 10)
        self.spd_pub = self.create_publisher(Float64, p('speed_limit_topic'), 10)
        self.create_timer(0.2, self._pub_limit)
        self.create_subscription(Odometry, p('odom_topic'), self._odom_cb, 20)
        self.create_subscription(Float64, p('servo_topic'), self._servo_cb, 20)
        self.create_subscription(Joy, p('joy_topic'), self._joy_cb, 10)
        self.create_subscription(AckermannDriveStamped, '/teleop',
                                 lambda m: self._seen.__setitem__('teleop', time.time()), 10)
        self.create_subscription(Imu, p('imu_topic'), self._imu_cb,
                                 qos_profile_sensor_data)
        self.create_timer(0.3, self._draw)

    def _pub_limit(self):
        """고른 시험 조향각을 조향 상한으로 계속 알린다 (5Hz).

        주기 발행이라 이 노드를 끄면 teleop 이 타임아웃으로 원래 상한을 되찾는다.
        """
        r = self.radii[self.ir]
        self.lim_pub.publish(Float64(data=float(math.atan(self.L / r))))
        self.spd_pub.publish(Float64(data=float(self.speeds[self.iv])))

    # ── 측정 횟수 누적 (재시작해도 이어진다) ───────────────────────────
    def _cfile(self):
        return os.path.join(_OUT, 'counts.json')

    def _load_counts(self):
        try:
            return json.load(open(self._cfile()))
        except Exception:
            return {}

    def _vbin(self, v):
        """실측 속도를 속도 격자에 붙인다.

        정반경 램프는 속도가 연속으로 오르므로, 사람이 고른 목표가 아니라
        **그 표본이 실제로 얼마에서 찍혔는지**로 분류해야 한다. 그래야 한 번의
        램프가 여러 칸을 채운다. 격자 간격의 절반을 넘게 벗어나면 버린다.
        """
        if not self.speeds:
            return None
        i = int(np.argmin([abs(v - s) for s in self.speeds]))
        half = (self.speeds[1] - self.speeds[0]) / 2 if len(self.speeds) > 1 else 0.25
        return self.speeds[i] if abs(v - self.speeds[i]) <= half else None

    def _key(self, v=None, r=None):
        """측정 칸 식별자. 반경은 소수 2자리로 적는다.

        ★ .1f 로 적으면 반경이 뭉개져 **다른 설정이 같은 칸에 쌓인다**
          (2026-08-06: 0.65m→"0.7", 0.74m→"0.7" 로 충돌. 조향각으로는
          26.9도와 24.0도라 완전히 다른 시험인데 counts.json 과 CSV 파일명이
          겹쳤다). 반경 격자가 촘촘해질수록 반드시 터지는 문제라 자리수를 늘린다.
        """
        v = self.speeds[self.iv] if v is None else v
        r = self.radii[self.ir] if r is None else r
        return f'{v:.1f}_{r:.2f}'

    # ── 입력 ───────────────────────────────────────────────────────────
    def _odom_cb(self, msg: Odometry):
        self._seen['odom'] = time.time()
        self.v = float(msg.twist.twist.linear.x)

    def _servo_cb(self, msg: Float64):
        self.delta = (float(msg.data) - self.so) / self.sg

    def _joy_cb(self, msg: Joy):
        self._seen['joy'] = time.time()
        p = lambda n: self.get_parameter(n).value
        bt, ax, ay = int(p('btn_toggle')), int(p('axis_dpad_x')), int(p('axis_dpad_y'))
        b = bool(msg.buttons[bt]) if len(msg.buttons) > bt else False
        dx = float(msg.axes[ax]) if len(msg.axes) > ax else 0.0
        dy = float(msg.axes[ay]) if len(msg.axes) > ay else 0.0
        if bool(p('dpad_x_invert')):
            dx = -dx
        if bool(p('dpad_y_invert')):
            dy = -dy
        self._dpad = (dx, dy, b)
        prev = self._prev_joy
        self._prev_joy = (b, dx, dy)
        if prev is None:
            return
        pb, pdx, pdy = prev

        # 설정 변경은 측정 중이 아닐 때만 (측정 중 바뀌면 기록이 오염된다)
        if self.state == IDLE:
            if dy > 0.5 and pdy <= 0.5:
                self.iv = min(self.iv + 1, len(self.speeds) - 1)
            elif dy < -0.5 and pdy >= -0.5:
                self.iv = max(self.iv - 1, 0)
            if dx > 0.5 and pdx <= 0.5:
                self.ir = max(self.ir - 1, 0)
            elif dx < -0.5 and pdx >= -0.5:
                self.ir = min(self.ir + 1, len(self.radii) - 1)

        if b and not pb:                                  # B 상승 엣지
            if self.state == IDLE:
                if abs(self.v) > 0.1:
                    self._msg = '⚠ 정지 상태에서 시작하세요 (바이어스 측정 때문)'
                    return
                self.rows.clear(); self.ss.clear(); self._ss_buf.clear()
                self._warm.clear(); self.beta = 0.0
                self.state = BIAS
                self.t_state = time.time()
                self._msg = ''
            else:
                self._finish()

    # ── IMU 처리 ───────────────────────────────────────────────────────
    def _imu_cb(self, msg: Imu):
        self._seen['imu'] = time.time()
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t <= 0.0:
            t = self.get_clock().now().nanoseconds * 1e-9
        dt = (t - self.last_t) if self.last_t else 0.02
        self.last_t = t
        if not (1e-4 < dt < 0.5):
            dt = 0.02

        # 축 매핑: 전방(+x_car) = -y_imu,  좌측(+y_car) = +x_imu,  위 = +z
        ax = -msg.linear_acceleration.y * self.to_ms2
        ay = +msg.linear_acceleration.x * self.to_ms2
        wz = +msg.angular_velocity.z * self.to_rad

        if self.state == BIAS:
            self._warm.append((wz, ax, ay))
            if time.time() - self.t_state >= float(self.get_parameter('bias_sec').value):
                a = np.asarray(self._warm)
                self.bias = {'wz': float(a[:, 0].mean()),
                             'ax': float(a[:, 1].mean()),
                             'ay': float(a[:, 2].mean())}
                self.state = REC
                self.t_state = time.time()
                self._msg = ('바이어스 확정 — 출발하세요! '
                             f'(ψ̇ {math.degrees(self.bias["wz"]):+.3f}°/s)')
            return
        if self.state != REC:
            return

        wz -= self.bias['wz']; ax -= self.bias['ax']; ay -= self.bias['ay']

        # 저역통과 (1차). ※ 모터 진동은 샘플링 주파수보다 높아 에일리어싱된다 —
        # 이 필터로는 못 지운다. 여기서 지우는 건 노면 요철 정도다.
        a_lpf = dt / (dt + 1.0 / (2 * math.pi * max(self.lpf_hz, 0.1)))
        self.wz_f += a_lpf * (wz - self.wz_f)
        self.ay_f += a_lpf * (ay - self.ay_f)
        self.ax_f += a_lpf * (ax - self.ax_f)

        # 레버암 보정: a_y_cg = a_y − ψ̈·r_x
        wz_dot = (self.wz_f - self.wz_prev) / dt
        self.wz_prev = self.wz_f
        ay_cg = self.ay_f - wz_dot * self.lever_x

        v = self.v
        wz_kin = v * math.tan(self.delta) / self.L      # 기구학(무슬립) 예측
        if abs(v) > self.min_speed:
            self.beta += (ay_cg / v - self.wz_f) * dt
        if abs(self.delta) < 0.02 and abs(self.wz_f) < 0.05:
            self.beta = 0.0                              # 직선에서 리셋
        self.beta = float(np.clip(self.beta, -0.5, 0.5))

        steady = self._steady(t, v, self.delta, self.wz_f, ay_cg)
        self.rows.append((t, v, self.delta, self.wz_f, wz_kin,
                          self.ay_f, ay_cg, self.ax_f, self.beta, int(steady)))

    def _steady(self, t, v, delta, wz, ay):
        """속도·조향·요레이트가 모두 안정된 구간만 K_us 회귀에 쓴다.

        과도상태에서는 δ = L·ψ̇/v + K_us·a_y 가 성립하지 않는다(관성 항 누락).
        정상원 기동이 중요한 이유가 이것이다.
        """
        p = lambda n: self.get_parameter(n).value
        if abs(v) < self.min_speed or abs(ay) < float(p('ss_min_ay')):
            self._ss_buf.clear(); return False
        self._ss_buf.append((t, v, delta, wz, ay))
        hold = float(p('ss_hold_sec'))
        while self._ss_buf and t - self._ss_buf[0][0] > hold:
            self._ss_buf.pop(0)
        if t - self._ss_buf[0][0] < hold * 0.9:
            return False
        a = np.asarray(self._ss_buf)
        if (a[:, 1].ptp() > float(p('ss_v_tol'))
                or a[:, 2].ptp() > float(p('ss_delta_tol'))
                or a[:, 3].ptp() > float(p('ss_wz_tol'))):
            return False
        sd, say = float(a[:, 2].mean()), float(a[:, 4].mean())
        sv, swz = float(a[:, 1].mean()), float(a[:, 3].mean())
        self.ss.append((sd, say, sv, swz))
        vb = self._vbin(sv)
        if vb is not None:
            wk = sv * math.tan(sd) / self.L
            if abs(wk) > 0.05:
                self._band.setdefault(vb, []).append(swz / wk)
        self._ss_buf.clear()
        return True

    # ── 테스트 종료 → 저장 ─────────────────────────────────────────────
    def _finish(self):
        self.state = IDLE
        if len(self.rows) < 20:
            self._msg = '표본이 너무 적어 버립니다 (20개 미만)'
            return
        v, r = self.speeds[self.iv], self.radii[self.ir]
        ts = time.strftime('%Y%m%d_%H%M%S')
        path = os.path.join(_OUT, f'{ts}_v{v:.1f}_r{r:.1f}.csv')
        t0 = self.rows[0][0]
        with open(path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow([f'# target_v={v}', f'target_r={r}',
                        f'ss_points={len(self.ss)}'])
            w.writerow(['t', 'v', 'delta', 'wz', 'wz_kin', 'ay', 'ay_cg',
                        'ax', 'beta', 'steady'])
            for row in self.rows:
                w.writerow([f'{row[0]-t0:.4f}'] + [f'{x:.5f}' for x in row[1:9]]
                           + [row[9]])
        # ★ 고른 목표가 아니라 **실제로 측정된 속도 대역**을 채운다.
        #   정반경 램프 한 번이 여러 속도 칸을 동시에 채우는 게 정상이다.
        for vb, ratios in self._band.items():
            if len(ratios) >= 2:
                kk = self._key(vb, r)
                self.counts[kk] = self.counts.get(kk, 0) + 1
        try:
            json.dump(self.counts, open(self._cfile(), 'w'))
        except Exception:
            pass
        filled = ', '.join(f'{vb:.1f}㎧' for vb in sorted(self._band)
                           if len(self._band[vb]) >= 2) or '없음'
        self._msg = (f'저장 {os.path.basename(path)} — {len(self.rows)}표본, '
                     f'정상상태 {len(self.ss)}점, 채운 속도대역: {filled}')
        self._band = {}
        self._fit_all()

    # ── 전체 데이터로 K_us 회귀 ────────────────────────────────────────
    def _fit_all(self):
        """지금까지 저장된 **모든** 테스트의 정상상태 점을 모아 회귀한다.

        한 번의 정상원은 a_y 가 거의 한 점이라 기울기가 안 나온다. 여러 (속도,
        반경) 조합이 쌓여 a_y 가 넓게 퍼져야 K_us 가 의미를 갖는다 — 그래서
        화면에 진행표를 띄운다.
        """
        D, A, V, W = [], [], [], []
        for fn in sorted(os.listdir(_OUT)):
            if not fn.endswith('.csv'):
                continue
            try:
                with open(os.path.join(_OUT, fn)) as f:
                    rd = csv.reader(f)
                    next(rd); next(rd)
                    for row in rd:
                        if len(row) < 10 or row[9] != '1':
                            continue
                        D.append(float(row[2])); W.append(float(row[3]))
                        A.append(float(row[6])); V.append(float(row[1]))
            except Exception:
                continue
        if len(D) < 8:
            self.fit = None
            return
        D, A, V, W = map(np.array, (D, A, V, W))
        resid = D - self.L * W / np.maximum(np.abs(V), 1e-3) * np.sign(V)
        k, b = np.polyfit(A, resid, 1)
        pred = k * A + b
        r2 = 1.0 - ((resid - pred) ** 2).sum() / max(
            ((resid - resid.mean()) ** 2).sum(), 1e-12)
        self.fit = (k, r2, len(A), float(A.min()), float(A.max()))

    # ── 화면 ───────────────────────────────────────────────────────────
    def _draw(self):
        v, r = self.speeds[self.iv], self.radii[self.ir]
        delta = math.atan(self.L / r)           # 기구학상 필요한 조향각
        ay = v * v / r
        lim = self.mu * 9.80665
        servo = self.sg * delta + self.so
        n = self.counts.get(self._key(), 0)
        bar = '═' * BOX_W

        L = ['\033[2J\033[H', '╔' + bar + '╗']
        A = L.append
        A(_row(f'  슬립 테스트 리그{" " * 30}[{STATE_NAME[self.state]:^10}]'))
        A('╠' + bar + '╣')
        A(_row(f'  ↑↓  RT 최대속도  {v:5.1f} ㎧   (트리거 1% = {v/100:.3f}㎧)'))
        A(_row(f'  ←→  L스틱 최대각 {math.degrees(delta):5.1f}°  (반경 {r:.2f}m)'))

        A(_row(f'      예상 a_y     {ay:5.2f} ㎧²  마찰한계(μ{self.mu:.1f}) 의 '
               f'{ay / lim * 100:3.0f}%'))
        if ay > lim:
            A(_row('      ⚠ 마찰한계 초과 — 미끄러집니다 (그것도 데이터입니다)'))
            # ★ 필요 공간 경고 (2026-08-06 실차: 피트에서 7㎧ 시험 중 벽에 여러 번
            #   강하게 충돌. 11g 스파이크가 로그에 남았다).
            #   한계를 넘으면 차는 설정 반경을 못 지키고, 잡을 수 있는 최소 반경은
            #   r_min = v²/(μg) 로 **속도의 제곱**에 비례해 커진다. 설정 반경이
            #   아니라 이 값이 실제로 필요한 공간이다.
            r_real = v * v / lim
            A(_row(f'      ⚠ 실제 선회반경 ≈ {r_real:4.1f}m → 지름 {2*r_real:4.1f}m '
                   f'이상의 빈 공간 필요'))
            A(_row('      ⚠ 좁은 실내(피트)에서 금지 — 벽에 그대로 박습니다'))
        A('╠' + bar + '╣')
        A(_row(f'  이 설정 측정  {n} / {self.target_runs} 회'
               + ('   ✔ 충분' if n >= self.target_runs else '')))
        A(_row('   반경\\속도 ' + ''.join(f'{s:>6.1f}' for s in self.speeds)))
        for rr in self.radii:
            cells = ''
            for ss in self.speeds:
                c = self.counts.get(self._key(ss, rr), 0)
                txt = str(c) if c else '·'
                sel = abs(ss - v) < 1e-6 and abs(rr - r) < 1e-6
                cells += f'{"[" + txt + "]":>6}' if sel else f'{txt:>6}'
            A(_row(f'   {rr:5.2f}m    ' + cells))
        A('╠' + bar + '╣')

        if self.state == REC and self.rows:
            _, cv, cd, cw, cwk, _, cay, _, cb, _ = self.rows[-1]
            ratio = (cw / cwk) if abs(cwk) > 0.05 else float('nan')
            rad = (cv / cw) if abs(cw) > 0.05 else float('nan')
            # L스틱을 끝까지 밀면 상한(=시험각)에 닿는다. 얼마나 닿았는지 표시.
            hit = abs(cd) / max(delta, 1e-6)
            mark = '✔ 끝까지' if hit > 0.97 else f'스틱 더 밀기 ({hit*100:3.0f}%)'
            A(_row(f'  {time.time() - self.t_state:5.1f}초   v {cv:4.2f}㎧'
                   f'   δ {math.degrees(cd):+6.1f}° / 상한 {math.degrees(delta):.1f}°'
                   f'   {mark}'))
            A(_row(f'  실제 반경 {rad:5.2f}m  (조향을 고정하면 슬립이 생길수록 커진다)'))
            A(_row(f'  a_y {cay:+5.2f}㎧² ({cay / lim * 100:3.0f}%)'
                   f'   ψ̇ {math.degrees(cw):+6.1f}°/s'
                   f'   정상상태 {len(self.ss)}점   β {math.degrees(cb):+5.1f}°'))
            # ── 속도 대역별 실제/기구학 — 이게 슬립 시작점을 보여준다 ──
            # 1.00 = 명령한 만큼 돈다(슬립 없음). 1 보다 작으면 덜 도는 것,
            # 즉 전륜이 미끄러지는 언더스티어다.
            cells, onset = '', None
            for ss_ in self.speeds:
                rs = self._band.get(ss_, [])
                if rs:
                    m = float(np.mean(rs[-5:]))
                    cells += f'  {ss_:.1f}:{m:.2f}'
                    if onset is None and m < 0.95:
                        onset = (ss_, m)
                else:
                    cells += f'  {ss_:.1f}:—'
            A(_row('  속도별 실제/기구학' + cells))
            if onset:
                A(_row(f'  ⚠ 슬립 시작 — {onset[0]:.1f}㎧ 에서 {onset[1]:.2f}'
                       f'  (반경 {r:.1f}m 기준)'))
        elif self.state == BIAS:
            left = float(self.get_parameter('bias_sec').value) - (time.time() - self.t_state)
            A(_row(f'  바이어스 측정 중… {max(left, 0):.1f}초  (움직이지 마세요)'))
        else:
            A(_row(f'  L스틱 끝까지 = {math.degrees(delta):.1f}°,'
                   f'  RT 끝까지 = {v:.1f}㎧ 로 걸어 두었습니다'))
            A(_row(f'  B = 시작  →  L스틱 고정, RT 를 {v:.1f}㎧ 까지'
                   f' {v/0.15:.0f}초에 걸쳐 천천히'))

        A('╠' + bar + '╣')
        now = time.time()
        ok = {k: (now - t) < 1.0 for k, t in self._seen.items()}
        A(_row('  연결  ' + '  '.join(
            f'{n} {"✔" if ok[k] else "✘"}'
            for k, n in (('joy', '조이스틱'), ('odom', '오도메트리'),
                         ('imu', 'IMU'), ('teleop', '조종출력')))))
        dx, dy, bb = self._dpad
        A(_row(f'  D-pad  x {dx:+.0f}  y {dy:+.0f}   B {"눌림" if bb else " · "}'
               f'   (반대로 움직이면 dpad_*_invert)'))
        if ok['joy'] and not ok['teleop']:
            # joystick_teleop 은 트리거가 중립인 걸 한 번 봐야 스로틀을 푼다.
            A(_row('  ⚠ 조종 출력 없음 — RT 에서 손을 떼세요 (트리거 중립 대기)'))
        fit = getattr(self, 'fit', None)
        if fit:
            k, r2, nn, amin, amax = fit
            A(_row(f'  K_us {k:+.5f} rad/㎧² ({math.degrees(k):+.2f}°/㎧²)'
                   f'   R² {r2:.2f}   n={nn}'))
            A(_row(f'  a_y 범위 {amin:+.1f} ~ {amax:+.1f} ㎧²'
                   + ('   ← 좁다. 조합을 넓히세요' if amax - amin < 3.0 else '')))
        else:
            A(_row('  K_us — 정상상태 8점 이상 모이면 표시됩니다'))
        A('╚' + bar + '╝')
        if self._msg:
            A('  ' + self._msg)
        print('\n'.join(L), flush=True)


def main():
    rclpy.init()
    node = SlipLogger()
    node._fit_all()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.state == REC:
            node._finish()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
