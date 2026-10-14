#!/usr/bin/env python3
"""
Safety Node (AEB) — 주행 통로(swept corridor) 기반 자동 비상 브레이크.

■ 왜 표준 iTTC 를 안 쓰는가 (2026-07-21 오발동 규명)

  F1TENTH Lab2 표준 AEB 는 모든 빔에 대해
      iTTC = range / max(v·cos θ, ε)
  를 구하고 최솟값이 임계 미만이면 제동한다. 이 저장소도 그렇게 돼 있었다.

  문제: 주행 방향과 나란한 벽은 차가 전진해도 그 빔의 range 가 거의 변하지
  않는다 (실제 range rate ≈ 0). 그런데 이 식은 접근 속도를 무조건 v·cos θ 로
  가정하므로, 측면 기하에 대해 충돌을 체계적으로 과다 예측한다. 각도 제한도
  proj>0.05 하나뿐이라 전방 반평면(±88°) 전체가 판정에 들어갔고, np.min 이라
  그 안의 빔 하나만 걸려도 발동했다. 실제로 장애물이 없는데 제동이 걸렸다.

■ 지금 방식 — 차가 실제로 지나갈 통로만 본다

  1. 스캔을 차량 좌표계 (x 전방, y 좌) 로 변환
  2. /odom 의 yaw rate 로 선회 반경 R = v/ω 을 얻어 **주행 경로를 원호로 예측**
     (직진이면 직선). 코너에서 바깥 벽에 오발동하지 않는 이유가 이것이다.
  3. 그 경로를 차폭만큼 부풀린 통로 안에 있는 점만 남긴다
     → 나란한 벽은 통로 밖이므로 애초에 후보에서 빠진다
  4. 남은 점까지의 **호 길이**가 정지거리보다 짧으면 제동
         d_stop = v²/(2a) + v·t_react
     (물리적 제동거리 + 반응시간. ttc_threshold 가 t_react 역할)
  5. 연속 N 프레임 검출되어야 발동 (단발 노이즈 제거)
  6. range_min 미만은 차체 자체 반사로 보고 버림

  이는 ForzaETH(F1TENTH 우승팀) 스택이 트랙 경계로 라이다 점을 걸러 벽을
  배제하는 것과 같은 취지다. 이 저장소의 stanley_controller Q2 경로도 이미
  inner/outer_bound 로 같은 일을 한다 — AEB 만 빠져 있었다.

주의: 대회 규정상 '수동' 비상브레이크는 금지지만, 온보드에서 자율적으로
동작하는 충돌 방지 로직은 자율주행의 일부이다. 하드웨어 킬스위치(토글식)는
별도로 반드시 장착해야 한다.
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')

        # ttc_threshold 는 이제 "반응시간(s)" 역할 — 정지거리 식의 v·t 항.
        self.declare_parameter('ttc_threshold',      0.15)
        self.declare_parameter('hold_sec',           1.0)
        # 통로 반폭 = 차폭/2 + 여유. 0.30 이면 폭 60cm 통로를 본다.
        self.declare_parameter('corridor_half_width', 0.30)
        # 제동 감속도 (m/s²). 실측 전엔 보수적으로.
        self.declare_parameter('brake_decel',        3.0)
        # 차체 자체 반사 제거. 이보다 가까운 점은 무시.
        self.declare_parameter('min_range',          0.12)
        # 통로를 볼 최대 거리 (m) — 이보다 먼 점은 계산에서 제외
        self.declare_parameter('max_lookahead',      6.0)
        # 이만큼 연속 프레임 검출되어야 발동 (단발 노이즈 제거)
        self.declare_parameter('consec_frames',      3)
        # 곡선 통로 사용. false 면 항상 직선 통로 (디버깅용)
        self.declare_parameter('use_arc_corridor',   True)
        # RViz 디버그 마커 발행
        self.declare_parameter('publish_debug',      True)
        # ★ 기본이 /odom_imu 다 (2026-08-04). 통로 예측이 yaw rate 로 선회 반경
        #   R=v/ω 을 잡는데, 원본 /odom 의 ω 는 서보 "명령" 에서 나와 서보 지연·
        #   타이어 슬립만큼 과대평가된다. 즉 실제보다 많이 휜 통로를 보고 판정했다.
        self.declare_parameter('odom_topic',         '/odom_imu')

        self.speed = 0.0
        self.yaw_rate = 0.0
        self.braking_until = None
        self.hit_streak = 0

        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self.odom_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        # mux에서 /teleop 이 priority 100 → 컨트롤러(/drive, 10)보다 우선
        self.brake_pub = self.create_publisher(AckermannDriveStamped, '/teleop', 10)
        self.dbg_pub = self.create_publisher(Marker, '/aeb_corridor', 1)

        p = self.get_parameter
        self.get_logger().info(
            f'SafetyNode(AEB, corridor) 시작 — '
            f'반폭 {p("corridor_half_width").value}m, '
            f'감속 {p("brake_decel").value}m/s², '
            f'반응 {p("ttc_threshold").value}s, '
            f'연속 {p("consec_frames").value}프레임')

    # ── 콜백 ──────────────────────────────────────────────────────────────

    def odom_cb(self, msg: Odometry):
        self.speed = msg.twist.twist.linear.x
        self.yaw_rate = msg.twist.twist.angular.z

    def scan_cb(self, scan: LaserScan):
        now = self.get_clock().now()

        # 제동 유지 중이면 계속 정지 명령
        if self.braking_until is not None:
            if now < self.braking_until:
                self.publish_brake()
                return
            self.braking_until = None
            self.hit_streak = 0

        v = float(self.speed)
        if abs(v) < 0.1:
            self.hit_streak = 0
            return

        hit = self._closest_in_corridor(scan, v)

        if hit is None:
            self.hit_streak = 0
            return

        s, x, y, rng = hit
        d_stop = self._stopping_distance(abs(v))
        if s >= d_stop:
            self.hit_streak = 0
            return

        # 단발 노이즈 제거 — 연속 N 프레임이어야 진짜로 본다
        self.hit_streak += 1
        if self.hit_streak < int(self.get_parameter('consec_frames').value):
            return

        ang = math.degrees(math.atan2(y, x))
        self.get_logger().warn(
            f'AEB! 통로 내 장애물 {s:.2f}m (정지거리 {d_stop:.2f}m) '
            f'θ={ang:+.0f}° range={rng:.2f}m v={v:.2f}m/s')
        self.braking_until = now + rclpy.duration.Duration(
            seconds=float(self.get_parameter('hold_sec').value))
        self.hit_streak = 0
        self.publish_brake()

    # ── 판정 ──────────────────────────────────────────────────────────────

    def _stopping_distance(self, v: float) -> float:
        """제동거리 = v²/(2a) + v·t_react."""
        a = max(float(self.get_parameter('brake_decel').value), 0.1)
        t = float(self.get_parameter('ttc_threshold').value)
        return v * v / (2.0 * a) + v * t

    def _closest_in_corridor(self, scan: LaserScan, v: float):
        """주행 통로 안에서 가장 가까운 점 → (호길이, x, y, range). 없으면 None."""
        p = self.get_parameter
        half_w = float(p('corridor_half_width').value)
        r_min = float(p('min_range').value)
        r_max = float(p('max_lookahead').value)

        ranges = np.asarray(scan.ranges, dtype=float)
        n = len(ranges)
        if n == 0:
            return None
        angles = scan.angle_min + np.arange(n) * scan.angle_increment

        # 유효 범위만. r_min 미만은 차체 자체 반사로 보고 버린다.
        ok = np.isfinite(ranges) & (ranges > r_min) & (ranges < r_max)
        if not ok.any():
            return None
        r = ranges[ok]
        a = angles[ok]
        x = r * np.cos(a)
        y = r * np.sin(a)

        # 선회 반경. ω 가 거의 0 이면 직진으로 본다.
        R = None
        if bool(p('use_arc_corridor').value):
            w = float(self.yaw_rate)
            if abs(w) > 1e-3 and abs(v) > 1e-3:
                R = v / w
                # 반경이 지나치게 크면 수치적으로 직선과 같다
                if abs(R) > 1e3:
                    R = None

        # 후진이면 좌표를 원점 대칭시켜 전진 문제로 바꾼다 (선회 중심도 반전)
        if v < 0:
            x, y = -x, -y
            if R is not None:
                R = -R

        if R is None:
            # 직선 통로: |y| < 반폭, 전방
            inside = (np.abs(y) < half_w) & (x > 0.0)
            if not inside.any():
                return None
            s = x[inside]
        else:
            # 원호 통로: 중심 (0, R) 에서의 거리가 |R| ± 반폭 이내
            d = np.hypot(x, y - R)
            inside = np.abs(d - abs(R)) < half_w
            if not inside.any():
                return None
            # 호 각도 φ — P(φ) = (R sinφ, R(1-cosφ)) 를 역산.
            # sign(R) 을 곱해 좌/우 선회 모두 φ∈[0,2π) 로 정규화된다.
            sr = math.copysign(1.0, R)
            phi = np.arctan2(sr * x[inside], sr * (R - y[inside]))
            phi = np.mod(phi, 2.0 * math.pi)
            s = abs(R) * phi

        i = int(np.argmin(s))
        xi = x[inside][i]
        yi = y[inside][i]
        # 마커는 원래(비반전) 좌표로 그려야 RViz 에서 맞는다
        if v < 0:
            xi, yi = -xi, -yi
        if bool(p('publish_debug').value):
            self._publish_debug(x[inside], y[inside], v)
        return float(s[i]), float(xi), float(yi), float(r[inside][i])

    # ── 출력 ──────────────────────────────────────────────────────────────

    def publish_brake(self):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = 0.0
        self.brake_pub.publish(msg)

    def _publish_debug(self, xs, ys, v: float):
        """통로 안에 들어온 점들을 RViz 에 표시 — 오발동 시 범인을 눈으로 확인."""
        m = Marker()
        m.header.frame_id = 'base_link'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'aeb'
        m.id = 0
        m.type = Marker.POINTS
        m.action = Marker.ADD
        m.scale.x = m.scale.y = 0.06
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.2, 0.0, 0.9
        sign = -1.0 if v < 0 else 1.0
        for px, py in zip(xs, ys):
            m.points.append(Point(x=float(px) * sign, y=float(py) * sign, z=0.0))
        self.dbg_pub.publish(m)


def main():
    rclpy.init()
    node = SafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
