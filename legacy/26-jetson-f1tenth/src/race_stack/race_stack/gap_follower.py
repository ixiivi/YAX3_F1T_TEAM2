#!/usr/bin/env python3
"""
Gap Follower (Disparity Extender) — Q2 / 1대1 경주용 반응형 컨트롤러.

맵 없이도 동작한다 (LiDAR만으로 회피). 정적 장애물과 상대 차량 모두
"가까운 물체"로 취급되어 자연스럽게 회피된다.

구독: /scan (LaserScan), /race/enabled (Bool, mission_manager가 발행)
발행: /drive (AckermannDriveStamped, mux priority 10)
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from sensor_msgs.msg import LaserScan

from race_stack.speed_profile import steer_speed_limit
from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDriveStamped


# ★ /race/enabled 는 mission_manager 가 **래치(TRANSIENT_LOCAL)** 로 발행한다.
#   기본 QoS(VOLATILE)로 구독하면 구독자가 생기기 전에 나간 값을 못 받는다.
#   race.launch.py 는 컨트롤러를 5~8초 지연 기동하므로 항상 그 상황이고, 그래서
#   출발 게이트가 걸어둔 enabled=false 를 놓쳐 버튼도 안 눌렀는데 출발했다
#   (2026-08-04 실차). 래치로 구독해야 마지막 값을 받는다.
LATCHED_IN = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

class GapFollower(Node):
    def __init__(self):
        super().__init__('gap_follower')
        # --- 파라미터 (race_script.yaml → launch에서 주입됨) ---
        # ── 물리 기반 속도 한계 (2026-08-09) ────────────────────────────
        # 반응형이라 레이싱라인이 없지만, **자기가 내리려는 조향각이 곧 곡률**이다:
        #     κ = |tan δ| / L   →   v ≤ √(η·μ_eff·g / κ)
        # 예전의 "조향이 크면 0.6배" 같은 임의 계수와 달리, 노면(μ)이나 차량
        # 기하가 바뀌면 자동으로 따라간다.
        self.declare_parameter('grip_util', 0.6)
        self.declare_parameter('mu',        0.348)   # race_script.yaml [map] 원본
        self.declare_parameter('pac_d',     1.15)
        self.declare_parameter('wheelbase', 0.32)
        # 물리와 무관한 안전 천장. 0 = 끔(기본).
        self.declare_parameter('hard_cap',  0.0)
        self.declare_parameter('max_speed', 2.0)
        # ★ 박스 위 벤치 모드. true 면 **발행 직전에 속도를 0 으로 못박는다**.
        #   조향은 살아 있어야 검증이 되므로 건드리지 않는다.
        self.declare_parameter('bench', False)
        self.declare_parameter('min_speed', 1.0)
        self.declare_parameter('car_width', 0.40)
        self.declare_parameter('disparity_threshold', 0.4)
        self.declare_parameter('max_range', 8.0)
        self.declare_parameter('fov_deg', 180.0)
        self.declare_parameter('steer_gain', 0.8)
        self.declare_parameter('speed_gain', 0.5)
        self.declare_parameter('max_steer', 0.36)

        self.enabled = False   # ★ mission_manager 의 /race/enabled(래치)를 받아야 켜진다.
        #   True 로 두면 그 값이 도착하기 전 한 틱이라도 먼저 나가버린다.
        #   mission_manager 가 없는 단독 실행이면 "정지 중: /race/enabled=false"
        #   로그가 이유를 알려준다 — 조용히 달려나가는 것보다 낫다.
        self.create_subscription(Bool, '/race/enabled', self.enabled_cb, LATCHED_IN)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.get_logger().info('GapFollower 시작')

    def enabled_cb(self, msg: Bool):
        if self.enabled and not msg.data:
            self.publish_stop()
        self.enabled = msg.data

    def p(self, name):
        return self.get_parameter(name).value

    # ---------------- 핵심 로직 ----------------
    def scan_cb(self, scan: LaserScan):
        if not self.enabled:
            return

        ranges = np.asarray(scan.ranges, dtype=float)
        ranges = np.nan_to_num(ranges, nan=0.0, posinf=self.p('max_range'))
        ranges = np.clip(ranges, 0.0, self.p('max_range'))

        # 1) 전방 FOV만 사용
        n = len(ranges)
        angles = scan.angle_min + np.arange(n) * scan.angle_increment
        half_fov = np.deg2rad(self.p('fov_deg')) / 2.0
        fov_mask = np.abs(angles) <= half_fov
        idx0 = np.argmax(fov_mask)                      # FOV 시작 인덱스
        r = ranges[fov_mask].copy()
        a = angles[fov_mask]
        if len(r) < 10:
            return

        # 2) Disparity Extender: 불연속 엣지를 차폭/2 만큼 가까운 값으로 확장
        half_w = self.p('car_width') / 2.0
        thr = self.p('disparity_threshold')
        inc = scan.angle_increment
        diffs = np.abs(np.diff(r))
        disp_idx = np.where(diffs > thr)[0]
        for i in disp_idx:
            closer = min(r[i], r[i + 1])
            if closer < 0.05:
                continue
            n_ext = int(np.ceil(half_w / (closer * inc)))
            if r[i] > r[i + 1]:          # i+1이 가까움 → i쪽(왼쪽 방향)으로 확장
                lo = max(0, i - n_ext + 1)
                r[lo:i + 1] = np.minimum(r[lo:i + 1], closer)
            else:                         # i가 가까움 → i+1쪽으로 확장
                hi = min(len(r), i + 1 + n_ext)
                r[i + 1:hi] = np.minimum(r[i + 1:hi], closer)

        # 3) 가장 먼 지점(=가장 넓게 열린 방향) 선택
        best_idx = int(np.argmax(r))
        target_angle = a[best_idx]

        # 4) 조향/속도 계산
        steer = float(np.clip(self.p('steer_gain') * target_angle,
                              -self.p('max_steer'), self.p('max_steer')))

        # 전방 ±10도 평균거리 기반 속도 (가까우면 감속)
        front_mask = np.abs(a) < np.deg2rad(10)
        front_dist = float(np.mean(r[front_mask])) if front_mask.any() else 0.0
        # 전방 여유에서 오는 상한 (벽이 가까우면 감속) — 이건 물리가 아니라 안전이다
        speed = self.p('speed_gain') * front_dist

        # ★ 조향각 → 곡률 → 마찰 예산 안에서의 최대 속도 (물리)
        v_phys = steer_speed_limit(steer, self.p('wheelbase'),
                                   self.p('mu') * self.p('pac_d'),
                                   self.p('grip_util'))
        speed = min(speed, v_phys)
        cap = float(self.p('hard_cap'))
        if cap > 0:
            speed = min(speed, cap)
        speed = max(speed, self.p('min_speed'))

        # ★★ bench(박스 위) — 여기가 유일한 강제 지점이다. 옮기거나 지우지 말 것.
        #   2026-08-09: 예전엔 bench 가 max_speed=0 을 주고 컨트롤러의 클램프에
        #   기대고 있었는데, 속도 개편에서 그게 사라져 box 가 풀속도로 주행했다.
        if bool(self.p('bench')):
            speed = 0.0
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = steer
        self.drive_pub.publish(msg)

    def publish_stop(self):
        msg = AckermannDriveStamped()
        msg.drive.speed = 0.0
        self.drive_pub.publish(msg)


def main():
    rclpy.init()
    node = GapFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ctrl-C 로 컨텍스트가 이미 닫힌 뒤면 발행이 예외를 던진다.
        # 종료 경로에서만 나는 노이즈라 조건을 두고 조용히 넘긴다.
        if rclpy.ok():
            node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
