#!/usr/bin/env python3
"""
MAP (Model-based Adaptive Pure Pursuit) 컨트롤러
- L1 가이던스로 횡가속도 계산
- Pacejka 타이어 모델 역산으로 슬립각 보정 → 조향각
- 곡률 기반 속도 캡 (v_max = sqrt(μgR)) + 역방향 스윕
- 참고: ETH-PBL MAP-Controller (github.com/ETH-PBL/MAP-Controller)
"""

import rclpy
from rclpy.node import Node
import numpy as np
import csv
import os
import yaml
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

from f1tenth_racing.pure_pursuit_node import WAYPOINTS_MAP, _MAPS_DIR

WAYPOINTS_PATH = os.path.join(_MAPS_DIR, f'{WAYPOINTS_MAP}_waypoints.csv')


def _load_race_script():
    """race_script.yaml의 map 섹션을 읽어 파라미터 dict 반환. 실패시 빈 dict."""
    d = os.path.dirname(os.path.abspath(__file__))
    for _ in range(12):
        p = os.path.join(d, 'race_script.yaml')
        if os.path.isfile(p):
            try:
                cfg = yaml.safe_load(open(p))
                return cfg.get('map', {})
            except Exception:
                return {}
        d = os.path.dirname(d)
    return {}


class MAPController(Node):
    def __init__(self):
        super().__init__('map_controller')
        self._near_idx    = 0
        self._initialized = False

        rs = _load_race_script()

        # ── 차량 파라미터 ──────────────────────────────────────────────────
        lf = rs.get('lf', 0.1757)
        lr = rs.get('lr', 0.1441)
        self.WHEELBASE   = lf + lr
        self.MAX_STEER   = 0.4189      # gym s_max [rad]
        self.SPEED_SCALE = rs.get('speed_scale', 2.0)

        # ── L1 가이던스 파라미터 ──────────────────────────────────────────
        self.L1_MIN = 1.3155
        self.L1_K   = 0.2993

        # ── 차량 동역학 ───────────────────────────────────────────────────
        self.MASS = rs.get('mass', 3.354)
        self.LF   = lf
        self.LR   = lr
        self.MU   = rs.get('mu',   1.0)

        # ── Pacejka Magic Formula 파라미터 ───────────────────────────────
        self.PAC_B = rs.get('pac_b', 3.8)
        self.PAC_C = rs.get('pac_c', 1.3)
        self.PAC_D = rs.get('pac_d', 1.15)
        self.PAC_E = rs.get('pac_e', -0.5)

        # ── 속도 프로파일 파라미터 ────────────────────────────────────────
        self.A_BRAKE = 6.0902

        # ── 조향 저역통과 필터 ────────────────────────────────────────────
        self.STEER_ALPHA = 0.4961
        self._steer_prev = 0.0
        self._cur_speed  = 0.0

        # ── 초기화 ────────────────────────────────────────────────────────
        self.wp = self._load_waypoints(WAYPOINTS_PATH)
        self.get_logger().info(f'웨이포인트 {len(self.wp)}개 로드 완료')

        self._precompute_spline_data()
        self._precompute_speed_profile()
        self._precompute_pacejka_lut()

        # ── ROS 인터페이스 ────────────────────────────────────────────────
        self.drive_pub  = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.path_pub   = self.create_publisher(Path, '/waypoints_path', 1)
        self.marker_pub = self.create_publisher(MarkerArray, '/waypoints_markers', 1)
        self.create_subscription(Odometry, '/ego_racecar/odom', self._odom_cb, 10)
        self.create_timer(1.0, self._publish_path)
        self._path_published = False

        input('Enter 키를 누르면 주행을 시작합니다...')

    # ── 웨이포인트 로드 ────────────────────────────────────────────────────
    def _load_waypoints(self, path):
        pts = []
        with open(path) as f:
            for row in csv.DictReader(f):
                pts.append([float(row['x']), float(row['y']), float(row['speed'])])
        return np.array(pts)

    # ── 사전 계산: 쿠빅 스플라인 곡률 ───────────────────────────────────
    def _precompute_spline_data(self):
        from scipy.interpolate import CubicSpline
        pts = self.wp[:, :2]
        n   = len(pts)
        diffs = np.diff(pts, axis=0)
        ds    = np.linalg.norm(diffs, axis=1)
        s     = np.concatenate([[0.0], np.cumsum(ds)])
        s_ext = np.append(s, s[-1] + np.linalg.norm(pts[0] - pts[-1]))
        x_ext = np.append(pts[:, 0], pts[0, 0])
        y_ext = np.append(pts[:, 1], pts[0, 1])
        cs_x  = CubicSpline(s_ext, x_ext, bc_type='periodic')
        cs_y  = CubicSpline(s_ext, y_ext, bc_type='periodic')
        dx  = cs_x(s, 1);  dy  = cs_y(s, 1)
        ddx = cs_x(s, 2);  ddy = cs_y(s, 2)
        denom = np.maximum((dx**2 + dy**2) ** 1.5, 1e-6)
        self._curvature = np.abs(dx * ddy - dy * ddx) / denom
        self.get_logger().info(
            f'곡률 사전 계산: max={self._curvature.max():.3f} mean={self._curvature.mean():.3f} 1/m')

    # ── 사전 계산: 역방향 스윕 속도 프로파일 (곡률 캡 없음 — 슬립 테스트용) ──
    def _precompute_speed_profile(self):
        n  = len(self.wp)
        ds = np.array([
            np.linalg.norm(self.wp[(i + 1) % n, :2] - self.wp[i, :2])
            for i in range(n)
        ])
        self._v_profile = self.wp[:, 2].copy() * self.SPEED_SCALE

        for _ in range(3):
            for i in range(n - 1, -1, -1):
                j = (i + 1) % n
                v_lim = np.sqrt(self._v_profile[j] ** 2 + 2.0 * self.A_BRAKE * ds[i])
                self._v_profile[i] = min(self._v_profile[i], v_lim)

        self.get_logger().info(
            f'속도 프로파일: {self._v_profile.min():.2f}~{self._v_profile.max():.2f} m/s '
            f'(SPEED_SCALE={self.SPEED_SCALE})')

    # ── 사전 계산: Pacejka LUT (단조 구간 ±0.3 rad) ──────────────────
    def _precompute_pacejka_lut(self):
        alphas = np.linspace(-0.30, 0.30, 500)
        Fy_norm = self.PAC_D * np.sin(
            self.PAC_C * np.arctan(
                self.PAC_B * alphas
                - self.PAC_E * (self.PAC_B * alphas - np.arctan(self.PAC_B * alphas))
            )
        )
        self._pac_alpha_lut = alphas
        self._pac_Fy_lut    = Fy_norm
        self.get_logger().info(
            f'Pacejka LUT: Fy=[{Fy_norm[0]:.3f}, {Fy_norm[-1]:.3f}]')

    def _pacejka_inv(self, Fy_norm_req: float) -> float:
        clamped = float(np.clip(Fy_norm_req,
                                self._pac_Fy_lut[0]  * 0.99,
                                self._pac_Fy_lut[-1] * 0.99))
        return float(np.interp(clamped, self._pac_Fy_lut, self._pac_alpha_lut))

    # ── MAP 조향각 계산 ────────────────────────────────────────────────
    def _steer_map(self, pos, yaw, goal) -> float:
        v  = max(self._cur_speed, 0.3)
        L1 = max(self.L1_MIN, self.L1_K * v)

        dx  = goal[0] - pos[0]
        dy  = goal[1] - pos[1]
        eta = float(np.arctan2(
            np.sin(np.arctan2(dy, dx) - yaw),
            np.cos(np.arctan2(dy, dx) - yaw)))

        a_lat = 2.0 * v**2 * np.sin(eta) / L1

        Fzf     = self.MASS * 9.81 * self.LR / self.WHEELBASE
        F_yf    = self.MASS * a_lat * self.LR / self.WHEELBASE
        alpha_f = self._pacejka_inv(F_yf / (self.MU * Fzf))

        delta_kin = np.arctan(self.WHEELBASE * 2.0 * np.sin(eta) / max(L1, 1e-3))
        return float(np.clip(delta_kin + alpha_f, -self.MAX_STEER, self.MAX_STEER))

    # ── 경로 시각화 ────────────────────────────────────────────────────
    def _publish_path(self):
        now = self.get_clock().now().to_msg()
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp    = now
        for x, y, _ in self.wp:
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0
            path_msg.poses.append(ps)
        path_msg.poses.append(path_msg.poses[0])
        self.path_pub.publish(path_msg)

        v_min = self._v_profile.min()
        v_max = self._v_profile.max()
        v_rng = max(v_max - v_min, 0.1)
        markers = MarkerArray()
        for i, (x, y, _) in enumerate(self.wp):
            t = (self._v_profile[i] - v_min) / v_rng
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp    = now
            m.ns = 'waypoints'; m.id = i
            m.type = Marker.SPHERE; m.action = Marker.ADD
            m.pose.position.x = x; m.pose.position.y = y
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.15
            m.color = ColorRGBA(r=1.0 - t, g=t, b=0.0, a=0.8)
            markers.markers.append(m)
        self.marker_pub.publish(markers)

        if not self._path_published:
            self.get_logger().info('경로 시각화 완료')
            self._path_published = True

    # ── 오도메트리 콜백 ────────────────────────────────────────────────
    def _odom_cb(self, msg):
        px  = msg.pose.pose.position.x
        py  = msg.pose.pose.position.y
        q   = msg.pose.pose.orientation
        yaw = np.arctan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y**2 + q.z**2))
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self._cur_speed = float(np.hypot(vx, vy))

        v  = max(self._cur_speed, 0.3)
        L1 = max(self.L1_MIN, self.L1_K * v)

        goal, _, near_idx = self._find_lookahead(np.array([px, py]), L1, yaw)
        if goal is None:
            return

        target_speed = float(self._v_profile[near_idx])

        pos   = np.array([px, py])
        steer = self._steer_map(pos, yaw, goal)
        steer = self.STEER_ALPHA * self._steer_prev + (1.0 - self.STEER_ALPHA) * steer
        self._steer_prev = steer

        self.get_logger().info(
            f'[MAP] κ={self._curvature[near_idx]:.3f}  '
            f'steer={np.degrees(steer):.1f}°  spd={target_speed:.2f}m/s',
            throttle_duration_sec=0.1)

        cmd = AckermannDriveStamped()
        cmd.drive.steering_angle = steer
        cmd.drive.speed          = float(target_speed)
        self.drive_pub.publish(cmd)

    # ── Lookahead 포인트 탐색 ──────────────────────────────────────────
    def _find_lookahead(self, pos, lookahead, yaw=None):
        dists = np.linalg.norm(self.wp[:, :2] - pos, axis=1)
        n     = len(self.wp)

        if not self._initialized:
            near = int(np.argmin(dists))
            if yaw is not None:
                fwd    = np.array([np.cos(yaw), np.sin(yaw)])
                wp_dir = self.wp[(near + 1) % n, :2] - self.wp[near, :2]
                if np.dot(fwd, wp_dir) < 0:
                    self.get_logger().warn('웨이포인트 방향 역전 감지 → 배열 역전')
                    self.wp         = self.wp[::-1]
                    self._curvature = self._curvature[::-1]
                    self._v_profile = self._v_profile[::-1]
                    near = int(n - 1 - near)
            self._initialized = True
        else:
            candidates = [(self._near_idx + i) % n for i in range(50)]
            near = candidates[int(np.argmin([dists[i] for i in candidates]))]

        self._near_idx = near

        arc = 0.0
        for i in range(n):
            i0  = (near + i)     % n
            i1  = (near + i + 1) % n
            seg = float(np.linalg.norm(self.wp[i1, :2] - self.wp[i0, :2]))
            if arc + seg >= lookahead:
                alpha   = float(np.clip((lookahead - arc) / max(seg, 1e-6), 0.0, 1.0))
                goal_xy = (1.0 - alpha) * self.wp[i0, :2] + alpha * self.wp[i1, :2]
                spd     = (1.0 - alpha) * self.wp[i0, 2]  + alpha * self.wp[i1, 2]
                return goal_xy, spd, near
            arc += seg

        return None, None, near


def main(args=None):
    rclpy.init(args=args)
    node = MAPController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
