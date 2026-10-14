#!/usr/bin/env python3
"""
하이브리드 경로 추종 컨트롤러
- MAP  (Model-based Adaptive Pure Pursuit) : 코너 + 경로 이탈 복귀 (Pacejka 슬립 보정)
- Stanley Controller                        : 완만한 커브·직선 구간
- 쿠빅 스플라인 기반 곡률·헤딩 사전 계산
- 곡률 기반 속도 캡 (v_max = √(μgR)) + 역방향 스윕 속도 프로파일 사전 계산
"""

import rclpy
from rclpy.node import Node
import numpy as np
import csv
import os
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

from f1tenth_racing.pure_pursuit_node import WAYPOINTS_MAP, _MAPS_DIR

WAYPOINTS_PATH = os.path.join(_MAPS_DIR, f'{WAYPOINTS_MAP}_waypoints.csv')


class HybridController(Node):
    def __init__(self):
        super().__init__('hybrid_controller')
        self._near_idx    = 0
        self._initialized = False

        # ── 차량 파라미터 ───────────────────────────────────────────────────
        self.WHEELBASE   = 0.33
        self.MAX_STEER   = 0.4
        self.SPEED_SCALE = 1.0

        # ── lookahead (near_idx 탐색용) ─────────────────────────────────────
        self.LA_BASE = 0.5931
        self.LA_K    = 0.1234

        # ── Stanley Controller 파라미터 ─────────────────────────────────────
        self.STANLEY_K = 0.5

        # ── MAP 컨트롤러 파라미터 (L1 가이던스 + Pacejka) ───────────────────
        self.MAP_L1_MIN = 0.45
        self.MAP_L1_K   = 0.12
        self.MASS = 3.74
        self.LF   = 0.15875
        self.LR   = 0.17145
        self.MU   = 1.0489
        self.PAC_B = 10.0
        self.PAC_C = 1.3
        self.PAC_D = 1.0
        self.PAC_E = -0.5

        # ── 제어기 전환 임계값 ──────────────────────────────────────────────
        self.HIGH_CURV          = 0.3
        self.CTE_RECOVER_THRESH = 0.3
        self.BLEND_MARGIN       = 0.03

        # ── 역방향 스윕 속도 프로파일용 제동 가속도 ──────────────────────────
        self.A_BRAKE = 9.4675

        # ── 조향 저역통과 필터 ──────────────────────────────────────────────
        self.STEER_ALPHA = 0.40
        self._steer_prev = 0.0
        self._cur_speed  = 0.0

        # ── 웨이포인트 로드 ─────────────────────────────────────────────────
        self.wp = self._load_waypoints(WAYPOINTS_PATH)
        self.get_logger().info(f'웨이포인트 {len(self.wp)}개 로드 완료')

        # ── 사전 계산 ───────────────────────────────────────────────────────
        self._precompute_spline_data()
        self._precompute_speed_profile()
        self._precompute_pacejka_lut()
        self._save_controller_map()

        # ── ROS 인터페이스 ──────────────────────────────────────────────────
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

    # ── 사전 계산: 쿠빅 스플라인 → 곡률·헤딩 배열 ────────────────────────
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

        cs_x = CubicSpline(s_ext, x_ext, bc_type='periodic')
        cs_y = CubicSpline(s_ext, y_ext, bc_type='periodic')

        dx  = cs_x(s, 1);  dy  = cs_y(s, 1)
        ddx = cs_x(s, 2);  ddy = cs_y(s, 2)

        denom = np.maximum((dx**2 + dy**2) ** 1.5, 1e-6)
        self._curvature = np.abs(dx * ddy - dy * ddx) / denom
        self._heading   = np.arctan2(dy, dx)

        self.get_logger().info(
            f'스플라인 사전 계산: 곡률 max={self._curvature.max():.3f} '
            f'mean={self._curvature.mean():.3f} 1/m')

    # ── 사전 계산: 역방향 스윕 속도 프로파일 ─────────────────────────────
    def _precompute_speed_profile(self):
        n  = len(self.wp)
        ds = np.array([
            np.linalg.norm(self.wp[(i + 1) % n, :2] - self.wp[i, :2])
            for i in range(n)
        ])

        self._v_profile = self.wp[:, 2].copy() * self.SPEED_SCALE  # 스케일 먼저

        kappa = np.maximum(self._curvature, 1e-3)
        v_curv_cap = np.sqrt(self.MU * 9.81 / kappa)
        self._v_profile = np.minimum(self._v_profile, v_curv_cap)

        for _ in range(3):
            for i in range(n - 1, -1, -1):
                j = (i + 1) % n
                v_lim = np.sqrt(self._v_profile[j] ** 2 + 2.0 * self.A_BRAKE * ds[i])
                self._v_profile[i] = min(self._v_profile[i], v_lim)

        self.get_logger().info(
            f'속도 프로파일: {self._v_profile.min():.2f}~{self._v_profile.max():.2f} m/s '
            f'(SPEED_SCALE={self.SPEED_SCALE})')

    # ── 사전 계산: Pacejka 타이어 모델 역변환 LUT ─────────────────────────
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
            f'Pacejka LUT: Fy=[{Fy_norm[0]:.3f}, {Fy_norm[-1]:.3f}] '
            f'(B={self.PAC_B} C={self.PAC_C} D={self.PAC_D} E={self.PAC_E})')

    def _pacejka_inv(self, Fy_norm_req: float) -> float:
        clamped = float(np.clip(Fy_norm_req,
                                self._pac_Fy_lut[0] * 0.99,
                                self._pac_Fy_lut[-1] * 0.99))
        return float(np.interp(clamped, self._pac_Fy_lut, self._pac_alpha_lut))

    # ── 제어기 분포 이미지 저장 ────────────────────────────────────────────
    def _save_controller_map(self):
        try:
            import yaml
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
            from PIL import Image as PILImage

            yaml_path = os.path.join(_MAPS_DIR, f'{WAYPOINTS_MAP}.yaml')
            with open(yaml_path) as f:
                cfg = yaml.safe_load(f)
            resolution = float(cfg.get('resolution', 0.05))
            origin_raw = cfg.get('origin', [-10.0, -10.0, 0.0])
            ox, oy = float(origin_raw[0]), float(origin_raw[1])

            img_path = None
            for ext in ['.png', '.pgm']:
                p = os.path.join(_MAPS_DIR, f'{WAYPOINTS_MAP}{ext}')
                if os.path.exists(p):
                    img_path = p
                    break
            if img_path is None:
                self.get_logger().warn('제어기 맵: 맵 이미지 없음 → 저장 생략')
                return

            img  = np.array(PILImage.open(img_path).convert('L'))
            h_px = img.shape[0]

            wx = self.wp[:, 0];  wy = self.wp[:, 1]
            px = (wx - ox) / resolution
            py = h_px - (wy - oy) / resolution

            ctrl = np.where(self._curvature >= self.HIGH_CURV, 0, 1)
            n_map     = int((ctrl == 0).sum())
            n_stanley = int((ctrl == 1).sum())

            COLORS = {0: ('red',       'MAP'),
                      1: ('limegreen', 'Stanley')}

            fig, (ax_map, ax_curv) = plt.subplots(
                2, 1, figsize=(12, 14),
                gridspec_kw={'height_ratios': [4, 1]})

            ax_map.imshow(img, cmap='gray', origin='upper', alpha=0.7)
            for cid, (color, label) in COLORS.items():
                mask = ctrl == cid
                if mask.any():
                    ax_map.scatter(px[mask], py[mask],
                                   c=color, s=8, label=label,
                                   linewidths=0, zorder=3)
            ax_map.legend(loc='lower right', fontsize=12, framealpha=0.9)
            ax_map.set_title(
                f'{WAYPOINTS_MAP}  Controller Distribution\n'
                f'MAP {n_map}pts  Stanley {n_stanley}pts',
                fontsize=13)
            ax_map.axis('off')

            idx = np.arange(len(self.wp))
            ax_curv.plot(idx, self._curvature, color='gray', linewidth=1.0, label='곡률')
            ax_curv.axhline(self.HIGH_CURV, color='red', linestyle='--',
                            linewidth=1.2, label=f'HIGH_CURV={self.HIGH_CURV}')
            ax_curv.fill_between(idx, self.HIGH_CURV, self._curvature,
                                 where=self._curvature >= self.HIGH_CURV,
                                 color='red', alpha=0.2)
            ax_curv.fill_between(idx, 0, self._curvature,
                                 where=self._curvature < self.HIGH_CURV,
                                 color='limegreen', alpha=0.2)
            ax_curv.set_xlim(0, len(self.wp))
            ax_curv.set_ylim(0, self._curvature.max() * 1.15)
            ax_curv.set_xlabel('Waypoint Index', fontsize=10)
            ax_curv.set_ylabel('Curvature (1/m)', fontsize=10)
            ax_curv.legend(loc='upper right', fontsize=9, framealpha=0.9)
            ax_curv.grid(True, alpha=0.3)

            plt.tight_layout()
            out_path = os.path.join(_MAPS_DIR, f'{WAYPOINTS_MAP}_controller.jpg')
            plt.savefig(out_path, dpi=150, bbox_inches='tight')
            plt.close()
            self.get_logger().info(f'제어기 맵 저장: {out_path}')

        except Exception as e:
            self.get_logger().warn(f'제어기 맵 저장 실패: {e}')

    # ── 경로 시각화 (RViz) ─────────────────────────────────────────────────
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

        v_min = self.wp[:, 2].min()
        v_max = self.wp[:, 2].max()
        v_rng = max(v_max - v_min, 0.1)
        markers = MarkerArray()
        for i, (x, y, v) in enumerate(self.wp):
            t = (v - v_min) / v_rng
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

    # ── CTE + 헤딩 오차 계산 ──────────────────────────────────────────────
    def _compute_cte_and_heading_err(self, px, py, yaw, near_idx):
        n  = len(self.wp)
        p1 = self.wp[near_idx, :2]
        p2 = self.wp[(near_idx + 1) % n, :2]

        tangent = p2 - p1
        t_len   = np.linalg.norm(tangent)
        if t_len < 1e-6:
            return 0.0, 0.0
        tangent /= t_len
        normal  = np.array([-tangent[1], tangent[0]])

        cte         = float(np.dot(np.array([px, py]) - p1, normal))
        path_yaw    = self._heading[near_idx]
        heading_err = float(np.arctan2(np.sin(yaw - path_yaw),
                                       np.cos(yaw - path_yaw)))
        return cte, heading_err

    # ── MAP 조향각 계산 ────────────────────────────────────────────────────
    def _steer_map(self, pos, yaw, near_idx):
        v  = max(self._cur_speed, 0.3)
        L1 = max(self.MAP_L1_MIN, self.MAP_L1_K * v)

        goal, _, _ = self._find_lookahead(pos, L1, yaw)
        if goal is None:
            return None

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

    # ── Stanley 조향각 계산 ────────────────────────────────────────────────
    def _steer_stanley(self, cte, heading_err):
        v = max(self._cur_speed, 0.3)
        return float(-(heading_err + np.arctan(self.STANLEY_K * cte / v)))

    # ── 오도메트리 콜백 ────────────────────────────────────────────────────
    def _odom_cb(self, msg):
        px  = msg.pose.pose.position.x
        py  = msg.pose.pose.position.y
        q   = msg.pose.pose.orientation
        yaw = np.arctan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y ** 2 + q.z ** 2))

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self._cur_speed = float(np.hypot(vx, vy))

        lookahead_base = max(0.5, self.LA_BASE + self.LA_K * self._cur_speed)
        _, _, near_idx = self._find_lookahead(
            np.array([px, py]), lookahead_base, yaw)

        target_speed = float(self._v_profile[near_idx])

        cte, heading_err = self._compute_cte_and_heading_err(px, py, yaw, near_idx)
        curv = self._curvature[near_idx]
        pos  = np.array([px, py])

        # ── 제어기 선택 ───────────────────────────────────────────────────
        if abs(cte) >= self.CTE_RECOVER_THRESH or curv >= self.HIGH_CURV:
            steer = self._steer_map(pos, yaw, near_idx)
            ctrl_name = 'MAP'
            if steer is None:
                steer     = self._steer_stanley(cte, heading_err)
                ctrl_name = 'Stanley(fallback)'
        else:
            steer     = self._steer_stanley(cte, heading_err)
            ctrl_name = 'Stanley'

        # ── 경계 소프트 블렌딩 (Stanley ↔ MAP) ───────────────────────────
        lo = self.HIGH_CURV - self.BLEND_MARGIN
        hi = self.HIGH_CURV + self.BLEND_MARGIN
        if lo < curv < hi:
            t   = (curv - lo) / (2 * self.BLEND_MARGIN)
            s_m = self._steer_map(pos, yaw, near_idx) or steer
            s_s = self._steer_stanley(cte, heading_err)
            steer     = (1.0 - t) * s_s + t * s_m
            ctrl_name = f'blend(Stanley↔MAP t={t:.2f})'

        steer = float(np.clip(steer, -self.MAX_STEER, self.MAX_STEER))
        steer = self.STEER_ALPHA * self._steer_prev + (1.0 - self.STEER_ALPHA) * steer
        self._steer_prev = steer

        self.get_logger().info(
            f'[{ctrl_name}] κ={curv:.3f} CTE={cte:.3f}m '
            f'steer={np.degrees(steer):.1f}°  spd={target_speed:.2f}m/s')

        cmd = AckermannDriveStamped()
        cmd.drive.steering_angle = steer
        cmd.drive.speed          = float(target_speed)
        self.drive_pub.publish(cmd)

    # ── Adaptive Lookahead 포인트 탐색 (아크 길이 기반 보간) ──────────────
    def _find_lookahead(self, pos, lookahead, yaw=None):
        dists = np.linalg.norm(self.wp[:, :2] - pos, axis=1)
        n     = len(self.wp)
        search_range = 50

        if not self._initialized:
            near = int(np.argmin(dists))
            if yaw is not None:
                fwd    = np.array([np.cos(yaw), np.sin(yaw)])
                wp_dir = self.wp[(near + 1) % n, :2] - self.wp[near, :2]
                if np.dot(fwd, wp_dir) < 0:
                    self.get_logger().warn(
                        '웨이포인트 방향이 차량 헤딩과 반대 → 배열 역전')
                    self.wp         = self.wp[::-1]
                    self._curvature = self._curvature[::-1]
                    self._heading   = self._heading[::-1]
                    self._v_profile = self._v_profile[::-1]
                    near = int(n - 1 - near)
            self._initialized = True
        else:
            candidates = [(self._near_idx + i) % n for i in range(search_range)]
            near = candidates[int(np.argmin([dists[i] for i in candidates]))]

        self._near_idx = near

        arc = 0.0
        for i in range(n):
            i0  = (near + i)     % n
            i1  = (near + i + 1) % n
            seg = float(np.linalg.norm(self.wp[i1, :2] - self.wp[i0, :2]))
            if arc + seg >= lookahead:
                alpha    = float(np.clip((lookahead - arc) / max(seg, 1e-6), 0.0, 1.0))
                goal_xy  = (1.0 - alpha) * self.wp[i0, :2] + alpha * self.wp[i1, :2]
                goal_spd = (1.0 - alpha) * self.wp[i0, 2]  + alpha * self.wp[i1, 2]
                return goal_xy, goal_spd, near
            arc += seg

        return None, None, near


def main(args=None):
    rclpy.init(args=args)
    node = HybridController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
