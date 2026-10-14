#!/usr/bin/env python3
"""
Stanley 경로 추종 컨트롤러 (gym 시뮬 전용)

race_script.yaml [stanley] 섹션과 동일한 파라미터를 사용한다.
  - 어댑티브 룩어헤드 (L = (Lmax-Lmin)/Lgain * v + Lmin)
  - 중심차분 접선 기반 path_yaw → CTE 정밀도 향상
  - 곡률 피드포워드 (ff_gain * atan(wheelbase * κ), ff_preview_sec 예견)
  - 조향 LPF (steer_lpf)

실차와의 차이:
  - 위치: /ego_racecar/odom 직접 사용 (AMCL 불필요, 시뮬은 ground-truth)
  - 장애물 회피 없음 (Q2 로직 제외)
  - /race/enabled 게이트 없음 — Enter 후 즉시 출발
"""

import csv
import math
import os

import numpy as np

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

from f1tenth_racing.pure_pursuit_node import WAYPOINTS_MAP, _MAPS_DIR

WAYPOINTS_PATH = os.path.join(_MAPS_DIR, f'{WAYPOINTS_MAP}_waypoints.csv')


def _normalize(a: float) -> float:
    while a >  math.pi: a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a


class StanleyControllerNode(Node):
    def __init__(self):
        super().__init__('stanley_controller')
        self._initialized = False

        # ── 차량 파라미터 (race_script.yaml sim_vehicle 기준) ──────────────
        self.WHEELBASE  = 0.33
        self.MAX_STEER  = 0.36      # sim_vehicle.s_max

        # ── Stanley 파라미터 (race_script.yaml [stanley] 과 동일) ───────────
        self.GAIN       = 1.6       # stanley.gain
        self.K_CTE      = 1.8       # stanley.k
        self.KS         = 1.5       # stanley.ks
        self.VEL_SCALE  = 1.0       # stanley.vel_scale
        self.FF_GAIN    = 1.2       # stanley.ff_gain
        self.FF_PREV_S  = 0.38      # stanley.ff_preview_sec
        self.STEER_LPF  = 1.0       # stanley.steer_lpf  (1.0 = off)
        self.L_MIN      = 0.5       # stanley.lookahead_min
        self.L_MAX      = 2.5       # stanley.lookahead_max
        self.L_GAIN     = 7.0       # stanley.lookahead_gain
        self.INTERP     = 20        # stanley.interp_scale

        # ── 상태 ────────────────────────────────────────────────────────────
        self._cur_speed  = 0.0
        self._steer_prev = 0.0

        # ── 웨이포인트 로드 ─────────────────────────────────────────────────
        self.wp = self._load_waypoints(WAYPOINTS_PATH)
        self.get_logger().info(f'웨이포인트 {len(self.wp)}개 로드')

        # ── 곡률 사전계산 (피드포워드용) ────────────────────────────────────
        self._kappa, self._wp_ds = self._precompute_curvature()
        self.get_logger().info(
            f'곡률 사전계산: |κ|max={np.abs(self._kappa).max():.3f}  '
            f'ds={self._wp_ds:.3f}m')

        # ── ROS 인터페이스 ──────────────────────────────────────────────────
        self.drive_pub  = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.path_pub   = self.create_publisher(Path, '/waypoints_path', 1)
        self.marker_pub = self.create_publisher(MarkerArray, '/waypoints_markers', 1)
        self.wp_pub     = self.create_publisher(Marker, '/waypoint', 10)
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

    # ── 곡률 사전계산 (실차 stanley_controller.py 의 _precompute_curvature 동일) ──
    def _precompute_curvature(self, span: float = 0.2, smooth: int = 5):
        w  = self.wp[:, :2]
        ds = float(np.mean(np.hypot(*(np.roll(w, -1, axis=0) - w).T)))
        m  = max(1, int(round(span / max(ds, 1e-6))))
        a  = np.roll(w,  m, axis=0)
        c  = np.roll(w, -m, axis=0)
        ab, bc, ac = w - a, c - w, c - a
        cross = ab[:, 0] * bc[:, 1] - ab[:, 1] * bc[:, 0]
        denom = (np.linalg.norm(ab, axis=1) * np.linalg.norm(bc, axis=1)
                 * np.linalg.norm(ac, axis=1))
        kappa = 2.0 * cross / np.maximum(denom, 1e-9)
        kern  = np.ones(smooth) / smooth
        pad   = np.concatenate([kappa[-smooth:], kappa, kappa[:smooth]])
        kappa = np.convolve(pad, kern, 'same')[smooth:-smooth]
        return kappa, ds

    # ── 경로 시각화 ────────────────────────────────────────────────────────
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

    # ── 오도메트리 콜백 ────────────────────────────────────────────────────
    def _odom_cb(self, msg: Odometry):
        px  = msg.pose.pose.position.x
        py  = msg.pose.pose.position.y
        q   = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y**2 + q.z**2))

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self._cur_speed = float(np.hypot(vx, vy))

        steer, speed = self._compute_cmd(px, py, yaw)

        cmd = AckermannDriveStamped()
        cmd.drive.steering_angle = steer
        cmd.drive.speed          = speed
        self.drive_pub.publish(cmd)

    # ── 조향·속도 계산 (실차 _compute_cmd 와 동일한 로직) ─────────────────
    def _compute_cmd(self, px: float, py: float, yaw: float):
        wps = self.wp
        n   = len(wps)
        v   = abs(self._cur_speed)

        # ── 어댑티브 룩어헤드 ────────────────────────────────────────────
        L = float(np.clip(
            (self.L_MAX - self.L_MIN) / self.L_GAIN * v + self.L_MIN,
            self.L_MIN, self.L_MAX))

        # ── 최근접 웨이포인트 ────────────────────────────────────────────
        d = np.hypot(wps[:, 0] - px, wps[:, 1] - py)

        if not self._initialized:
            nearest = int(np.argmin(d))
            # 진행 방향 확인 — 반대면 웨이포인트 배열 역전
            fwd    = np.array([math.cos(yaw), math.sin(yaw)])
            wp_dir = wps[(nearest + 1) % n, :2] - wps[nearest, :2]
            if np.dot(fwd, wp_dir) < 0:
                self.wp         = self.wp[::-1]
                self._kappa     = self._kappa[::-1]
                wps = self.wp
                nearest = int(n - 1 - nearest)
                self.get_logger().warn('웨이포인트 방향 역전')
            self._near_idx   = nearest
            self._initialized = True
        else:
            # 이전 최근접에서 50개 앞까지만 탐색
            cands = [(self._near_idx + i) % n for i in range(50)]
            nearest = cands[int(np.argmin([d[i] for i in cands]))]
            self._near_idx = nearest

        # ── 룩어헤드 목표점 보간 ─────────────────────────────────────────
        traj_d  = np.hypot(wps[:, 0] - wps[nearest, 0],
                            wps[:, 1] - wps[nearest, 1])
        seg_end = nearest
        for _ in range(n):
            if traj_d[seg_end] > L:
                break
            seg_end = (seg_end + 1) % n
        seg_beg = (seg_end - 1 + n) % n

        xs = np.linspace(wps[seg_beg, 0], wps[seg_end, 0], self.INTERP)
        ys = np.linspace(wps[seg_beg, 1], wps[seg_end, 1], self.INTERP)
        i_best = int(np.argmin(np.abs(np.hypot(xs - px, ys - py) - L)))
        target = np.array([xs[i_best], ys[i_best]])

        # ── 중심차분 접선 기반 path_yaw ──────────────────────────────────
        # 현(chord)이 아닌 접선을 쓴다 — 코너에서 안쪽 파고드는 문제 해소
        m_t   = max(1, int(round(0.1 / self._wp_ds)))
        j_f   = (nearest + m_t) % n
        j_b   = (nearest - m_t) % n
        path_yaw = math.atan2(wps[j_f, 1] - wps[j_b, 1],
                              wps[j_f, 0] - wps[j_b, 0])

        # ── CTE (부호 있는 횡오차) ────────────────────────────────────────
        closest  = wps[nearest, :2]
        cte_size = float(np.hypot(px - closest[0], py - closest[1]))
        cte_sign = float(np.sign(np.cross(
            [math.cos(path_yaw), math.sin(path_yaw)],
            [px - closest[0],    py - closest[1]])))
        cte     = cte_size * cte_sign
        yaw_err = _normalize(path_yaw - yaw)

        # ── 곡률 피드포워드 ──────────────────────────────────────────────
        prev_m   = int(v * self.FF_PREV_S / max(self._wp_ds, 1e-6))
        kap      = float(self._kappa[(nearest + prev_m) % n])
        steer_ff = self.FF_GAIN * math.atan(self.WHEELBASE * kap)

        # ── Stanley 조향 계산 ────────────────────────────────────────────
        v_safe = max(v, 0.5)
        steer  = steer_ff + self.GAIN * (
            yaw_err - math.atan2(self.K_CTE * cte, v_safe + self.KS))

        steer = float(np.clip(steer, -self.MAX_STEER, self.MAX_STEER))

        # ── 조향 저역통과 ────────────────────────────────────────────────
        steer = self.STEER_LPF * steer + (1.0 - self.STEER_LPF) * self._steer_prev
        self._steer_prev = steer

        # ── 목표 속도 ────────────────────────────────────────────────────
        cmd_v = float(wps[nearest, 2]) * self.VEL_SCALE

        self.get_logger().info(
            f'[Stanley] L={L:.2f}m CTE={cte:.3f}m yaw_err={math.degrees(yaw_err):.1f}° '
            f'ff={math.degrees(steer_ff):.1f}° steer={math.degrees(steer):.1f}° '
            f'spd={cmd_v:.2f}m/s', throttle_duration_sec=0.1)

        self._pub_wp_marker(target)
        return steer, cmd_v

    def _pub_wp_marker(self, target: np.ndarray):
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp    = self.get_clock().now().to_msg()
        m.ns = 'target'; m.id = 0
        m.type = Marker.SPHERE; m.action = Marker.ADD
        m.pose.position.x = float(target[0])
        m.pose.position.y = float(target[1])
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.2
        m.color = ColorRGBA(r=0.0, g=0.5, b=1.0, a=1.0)
        self.wp_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = StanleyControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
