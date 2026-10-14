#!/usr/bin/env python3
"""
Pure Pursuit 자율주행 컨트롤러
- 속도 비례 adaptive lookahead
- 전방 감속 (코너 진입 전 미리 감속)
- 비상 회피: 측면 벽 보정 · 충돌 후진 · Stuck→FTG
"""

import rclpy
from rclpy.node import Node
import numpy as np
import csv
import os
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA


def _find_repo_root():
    """sim_ws/ 와 f1tenth_ws/ 를 모두 포함하는 디렉토리를 repo 루트로 반환."""
    d = os.path.dirname(os.path.abspath(__file__))
    for _ in range(12):
        if os.path.isdir(os.path.join(d, 'sim_ws')) and os.path.isdir(os.path.join(d, 'f1tenth_ws')):
            return d
        d = os.path.dirname(d)
    raise RuntimeError('repo root (sim_ws/ + f1tenth_ws/) 를 찾을 수 없습니다')


WAYPOINTS_MAP = 'cool_clean'   # setup_sim.py 가 이 이름만 업데이트합니다
_MAPS_DIR      = os.path.join(_find_repo_root(), 'sim_ws', 'src', 'f1tenth_gym_ros', 'maps')
WAYPOINTS_PATH = os.path.join(_MAPS_DIR, f'{WAYPOINTS_MAP}_waypoints.csv')


class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit')
        self._near_idx = 0
        self._initialized = False

        # ── 튜닝 파라미터 ──────────────────────────────────────────────────
        self.WHEELBASE    = 0.33
        self.SPEED_SCALE  = 1.1057
        self.MAX_STEER    = 0.4

        # Adaptive lookahead: L = LA_BASE + LA_K * v
        self.LA_BASE      = 0.5931
        self.LA_K         = 0.1234

        # 제동 거리 기반 전방 감속
        self.PREVIEW_N    = 79
        self.A_BRAKE      = 9.4675

        # 조향 저역통과 필터
        self.STEER_ALPHA  = 0.40
        self._steer_prev  = 0.0

        # ── 비상 회피 파라미터 ─────────────────────────────────────────────
        self.WALL_SIDE_THRESH  = 0.5
        self.WALL_BIAS_GAIN    = 0.4
        self.COLLISION_THRESH  = 0.25
        self.BRAKE_START_DIST  = 2.0
        self.REVERSE_SPEED     = 0.8
        self.REVERSE_DURATION  = 3.5

        # ── Stuck → FTG 전환 파라미터 ──────────────────────────────────────
        self.STUCK_TIME        = 3.0
        self.STUCK_DIST_THRESH = 0.15

        # ── FTG 파라미터 ───────────────────────────────────────────────────
        self.FTG_GAP_THRESH    = 0.5
        self.FTG_BUBBLE_R      = 0.3
        self.FTG_SPEED_MIN     = 0.5
        self.FTG_SPEED_MAX     = 3.0
        self.FTG_SPEED_GAIN    = 1.0
        self.FTG_STEER_GAIN    = 1.0

        # ── 상태 머신 ──────────────────────────────────────────────────────
        self._drive_state          = 'NORMAL'
        self._reverse_start        = None
        self._reverse_return_state = 'NORMAL'
        self._scan_data            = None
        self._pos_history          = []

        # ── 웨이포인트 로드 ────────────────────────────────────────────────
        self.wp = self._load_waypoints(WAYPOINTS_PATH)
        self.get_logger().info(f'웨이포인트 {len(self.wp)}개 로드 완료')

        self._cur_speed = 0.0

        # ── ROS 인터페이스 ─────────────────────────────────────────────────
        self.drive_pub  = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.path_pub   = self.create_publisher(Path, '/waypoints_path', 1)
        self.marker_pub = self.create_publisher(MarkerArray, '/waypoints_markers', 1)
        self.create_subscription(Odometry, '/ego_racecar/odom', self._odom_cb, 10)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)

        self.create_timer(1.0, self._publish_path)
        self._path_published = False

        input('Enter 키를 누르면 주행을 시작합니다...')
        self._pos_history.clear()

    # ── 웨이포인트 로드 ────────────────────────────────────────────────────
    def _load_waypoints(self, path):
        pts = []
        with open(path) as f:
            for row in csv.DictReader(f):
                pts.append([float(row['x']), float(row['y']), float(row['speed'])])
        return np.array(pts)

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
            m.color = ColorRGBA(r=1.0-t, g=t, b=0.0, a=0.8)
            markers.markers.append(m)
        self.marker_pub.publish(markers)

        if not self._path_published:
            self.get_logger().info('경로 시각화 완료')
            self._path_published = True

    # ── LiDAR 콜백 + 비상 회피 ────────────────────────────────────────────
    def _scan_cb(self, msg):
        self._scan_data = msg
        if self._drive_state == 'REVERSING':
            return
        front_min = self._front_min_dist(msg)
        if front_min < self.COLLISION_THRESH:
            self.get_logger().warn(
                f'충돌 감지! 전방 최소 거리 {front_min:.2f}m → 후진 시작')
            self._reverse_return_state = self._drive_state
            self._drive_state          = 'REVERSING'
            self._reverse_start        = self.get_clock().now()

    def _front_min_dist(self, msg):
        angles = np.arange(len(msg.ranges)) * msg.angle_increment + msg.angle_min
        ranges = np.array(msg.ranges, dtype=float)
        valid  = np.isfinite(ranges) & (ranges > msg.range_min) & (ranges < msg.range_max)
        front  = valid & (np.abs(angles) < np.radians(30))
        return float(ranges[front].min()) if front.any() else float('inf')

    def _side_bias_correction(self):
        """좌/우 벽 거리 불균형 시 반대 방향 조향 보정값(rad) 반환."""
        if self._scan_data is None:
            return 0.0
        msg    = self._scan_data
        angles = np.arange(len(msg.ranges)) * msg.angle_increment + msg.angle_min
        ranges = np.array(msg.ranges, dtype=float)
        valid  = np.isfinite(ranges) & (ranges > msg.range_min) & (ranges < msg.range_max)

        left_mask  = valid & (angles >  np.radians(60)) & (angles <  np.radians(120))
        right_mask = valid & (angles > -np.radians(120)) & (angles < -np.radians(60))

        left_dist  = float(np.median(ranges[left_mask]))  if left_mask.any()  else self.WALL_SIDE_THRESH
        right_dist = float(np.median(ranges[right_mask])) if right_mask.any() else self.WALL_SIDE_THRESH

        correction = 0.0
        if left_dist < self.WALL_SIDE_THRESH:
            correction += self.WALL_BIAS_GAIN * (self.WALL_SIDE_THRESH - left_dist) / self.WALL_SIDE_THRESH
        if right_dist < self.WALL_SIDE_THRESH:
            correction -= self.WALL_BIAS_GAIN * (self.WALL_SIDE_THRESH - right_dist) / self.WALL_SIDE_THRESH
        return float(np.clip(correction, -0.3, 0.3))

    def _handle_reversing(self):
        elapsed = (self.get_clock().now() - self._reverse_start).nanoseconds * 1e-9
        if elapsed >= self.REVERSE_DURATION:
            self._drive_state = self._reverse_return_state
            self._initialized = False
            self._pos_history.clear()
            self.get_logger().info(f'후진 완료 → {self._drive_state} 복귀')
            return
        cmd = AckermannDriveStamped()
        cmd.drive.speed          = -self.REVERSE_SPEED
        cmd.drive.steering_angle = 0.0
        self.drive_pub.publish(cmd)

    def _check_stuck(self, px, py):
        """(is_stuck, displacement_m) 반환."""
        now_ns = self.get_clock().now().nanoseconds
        self._pos_history.append((now_ns, px, py))
        cutoff_ns = now_ns - int(self.STUCK_TIME * 1e9)
        self._pos_history = [(t, x, y) for t, x, y in self._pos_history if t >= cutoff_ns]
        if len(self._pos_history) < 2:
            return False, 0.0
        oldest_ns, ox, oy = self._pos_history[0]
        if (now_ns - oldest_ns) * 1e-9 < self.STUCK_TIME * 0.9:
            return False, 0.0
        disp = float(np.hypot(px - ox, py - oy))
        return disp < self.STUCK_DIST_THRESH, disp

    def _follow_the_gap(self):
        if self._scan_data is None:
            return
        msg    = self._scan_data
        angles = np.arange(len(msg.ranges)) * msg.angle_increment + msg.angle_min
        ranges = np.array(msg.ranges, dtype=float)

        front  = np.abs(angles) < np.radians(90)
        r      = np.where(np.isfinite(ranges), ranges, msg.range_max)
        r      = np.clip(r, 0.0, msg.range_max)
        r[~front] = 0.0

        front_valid = front & np.isfinite(ranges) & (ranges > msg.range_min)
        if front_valid.any():
            min_idx     = int(np.argmin(np.where(front_valid, ranges, np.inf)))
            bubble_half = max(1, int(self.FTG_BUBBLE_R / max(msg.angle_increment, 1e-6)))
            r[max(0, min_idx - bubble_half): min_idx + bubble_half + 1] = 0.0

        best = (0, 0, 0)
        cur_start, cur_len = None, 0
        for i, rv in enumerate(r):
            if rv > self.FTG_GAP_THRESH:
                if cur_start is None:
                    cur_start = i
                cur_len += 1
                if cur_len > best[2]:
                    best = (cur_start, i, cur_len)
            else:
                cur_start, cur_len = None, 0

        gs, ge, gl = best
        if gl == 0:
            cmd = AckermannDriveStamped()
            cmd.drive.speed = 0.0
            self.drive_pub.publish(cmd)
            return

        best_i       = gs + int(np.argmax(r[gs:ge + 1]))
        target_angle = float(angles[best_i])

        front_r   = ranges[front_valid]
        front_min = float(front_r.min()) if len(front_r) > 0 else msg.range_max
        speed     = float(np.clip(front_min * self.FTG_SPEED_GAIN,
                                  self.FTG_SPEED_MIN, self.FTG_SPEED_MAX))

        steer = float(np.clip(target_angle * self.FTG_STEER_GAIN, -self.MAX_STEER, self.MAX_STEER))
        steer = self.STEER_ALPHA * self._steer_prev + (1.0 - self.STEER_ALPHA) * steer
        self._steer_prev = steer

        self.get_logger().info(
            f'[FTG] 갭[{gs}:{ge}] 목표각={np.degrees(target_angle):.1f}°  '
            f'전방최소={front_min:.2f}m  spd={speed:.2f}')

        cmd = AckermannDriveStamped()
        cmd.drive.steering_angle = steer
        cmd.drive.speed          = speed
        self.drive_pub.publish(cmd)

    # ── 오도메트리 콜백 ────────────────────────────────────────────────────
    def _odom_cb(self, msg):
        if self._drive_state == 'REVERSING':
            self._handle_reversing()
            return

        if self._drive_state == 'FTG':
            self._follow_the_gap()
            return

        px  = msg.pose.pose.position.x
        py  = msg.pose.pose.position.y
        q   = msg.pose.pose.orientation
        yaw = np.arctan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y**2 + q.z**2))

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self._cur_speed = float(np.hypot(vx, vy))

        lookahead = self.LA_BASE + self.LA_K * self._cur_speed
        lookahead = max(0.5, lookahead)

        goal, wp_speed, near_idx = self._find_lookahead(
            np.array([px, py]), lookahead, yaw)
        if goal is None:
            return

        # ── 제동 거리 기반 속도 계산 ─────────────────────────────────────
        n = len(self.wp)
        dist_accum    = float(np.linalg.norm(self.wp[near_idx, :2] - np.array([px, py])))
        v_brake_limit = wp_speed

        for i in range(self.PREVIEW_N):
            cur  = self.wp[(near_idx + i)     % n, :2]
            nxt  = self.wp[(near_idx + i + 1) % n, :2]
            dist_accum += float(np.linalg.norm(nxt - cur))
            v_corner = self.wp[(near_idx + i + 1) % n, 2]
            v_limit  = float(np.sqrt(v_corner**2 + 2.0 * self.A_BRAKE * dist_accum))
            v_brake_limit = min(v_brake_limit, v_limit)

        target_speed = min(wp_speed, v_brake_limit) * self.SPEED_SCALE

        # ── 전방 장애물 비례 제동 ─────────────────────────────────────────
        if self._scan_data is not None:
            front_dist = self._front_min_dist(self._scan_data)
            if front_dist < self.BRAKE_START_DIST:
                brake_ratio = (front_dist - self.COLLISION_THRESH) / \
                              (self.BRAKE_START_DIST - self.COLLISION_THRESH)
                target_speed *= float(np.clip(brake_ratio, 0.0, 1.0))

        # ── Pure Pursuit 조향각 계산 ──────────────────────────────────────
        dx = goal[0] - px
        dy = goal[1] - py
        lx =  np.cos(yaw) * dx + np.sin(yaw) * dy
        ly = -np.sin(yaw) * dx + np.cos(yaw) * dy

        L = np.hypot(lx, ly)
        if L < 1e-4:
            return

        steer = np.arctan2(2.0 * self.WHEELBASE * ly, L ** 2)
        steer += self._side_bias_correction()
        steer = float(np.clip(steer, -self.MAX_STEER, self.MAX_STEER))
        steer = self.STEER_ALPHA * self._steer_prev + (1.0 - self.STEER_ALPHA) * steer
        self._steer_prev = steer

        # ── Stuck 감지 → FTG 전환 ────────────────────────────────────────
        stuck, stuck_disp = self._check_stuck(px, py)
        if stuck:
            self.get_logger().warn(
                f'[STUCK→FTG] {self.STUCK_TIME}s 동안 변위 {stuck_disp:.3f}m → FTG 전환')
            self._drive_state = 'FTG'
            self._pos_history.clear()
            return

        self.get_logger().info(
            f'[PURSUIT] near_idx={near_idx}  '
            f'lookahead={lookahead:.2f}m  spd={target_speed:.2f}m/s  '
            f'steer={np.degrees(steer):.1f}°')

        cmd = AckermannDriveStamped()
        cmd.drive.steering_angle = steer
        cmd.drive.speed          = float(target_speed)
        self.drive_pub.publish(cmd)

    # ── Adaptive Lookahead 포인트 탐색 (아크 길이 기반 보간) ────────────────
    def _find_lookahead(self, pos, lookahead, yaw=None):
        dists = np.linalg.norm(self.wp[:, :2] - pos, axis=1)
        n = len(self.wp)
        search_range = 50

        if not self._initialized:
            near = int(np.argmin(dists))
            if yaw is not None:
                fwd    = np.array([np.cos(yaw), np.sin(yaw)])
                wp_dir = self.wp[(near + 1) % n, :2] - self.wp[near, :2]
                if np.dot(fwd, wp_dir) < 0:
                    self.get_logger().warn(
                        '웨이포인트 방향이 차량 헤딩과 반대 → 배열 역전')
                    self.wp = self.wp[::-1]
                    near = int(n - 1 - near)
            self._initialized = True
        else:
            candidates = [(self._near_idx + i) % n for i in range(search_range)]
            near = candidates[int(np.argmin([dists[i] for i in candidates]))]

        self._near_idx = near

        arc = 0.0
        for i in range(n):
            i0 = (near + i) % n
            i1 = (near + i + 1) % n
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
    node = PurePursuit()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
