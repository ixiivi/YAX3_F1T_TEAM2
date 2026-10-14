#!/usr/bin/env python3
"""
Pure Pursuit — Q1 / Q3용 레이싱라인 추종 컨트롤러.
Q2 장애물 회피(옵션)도 지원한다 — Stanley의 Q2 로직과 동일한 방식
(트랙 경계 마스크 + 예측 통로 판정 + 목표점 좌우 시프트)을 그대로 재사용한다.

waypoints CSV (x, y[, speed]) 를 읽어 lookahead 지점을 추종한다.
위치는 AMCL(/amcl_pose)에서, 장애물 회피에 쓰는 속도·요레이트는 /odom에서 얻는다.

★ 장애물 회피는 기본 꺼짐(off) — obstacle_avoid_enabled 파라미터 하나로
  Stanley의 wall_avoid_enabled 처럼 껐다/켤 수 있다. 끄면 기존과 완전히
  동일하게(장애물 무시하고 레이싱라인만 추종) 동작한다.

구독: /amcl_pose, /odom, /scan(회피 켜졌을 때만), /race/enabled
발행: /drive, /obstacle(회피 켜졌을 때만, RViz 마커)
"""
import csv
import math
import os
import time
from collections import deque

import cv2
import numpy as np
import scipy.spatial.distance
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray
from ackermann_msgs.msg import AckermannDriveStamped

from race_stack.stanley_controller import (_load_map, _world_to_img, _REPO_ROOT,
                                          write_run_config)
from race_stack.speed_profile import (
    solve_profile, preview_speed, curvature_from_path)


# ★ /race/enabled 는 mission_manager 가 **래치(TRANSIENT_LOCAL)** 로 발행한다.
#   기본 QoS(VOLATILE)로 구독하면 구독자가 생기기 전에 나간 값을 못 받는다.
#   race.launch.py 는 컨트롤러를 5~8초 지연 기동하므로 항상 그 상황이고, 그래서
#   출발 게이트가 걸어둔 enabled=false 를 놓쳐 버튼도 안 눌렀는데 출발했다
#   (2026-08-04 실차). 래치로 구독해야 마지막 값을 받는다.
LATCHED_IN = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit')
        # ★ 로그 폴더명(save_run_log)에 쓰는 용도 — 조향/속도 로직과는 무관.
        self.declare_parameter('mode', 'q1')
        # 자이로 기반 오도메트리 (imu_odom). 원본은 '/odom'
        self.declare_parameter('odom_topic', '/odom_imu')
        self.declare_parameter('waypoint_file', '')
        self.declare_parameter('max_speed', 3.0)
        # ★ 박스 위 벤치 모드. true 면 **발행 직전에 속도를 0 으로 못박는다**.
        #   pure_pursuit 는 아직 clip(v, min, max) 를 쓰지만 그것에 기대지 않는다 —
        #   map/stanley 는 그 클램프가 사라져 2026-08-09 에 box 가 풀속도로 주행했다.
        self.declare_parameter('bench', False)
        self.declare_parameter('min_speed', 1.0)
        self.declare_parameter('lookahead', 1.8)
        self.declare_parameter('lookahead_speed_gain', 0.3)
        self.declare_parameter('wheelbase', 0.33)
        self.declare_parameter('max_steer', 0.36)
        self.declare_parameter('curvature_slowdown', 1.5)
        # MAP 과 같은 속도·같은 L 로 A/B 하기 위한 스위치. yaml 로 끈다.
        self.declare_parameter('use_profile', True)
        self.declare_parameter('adaptive_ld', True)
        self.declare_parameter('l1_min', 1.4983)
        self.declare_parameter('l1_k', 0.75)
        self.declare_parameter('l1_kappa', 0.0)
        self.declare_parameter('grip_util', 0.6)
        self.declare_parameter('mu', 0.348)
        self.declare_parameter('pac_d', 1.15)
        self.declare_parameter('lf', 0.1757)
        self.declare_parameter('lr', 0.1441)
        self.declare_parameter('a_lon_acc', 0.0)
        self.declare_parameter('a_lon_brk', 0.0)
        self.declare_parameter('hard_cap', 0.0)
        self.declare_parameter('speed_lag', 0.25)

        # ── Q2 장애물 회피 (옵션) ────────────────────────────────────────
        # ★ Stanley 의 wall_avoid_enabled 와 같은 방식 — yaml 값 하나로 on/off.
        self.declare_parameter('obstacle_avoid_enabled', False)
        self.declare_parameter('map_yaml',        '')
        self.declare_parameter('bound_dir',       '')
        self.declare_parameter('car_width',        0.30)
        self.declare_parameter('obstacle_margin',  0.12)
        self.declare_parameter('obstacle_decel',   0.85)
        self.declare_parameter('obstacle_shift_rate', 0.9)
        self.declare_parameter('obs_corridor_half_width', 0.30)
        self.declare_parameter('obs_release_scale', 1.6)
        self.declare_parameter('obs_horizon_sec',  0.3)
        self.declare_parameter('obs_horizon_min',  0.5)
        self.declare_parameter('obs_horizon_max',  3.0)
        self.declare_parameter('obs_vote_frames',  5)
        self.declare_parameter('laser_offset_x',   0.27)
        self.declare_parameter('laser_offset_y',   0.0)
        self.declare_parameter('grid_xmin',        0.35)
        self.declare_parameter('grid_xmax',        5.0)
        self.declare_parameter('grid_ymin',       -2.5)
        self.declare_parameter('grid_ymax',        2.5)
        self.declare_parameter('grid_res',         0.04)
        self.declare_parameter('grid_safe_dist',   0.3)
        self.declare_parameter('bound_ignore_px',  7.5)

        self.mode = self.get_parameter('mode').value
        wp_file = self.get_parameter('waypoint_file').value
        self.waypoints = self.load_waypoints(wp_file)
        if self.waypoints is None:
            self.get_logger().error(
                f'웨이포인트 파일을 못 읽음: "{wp_file}" — pure_pursuit 정지 상태. '
                f'./race.sh log <이름> 으로 웨이포인트를 먼저 기록하세요.')
        else:
            self.get_logger().info(f'웨이포인트 {len(self.waypoints)}개 로드')

        # 평균 점 간격 — 장애물 회피의 전방경로 예측(_predicted_track_path)에 쓴다.
        self._wp_ds = 0.1
        self._v_profile = None
        self._kappa = None
        self._a_brk = 0.0
        if self.waypoints is not None and len(self.waypoints) > 5:
            w = self.waypoints[:, :2]
            self._wp_ds = float(np.mean(np.hypot(*(np.roll(w, -1, axis=0) - w).T)))
            self._kappa = curvature_from_path(self.waypoints[:, 0],
                                             self.waypoints[:, 1])
            self._build_speed_profile()

        self.enabled = False   # ★ mission_manager 의 /race/enabled(래치)를 받아야 켜진다.
        #   True 로 두면 그 값이 도착하기 전 한 틱이라도 먼저 나가버린다.
        #   mission_manager 가 없는 단독 실행이면 "정지 중: /race/enabled=false"
        #   로그가 이유를 알려준다 — 조용히 달려나가는 것보다 낫다.
        self.last_speed = 0.0
        self.got_pose = False
        self.pos = None
        self.yaw = 0.0
        self.speed = 0.0        # /odom 실측 속도 — 장애물 회피 계산 전용
        self.yaw_rate = 0.0
        self._last_pose_t = None

        # ── Q2 장애물 회피 상태 ──────────────────────────────────────────
        self.obs_on = bool(self.get_parameter('obstacle_avoid_enabled').value)
        self.map_meta      = None
        self.inner_bound   = None
        self.outer_bound   = None
        self.track_mask    = None
        self.lane_free     = True
        self.blocking_pts  = []
        self._shift_side   = 0.0
        self._shift_cur    = 0.0
        self._shift_vec    = None
        self.obs_shift     = self.p('car_width') + self.p('obstacle_margin')
        self.free_votes: deque = deque(maxlen=max(1, int(self.p('obs_vote_frames'))))

        if self.obs_on:
            self._init_obstacle_avoid()

        # ── 실주행 경로 로그 (Stanley 의 save_run_log 와 동일한 방식) ─────
        # 게인 비교 시 "어느 주행인지" 폴더명·gains 스냅샷에 컨트롤러까지
        # 남기려고 pure_pursuit 에도 이식했다. 50Hz 루프 안이 아니라 pose_cb
        # (AMCL 주기)에서만 쌓으므로 stanley 보다 포인트 밀도는 낮다.
        self._trail_log = []
        self._last_trail = None
        self._run_t0 = self.get_clock().now()

        self.create_subscription(Bool, '/race/enabled', self.enabled_cb, LATCHED_IN)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 self.pose_cb, 10)
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self.odom_cb, 10)
        if self.obs_on:
            self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
            self.obs_pub = self.create_publisher(MarkerArray, '/obstacle', 10)

        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        # ── 고정 주기 명령 발행 (20Hz) ─────────────────────────────────
        # AMCL 은 차가 update_min_d(10cm) 이상 움직여야만 /amcl_pose 를 내고,
        # ackermann_mux 는 /drive 가 0.2s 끊기면 명령을 버린다. pose 콜백에서만
        # /drive 를 내면 "pose 1개 → 0.2s 구동 → 10cm 못 감 → pose 없음" 의
        # 데드락으로 차가 출발하지 못한다 (odom kick 데드락과 별개의 두 번째 고리).
        # → pose 콜백은 조향·속도 계산만 하고, 발행은 이 타이머가 계속 한다.
        #   첫 pose 전에는 0속도를 흘려 vesc_to_odom odom kick 역할도 겸한다.
        self._cmd = (0.0, 0.0)      # (steer, speed) — 최근 계산된 명령
        self.create_timer(0.05, self._cmd_tick)

        self.get_logger().info(
            f'PurePursuit ready (obstacle_avoid='
            f'{"on" if self.obs_on else "off"}'
            f' profile={"on" if self._v_profile is not None else "off"}'
            f' adaptive_ld={bool(self.p("adaptive_ld"))})')

    def p(self, name):
        return self.get_parameter(name).value

    def _build_speed_profile(self):
        """MAP 과 같은 solve_profile. use_profile=false 면 CSV 속도를 쓴다."""
        if not bool(self.p('use_profile')) or self.waypoints is None:
            return
        try:
            v, info = solve_profile(
                self.waypoints[:, 0], self.waypoints[:, 1], self.waypoints[:, 2],
                mu_eff=float(self.p('mu')) * float(self.p('pac_d')),
                grip_util=float(self.p('grip_util')),
                lf=float(self.p('lf')), lr=float(self.p('lr')),
                v_min=float(self.p('min_speed')),
                a_lon_acc=float(self.p('a_lon_acc')),
                a_lon_brk=float(self.p('a_lon_brk')),
                hard_cap=float(self.p('hard_cap')),
                curvature=self._kappa)
        except Exception as e:
            self.get_logger().warn(f'solve_profile 실패({e}) → CSV 속도')
            return
        self._v_profile = v
        self._a_brk = float(info.get('a_brk', 0.0) or 0.0)
        self.get_logger().info(
            f"PP 속도 프로파일 η={info['eta']:.2f}  "
            f"v {info['v_min']:.2f}~{info['v_max']:.2f}㎧  "
            f"랩 {info['lap_time']:.1f}s")

    # ── Q2 장애물 회피 초기화 (맵 + 경계 폴리곤 로드, 트랙 마스크 사전계산) ──
    def _init_obstacle_avoid(self):
        map_yaml = self.p('map_yaml')
        bound_dir = self.p('bound_dir') or (
            os.path.join(os.path.dirname(map_yaml),
                         os.path.splitext(os.path.basename(map_yaml))[0])
            if map_yaml else '')

        if map_yaml:
            try:
                _, self.map_meta = _load_map(map_yaml)
                self.get_logger().info('map loaded for obstacle detection')
            except Exception as e:
                self.get_logger().error(f'map load failed: {e}')

        for fname, attr in [('inner_bound.npy', 'inner_bound'),
                             ('outer_bound.npy', 'outer_bound')]:
            fpath = os.path.join(bound_dir, fname)
            if os.path.exists(fpath):
                setattr(self, attr, np.load(fpath))
            else:
                self.get_logger().warn(
                    f'{fname} not found in {bound_dir} — '
                    'obstacle avoidance will run without polygon test')

        self._build_track_mask(float(self.p('bound_ignore_px')))

    def _build_track_mask(self, ign: float):
        self.track_mask = None
        if (self.inner_bound is None or self.outer_bound is None
                or self.map_meta is None):
            return
        h, w = self.map_meta[0], self.map_meta[1]
        outer_m = np.zeros((h, w), np.uint8)
        inner_m = np.zeros((h, w), np.uint8)
        cv2.fillPoly(outer_m, [self.outer_bound.astype(np.int32)], 1)
        cv2.fillPoly(inner_m, [self.inner_bound.astype(np.int32)], 1)
        d_out = cv2.distanceTransform(outer_m, cv2.DIST_L2, 5)
        d_in = cv2.distanceTransform((1 - inner_m).astype(np.uint8),
                                     cv2.DIST_L2, 5)
        self.track_mask = (d_out > ign) & (d_in > ign)
        self.get_logger().info(
            f'track mask built: {int(self.track_mask.sum())} / {h*w} px '
            f'(bound_ignore {ign}px)')

    def _cmd_tick(self):
        out = AckermannDriveStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        if self.enabled and self.got_pose:
            out.drive.steering_angle, out.drive.speed = \
                float(self._cmd[0]), float(self._cmd[1])
        # ★★ bench(박스 위) — 여기가 유일한 강제 지점이다. 옮기거나 지우지 말 것.
        if bool(self.p('bench')):
            out.drive.speed = 0.0
        self.drive_pub.publish(out)

    def load_waypoints(self, path):
        try:
            pts = []
            with open(path) as f:
                for row in csv.reader(f):
                    if not row or row[0].startswith('#'):
                        continue
                    x, y = float(row[0]), float(row[1])
                    v = float(row[2]) if len(row) > 2 else -1.0
                    pts.append((x, y, v))
            return np.array(pts) if pts else None
        except (OSError, ValueError):
            return None

    def enabled_cb(self, msg: Bool):
        if self.enabled and not msg.data:
            self._cmd = (0.0, 0.0)   # 재활성화 시 이전 명령 재사용 방지
            self.publish_stop()
            # 회피 상태도 초기화 — 재활성화 시 낡은 시프트가 남지 않도록.
            self._shift_side = 0.0
            self._shift_cur = 0.0
            self._shift_vec = None
            self.lane_free = True
            self.free_votes.clear()
        self.enabled = msg.data

    def odom_cb(self, msg: Odometry):
        self.speed = msg.twist.twist.linear.x
        self.yaw_rate = msg.twist.twist.angular.z

    def pose_cb(self, msg: PoseWithCovarianceStamped):
        self.got_pose = True
        if not self.enabled or self.waypoints is None:
            return
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        self.pos = np.array([px, py])
        self.yaw = yaw

        now = self.get_clock().now()
        dt = 0.05
        if self._last_pose_t is not None:
            dt = float(np.clip(
                (now - self._last_pose_t).nanoseconds * 1e-9, 0.0, 0.5))
        self._last_pose_t = now

        # 가장 가까운 웨이포인트
        d = np.hypot(self.waypoints[:, 0] - px, self.waypoints[:, 1] - py)
        nearest = int(np.argmin(d))
        n = len(self.waypoints)
        v_now = max(float(self.speed or 0.0), 0.0)

        if bool(self.p('adaptive_ld')):
            L = max(float(self.p('l1_min')), float(self.p('l1_k')) * v_now)
            kap = 0.0 if self._kappa is None else abs(float(self._kappa[nearest]))
            L = L / (1.0 + float(self.p('l1_kappa')) * kap)
        else:
            L = (self.p('lookahead')
                 + self.p('lookahead_speed_gain') * self.last_speed)

        # 가장 가까운 웨이포인트부터 앞으로 진행하며 lookahead 지점 탐색
        target = None
        for k in range(n):
            i = (nearest + k) % n
            if d[i] >= L:
                target = self.waypoints[i]
                break
        if target is None:
            target = self.waypoints[(nearest + n // 4) % n]

        tx, ty, tv = float(target[0]), float(target[1]), float(target[2])

        # ── Q2 장애물 회피: 목표점을 옆으로 시프트 ──────────────────────
        obs_decel_active = False
        if self.obs_on:
            tx, ty = self._obstacle_shift(tx, ty, px, py, yaw, nearest, dt)
            obs_decel_active = not self.lane_free

        # 차량 좌표계로 변환
        dx = tx - px
        dy = ty - py
        lx = math.cos(-yaw) * dx - math.sin(-yaw) * dy
        ly = math.sin(-yaw) * dx + math.cos(-yaw) * dy
        if lx <= 0.0:   # 목표가 뒤에 있으면 강하게 회전만
            steer = self.p('max_steer') * (1.0 if ly > 0 else -1.0)
            curvature = 999.0
        else:
            Ld = math.hypot(lx, ly)
            curvature = 2.0 * ly / (Ld * Ld)
            steer = math.atan(self.p('wheelbase') * curvature)
        steer = float(np.clip(steer, -self.p('max_steer'), self.p('max_steer')))

        # 속도: MAP 과 같은 프로파일이 있으면 그걸, 없으면 CSV / 곡률 감속
        if self._v_profile is not None:
            speed = preview_speed(
                self._v_profile, nearest, v_now, self._wp_ds,
                lag=float(self.p('speed_lag')), a_brk=self._a_brk)
        elif tv > 0:
            speed = min(tv, self.p('max_speed'))
        else:
            speed = self.p('max_speed') / (1.0 + self.p('curvature_slowdown')
                                           * abs(curvature))
        if obs_decel_active:
            speed *= self.p('obstacle_decel')
        cap = float(self.p('hard_cap') or 0.0) or float(self.p('max_speed'))
        speed = float(np.clip(speed, self.p('min_speed'), cap))
        self.last_speed = speed

        self._cmd = (steer, speed)   # 발행은 _cmd_tick(20Hz)이 담당
        self._update_car_trail(px, py, speed)

    # ── Q2 장애물 회피 — Stanley 의 _obstacle_shift 와 동일한 방식 ──────────
    def _obstacle_shift(self, tx: float, ty: float, px: float, py: float,
                        yaw: float, nearest: int, dt: float):
        # 창이 다 찼을 때만 판정 (과반). 히스테리시스로 깜빡임을 막는다.
        if len(self.free_votes) == self.free_votes.maxlen:
            was_free = self.lane_free
            self.lane_free = sum(self.free_votes) * 2 >= self.free_votes.maxlen
            if was_free != self.lane_free:
                if self.lane_free:
                    self.get_logger().info('[obstacle] 경로 확보 — 회피 해제')
                else:
                    self.get_logger().warn(
                        f'[obstacle] 경로 차단! 점 {len(self.blocking_pts)}개 '
                        f'→ 목표점 {self.obs_shift:.2f}m 이동')

        # ── 회피 기준 방향은 경로 법선(map 상에서 안정적) ──────────────
        n_wp = len(self.waypoints)
        m_t = max(1, int(round(0.1 / self._wp_ds)))
        tf = self.waypoints[(nearest + m_t) % n_wp, :2]
        tb = self.waypoints[(nearest - m_t) % n_wp, :2]
        tang = tf - tb
        tn = float(np.hypot(*tang))
        left = (np.array([-tang[1], tang[0]]) / tn if tn > 1e-6
                else np.array([-math.sin(yaw), math.cos(yaw)]))

        if not self.lane_free and self.blocking_pts:
            # 방향 래치 — 한 번 정하면 이 장애물을 지나칠 때까지 유지.
            if self._shift_side == 0:
                obs_c = np.mean(self.blocking_pts, axis=0)
                lateral = float((obs_c - np.array([px, py])) @ left)
                self._shift_side = -1.0 if lateral > 0 else 1.0
            goal = self._shift_side * self.obs_shift
        else:
            self._shift_side = 0
            goal = 0.0

        # ── 램프 — 계단으로 옮기면 조향이 급격히 튄다. dt 기반으로 부드럽게. ──
        step = self.p('obstacle_shift_rate') * dt
        self._shift_cur += float(np.clip(goal - self._shift_cur, -step, step))

        if abs(self._shift_cur) > 1e-3:
            self._shift_vec = self._shift_cur * left
            return tx + self._shift_vec[0], ty + self._shift_vec[1]
        self._shift_vec = None
        return tx, ty

    def scan_cb(self, msg: LaserScan):
        if self.pos is None or self.waypoints is None:
            return

        xmin = self.p('grid_xmin'); xmax = self.p('grid_xmax')
        ymin = self.p('grid_ymin'); ymax = self.p('grid_ymax')
        res  = self.p('grid_res');  safe = self.p('grid_safe_dist')

        ranges = np.clip(msg.ranges, msg.range_min, msg.range_max)

        nx = int((xmax - xmin) / res) + 1
        ny = int((ymax - ymin) / res) + 1
        gx = np.linspace(xmin, xmax, nx)
        gy = np.linspace(ymin, ymax, ny)
        gy, gx = np.meshgrid(gy, gx)

        # 격자는 base_link 기준, ranges 는 laser 원점 기준 — 오프셋 보정.
        lx = gx - self.p('laser_offset_x')
        ly = gy - self.p('laser_offset_y')
        rho = np.sqrt(lx**2 + ly**2)
        phi = np.arctan2(ly, lx)
        idx = np.clip(
            ((phi - msg.angle_min) / msg.angle_increment).astype(int),
            0, len(ranges) - 1)
        occ = (np.abs(rho - ranges[idx]) < safe).ravel()

        gxf = gx.ravel()[occ]
        gyf = gy.ravel()[occ]

        if len(gxf) == 0:
            self.blocking_pts = []
            self.free_votes.append(True)
            return

        cy, sy = math.cos(self.yaw), math.sin(self.yaw)
        R = np.array([[cy, -sy], [sy, cy]])
        pts_world = (R @ np.vstack((gxf, gyf))).T + self.pos

        self._classify_obstacles(pts_world)

    def _classify_obstacles(self, pts_world: np.ndarray):
        if self.track_mask is None:
            on_track = list(pts_world)
        else:
            h, w = self.map_meta[0], self.map_meta[1]
            res, ox, oy = self.map_meta[2], self.map_meta[3], self.map_meta[4]
            pts_img = _world_to_img(pts_world, h, res, ox, oy)
            ix, iy = pts_img[:, 0], pts_img[:, 1]
            inb = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
            keep = np.zeros(len(pts_world), bool)
            keep[inb] = self.track_mask[iy[inb], ix[inb]]
            on_track = list(pts_world[keep])

        if not on_track:
            self.blocking_pts = []
            self.free_votes.append(True)
            return

        blocking = [pt for pt, k in
                    zip(on_track, self._in_predicted_corridor(np.asarray(on_track)))
                    if k]
        self.blocking_pts = blocking
        self.free_votes.append(not blocking)
        if blocking:
            self._pub_obstacle_markers(blocking)

    def _predicted_track_path(self, horizon: float) -> np.ndarray:
        """앞으로 지나갈 경로를 "레이싱라인 + 현재 횡오차"로 예측 (map 좌표)."""
        wps = self.waypoints
        if wps is None or len(wps) < 3:
            return np.zeros((0, 2))
        n = len(wps)
        px, py = float(self.pos[0]), float(self.pos[1])
        near = int(np.argmin(np.hypot(wps[:, 0] - px, wps[:, 1] - py)))

        m = max(1, int(round(horizon / max(self._wp_ds, 1e-3))))
        m = min(m, n - 1)
        idx = (near + np.arange(m + 1)) % n
        seg = wps[idx, :2]

        j_f = (near + 1) % n
        tang = wps[j_f, :2] - wps[near, :2]
        nt = float(np.hypot(*tang))
        if nt < 1e-6:
            return seg
        tang = tang / nt
        left = np.array([-tang[1], tang[0]])
        cte = float((np.array([px, py]) - wps[near, :2]) @ left)
        return seg + left * cte

    def _in_predicted_corridor(self, pts_world: np.ndarray) -> np.ndarray:
        """차가 실제로 지나갈 통로 안에 있는 점인가 — 순간 원호와 라인+횡오차
        예측의 합집합 (Stanley 와 동일한 이중 판정, 자세한 이유는
        stanley_controller.py:_in_predicted_corridor 주석 참조)."""
        if len(pts_world) == 0:
            return np.zeros(0, bool)

        d = pts_world - self.pos
        c, s = math.cos(-self.yaw), math.sin(-self.yaw)
        fx = c * d[:, 0] - s * d[:, 1]
        fy = s * d[:, 0] + c * d[:, 1]

        scale = 1.0 if self.lane_free else self.p('obs_release_scale')
        hw = self.p('obs_corridor_half_width') * scale
        horizon = self._obs_horizon() * scale
        v, w = abs(self.speed), self.yaw_rate

        R = None
        if abs(w) > 1e-3 and v > 1e-3:
            R = v / w if self.speed >= 0 else -v / w
            if abs(R) > 1e3:
                R = None

        if R is None:
            arc_in = (fx > 0.0) & (fx < horizon) & (np.abs(fy) < hw)
        else:
            dist = np.hypot(fx, fy - R)
            lateral = np.abs(dist - abs(R)) < hw
            sr = math.copysign(1.0, R)
            phi = np.mod(np.arctan2(sr * fx, sr * (R - fy)), 2.0 * math.pi)
            arc_in = lateral & (abs(R) * phi < horizon)

        path = self._predicted_track_path(horizon)
        if len(path) == 0:
            return arc_in
        ahead = fx > -0.05
        path_in = np.zeros(len(pts_world), bool)
        if ahead.any():
            dmin = scipy.spatial.distance.cdist(
                path, pts_world[ahead]).min(axis=0)
            path_in[ahead] = dmin < hw

        return arc_in | path_in

    def _obs_horizon(self) -> float:
        return float(np.clip(
            self.p('obs_horizon_min') + self.p('obs_horizon_sec') * abs(self.speed),
            self.p('obs_horizon_min'), self.p('obs_horizon_max')))

    def _pub_obstacle_markers(self, pts: list):
        arr = MarkerArray()
        for i, pt in enumerate(pts[:30]):
            m = Marker()
            m.header.frame_id = 'map'
            m.ns = 'obstacle'; m.id = i
            m.type = Marker.CUBE; m.action = Marker.ADD
            m.pose.position.x = float(pt[0])
            m.pose.position.y = float(pt[1])
            m.color.r = 1.0; m.color.a = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.1
            m.lifetime.nanosec = int(1e8)
            arr.markers.append(m)
        self.obs_pub.publish(arr)

    def publish_stop(self):
        msg = AckermannDriveStamped()
        msg.drive.speed = 0.0
        self.drive_pub.publish(msg)

    def _update_car_trail(self, x: float, y: float, speed: float):
        """0.1m 이상 이동 시에만 점 추가 — driven_path.csv 로 종료 시 flush."""
        if self._last_trail is not None and \
                math.hypot(x - self._last_trail[0], y - self._last_trail[1]) < 0.1:
            return
        self._last_trail = (x, y)
        t = (self.get_clock().now() - self._run_t0).nanoseconds / 1e9
        self._trail_log.append((t, x, y, float(speed)))
        if len(self._trail_log) > 3000:
            self._trail_log.pop(0)

    def save_run_log(self):
        """주행 종료 시 1회 — 실주행 경로 + 그 시점 게인(race_script.yaml)을
        logs/ 밑에 팀 공유 폴더로 저장 (stanley_controller.save_run_log 와 동일 포맷).

        ★ 폴더명에 컨트롤러(+ 장애물회피 on/off)까지 넣는다 — q2 를 stanley/
          pure_pursuit 둘 다로 돌리게 된 뒤로는 mode 만으론 어느 주행인지
          구분이 안 된다. gains.yaml 을 열어야 알던 정보를 폴더명만 보고
          바로 알 수 있게 한다.
        """
        if not self._trail_log:
            return
        ts = time.strftime('%Y%m%d_%H%M%S')
        ctrl_tag = 'pure_pursuit_avoid' if self.obs_on else 'pure_pursuit'
        run_dir = os.path.join(_REPO_ROOT, 'logs', f'{ts}_{self.mode}_{ctrl_tag}')
        try:
            os.makedirs(run_dir, exist_ok=True)

            with open(os.path.join(run_dir, 'driven_path.csv'), 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['t', 'x', 'y', 'speed'])
                for t, x, y, v in self._trail_log:
                    w.writerow([f'{t:.3f}', f'{x:.4f}', f'{y:.4f}', f'{v:.3f}'])

            # ★ race_script.yaml + 차량 오버레이를 함께 남긴다.
            #   오버레이가 빠지면 스냅샷이 조용히 거짓말을 한다 (2026-08-11).
            write_run_config(run_dir)

            self.get_logger().info(
                f'[run_log] {len(self._trail_log)}점, {run_dir} 에 저장')
        except Exception as e:
            self.get_logger().warn(f'[run_log] 저장 실패: {e}')


def main():
    rclpy.init()
    node = PurePursuit()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # SIGTERM(race.sh kill_stack)은 rclpy 가 컨텍스트를 내리면서
        # ExternalShutdownException 으로 올라온다. KeyboardInterrupt 만 잡으면
        # 이게 안 잡혀 종료 정리가 반쪽이 된다.
        pass
    except Exception:
        # 컨텍스트가 이미 내려간 뒤 executor 내부(wait set 등)에서 나는 예외는
        # 종료 잡음일 뿐이다. 살아 있는 상태의 예외만 그대로 올린다.
        # (RCLError 는 rclpy 의 private 모듈에만 있어 타입으로 못 잡는다)
        if rclpy.ok():
            raise
    finally:
        # ★ 컨텍스트가 이미 닫힌 뒤면 발행이 예외를 던진다. 가드 없이 두면 그
        #   예외가 finally 를 뚫고 나가 **save_run_log 가 실행되지 않는다** —
        #   2026-08-06 에 MAP·pure_pursuit 주행 로그가 하나도 안 남던 진짜 원인.
        #   (stanley 는 이 가드가 있어서 그 주행만 로그가 남아 있었다)
        if rclpy.ok():
            node.publish_stop()
        node.save_run_log()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
