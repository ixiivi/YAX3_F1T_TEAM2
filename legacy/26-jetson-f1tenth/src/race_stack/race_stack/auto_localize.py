#!/usr/bin/env python3
"""
auto_localize.py — 기동 시 라이다 스캔을 맵에 맞춰 초기 위치를 자동으로 잡는다.

왜 필요한가
───────────
AMCL 은 `recovery_alpha_slow/fast: 0` (전역 복구 꺼짐)이라 초기 파티클을
`init_pose` 주변에만 뿌린다. 게다가 정지 상태에서는 `update_min_d`(2.5cm) /
`update_min_a`(1.7°) 조건 때문에 **필터를 갱신하지도, `/amcl_pose` 를 발행하지도
않는다.** 그래서 지금까지는 RViz 에서 `2D Pose Estimate` 를 사람이 찍어줘야
정렬이 됐다 — 대회 출발선에서 노트북 열고 클릭하는 건 현실적이지 않다.

이 노드가 그 클릭을 대신한다:

  1. 맵에서 "가장 가까운 장애물까지의 거리" 장(거리변환)을 한 번 만든다
  2. `init_pose` 주변으로 후보 자세(x, y, yaw)를 격자로 깔고
  3. 각 후보에서 현재 스캔 점들을 맵 좌표로 옮겨, 장애물까지 평균 거리를 잰다
  4. 가장 잘 맞는 자세를 `/initialpose` 로 발행한다

발행 즉시 AMCL 이 파티클을 그 자리에 다시 뿌리고 `/amcl_pose` 를 내보내므로,
컨트롤러도 pose 를 받고 출발 게이트의 "AMCL ✓" 도 뜬다.

★ 이건 전역 위치추정이 아니다. `init_pose` 근처에서만 찾는다 (기본 ±0.5m, ±15°).
  차를 출발선 근처에 놓기만 하면 되고, 센티미터 단위로 맞출 필요가 없어진다.
  아예 다른 곳에 놓으면 실패하며, 그때는 점수가 나쁘다고 경고한다.

  좌우 대칭이 강한 트랙에서는 뒤집힌 자세가 비슷한 점수를 낼 수 있다.
  탐색 범위를 좁게 두는 이유이기도 하다 (기본값이면 방향이 뒤집힐 여지가 없다).

재실행: `/race/relocalize` 에 Bool true 를 쏘면 다시 맞춘다.
"""
import math
import os

import cv2
import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from PIL import Image
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32


def _wrap(a):
    """각도를 -π~π 로."""
    return math.atan2(math.sin(a), math.cos(a))


class AutoLocalize(Node):
    def __init__(self):
        super().__init__('auto_localize')

        self.declare_parameter('map_yaml', '')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/odom_imu')
        self.declare_parameter('init_x', 0.0)
        self.declare_parameter('init_y', 0.0)
        self.declare_parameter('init_yaw', 0.0)
        # 탐색 범위 — 차를 출발선 "근처"에 놓는 정도의 오차만 흡수한다.
        # 넓히면 대칭 구조에서 엉뚱한 자세로 수렴할 위험이 커진다.
        self.declare_parameter('search_xy', 0.5)          # ± m
        self.declare_parameter('search_yaw_deg', 15.0)    # ± deg
        self.declare_parameter('step_xy', 0.05)           # m (맵 해상도와 같게)
        self.declare_parameter('step_yaw_deg', 1.0)       # deg
        self.declare_parameter('max_range', 8.0)          # 이보다 먼 점은 안 씀
        self.declare_parameter('n_beams', 180)            # 스캔 다운샘플 개수
        self.declare_parameter('laser_offset_x', 0.27)    # base_link → laser
        self.declare_parameter('laser_offset_y', 0.0)
        self.declare_parameter('startup_delay', 2.0)      # 스캔 안정화 대기 (s)
        # 평균 오차가 이보다 크면 "제대로 못 맞췄다" 고 본다 (m)
        self.declare_parameter('fit_warn', 0.20)
        # 한 스텝에 이만큼 넘게 튀면 경고 (m). 1.45m/s 에서 AMCL 갱신 간격을
        # 감안해도 30cm 이상은 주행으로 설명되지 않는다.
        self.declare_parameter('jump_warn', 0.30)
        # 복도 벽 방향으로 yaw 를 먼저 맞출 것인가
        self.declare_parameter('use_walls', True)
        self.declare_parameter('wall_range', 3.0)     # 옆벽으로 볼 최대 거리 (m)
        # 스캔은 PCA 직선다움(0~1), 맵은 히스토그램 봉우리 비율 — 척도가 달라 따로 둔다
        self.declare_parameter('wall_scan_min', 0.85)
        self.declare_parameter('wall_map_min',  0.35)

        gp = lambda n: self.get_parameter(n).value
        self.init = (float(gp('init_x')), float(gp('init_y')), float(gp('init_yaw')))
        self.laser_dx = float(gp('laser_offset_x'))
        self.laser_dy = float(gp('laser_offset_y'))
        self.max_range = float(gp('max_range'))
        self.n_beams = int(gp('n_beams'))
        self.fit_warn = float(gp('fit_warn'))
        self.jump_warn = float(gp('jump_warn'))
        self._amcl_prev = None
        self.wall_range = float(gp('wall_range'))
        self.wall_scan_min = float(gp('wall_scan_min'))
        self.wall_map_min = float(gp('wall_map_min'))

        self.scan = None
        self.speed = 0.0          # 주행 중 재시드 방지용
        self.done = False
        self.dist = None          # 거리변환 (m 단위), 맵 픽셀 격자
        self._bnd = None          # (by, bx, 방향, 세기) 벽 표면 픽셀

        self._load_map(str(gp('map_yaml')))

        self.pub = self.create_publisher(PoseWithCovarianceStamped,
                                         '/initialpose', 10)
        # 정합 결과(평균오차 m)를 알려 mission_manager 가 "출발 준비" 판정에 쓴다.
        # 늦게 뜨는 구독자도 받도록 래치.
        self.fit_pub = self.create_publisher(
            Float32, '/race/localized',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(LaserScan, str(gp('scan_topic')),
                                 self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(Bool, '/race/relocalize', self._redo_cb, 10)
        self.create_subscription(Odometry, str(gp('odom_topic')), self._odom_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 self._amcl_cb, 10)
        # ★ 사람이 RViz 에서 "2D Pose Estimate" 를 찍었으면 **양보한다**.
        #   2026-08-09 실차: 자동 정렬이 자꾸 어긋나 사람이 손으로 찍어뒀는데,
        #   이 노드가 startup_delay 뒤에 자기 결과를 덮어써서 **반드시 점프**가
        #   났다. 정지 상태인지만 봤지 "이미 사람이 정해줬는지" 는 안 봤다.
        #   우리가 쏜 것과 구분하려고 발행 직후 0.5s 는 무시한다.
        self._self_pub_t = -1e9
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose',
                                 self._manual_cb, 10)
        self.create_timer(float(gp('startup_delay')), self._try)

        self.get_logger().info(
            f'AutoLocalize 시작 — init ({self.init[0]:.2f}, {self.init[1]:.2f}, '
            f'{math.degrees(self.init[2]):.1f}°) 주변 ±{gp("search_xy")}m '
            f'±{gp("search_yaw_deg")}° 탐색')

    # ── 맵 → 거리변환 ──────────────────────────────────────────────────────
    def _load_map(self, path):
        if not path or not os.path.exists(path):
            self.get_logger().error(f'맵 yaml 을 못 찾음: {path}')
            return
        cfg = yaml.safe_load(open(path))
        self.res = float(cfg['resolution'])
        self.ox = float(cfg['origin'][0])
        self.oy = float(cfg['origin'][1])
        occ_th = float(cfg.get('occupied_thresh', 0.65))
        img_path = os.path.join(os.path.dirname(path), cfg['image'])
        img = np.asarray(Image.open(img_path).convert('L'))
        self.H, self.W = img.shape
        # map_server 규약: 밝을수록 자유공간. (255-v)/255 가 점유 확률
        occupied = ((255 - img.astype(np.float32)) / 255.0) > occ_th
        # 자유공간에서 "가장 가까운 장애물까지" 거리 (픽셀 → m)
        free = (~occupied).astype(np.uint8)
        self.dist = cv2.distanceTransform(free, cv2.DIST_L2, 5) * self.res

        # 벽 "표면" 과 그 국소 방향을 미리 뽑아 둔다 (_map_wall_dir 용).
        # 표면 = 장애물이면서 자유공간에 인접한 픽셀 = 라이다가 실제로 보는 면.
        occ_u8 = occupied.astype(np.uint8)
        bnd = (occ_u8 > 0) & (cv2.dilate(free, np.ones((3, 3), np.uint8)) > 0)
        by, bx = np.nonzero(bnd)
        if len(bx) >= 30:
            blur = cv2.GaussianBlur(occ_u8.astype(np.float32), (5, 5), 0)
            gx = cv2.Sobel(blur, cv2.CV_32F, 1, 0, ksize=3)
            gy = cv2.Sobel(blur, cv2.CV_32F, 0, 1, ksize=3)
            # 기울기는 벽의 법선. +90° 가 벽 방향. 이미지 y 는 아래로 증가하므로 부호 반전
            ang = np.arctan2(-gy[by, bx], gx[by, bx]) + math.pi / 2.0
            self._bnd = (by, bx, ang, np.hypot(gx[by, bx], gy[by, bx]))
        else:
            self._bnd = None
        self.get_logger().info(
            f'맵 로드: {self.W}x{self.H}px @ {self.res}m, '
            f'장애물 {int(occupied.sum())}px (표면 {len(bx)}px)')

    # ── 콜백 ───────────────────────────────────────────────────────────────
    def _scan_cb(self, msg: LaserScan):
        self.scan = msg

    def _odom_cb(self, msg):
        self.speed = abs(msg.twist.twist.linear.x)

    # ── AMCL 감시 ──────────────────────────────────────────────────────────
    def _amcl_cb(self, msg: PoseWithCovarianceStamped):
        """AMCL 이 물리적으로 불가능한 답을 내면 즉시 알린다.

        ★ 왜 필요한가 (2026-08-04 실차)
          주행 중 위치가 80ms 만에 1.98m 순간이동했고, 그 뒤 20개 표본이 전부
          **맵의 벽 안쪽**이었다. 차는 벽 속에 있을 수 없으므로 그건 보정이
          아니라 발산이다. 그런데 그 순간 로그에는 아무것도 안 남아서, 나중에
          주행 로그를 뜯어보고서야 알았다. 실시간으로 보이면 그 자리에서 판단할
          수 있다 — 컨트롤러가 그 값을 그대로 믿고 조향하기 때문이다.

        여기서 고치지는 않는다. 잘못된 pose 를 코드가 임의로 무시하면 "왜 안
        가지?" 라는 더 어려운 증상이 된다. 판단 근거만 준다.
        """
        if self.dist is None:
            return
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        ix = int((x - self.ox) / self.res)
        iy = int(self.H - (y - self.oy) / self.res)
        inside = 0 <= ix < self.W and 0 <= iy < self.H
        clear = float(self.dist[iy, ix]) if inside else -1.0

        if self._amcl_prev is not None:
            d = math.hypot(x - self._amcl_prev[0], y - self._amcl_prev[1])
            if d > self.jump_warn:
                self.get_logger().error(
                    f'⚠ AMCL 위치 점프 {d*100:.0f}cm — '
                    f'({self._amcl_prev[0]:+.2f},{self._amcl_prev[1]:+.2f}) → '
                    f'({x:+.2f},{y:+.2f}). 컨트롤러가 이 값을 그대로 믿습니다')
        self._amcl_prev = (x, y)

        if not inside:
            self.get_logger().error(f'⚠ AMCL 이 맵 밖을 가리킵니다 ({x:+.2f},{y:+.2f})',
                                    throttle_duration_sec=1.0)
        elif clear <= 0.0:
            self.get_logger().error(
                f'⚠ AMCL 이 **벽 안쪽**을 가리킵니다 ({x:+.2f},{y:+.2f}) — '
                '물리적으로 불가능한 추정입니다. 위치추정이 깨진 상태입니다',
                throttle_duration_sec=1.0)

    def _manual_cb(self, msg: PoseWithCovarianceStamped):
        import time as _t
        if _t.time() - self._self_pub_t < 0.5:
            return                      # 우리가 쏜 것
        if self.done:
            return
        self.done = True
        self.get_logger().warn(
            f'수동 /initialpose 수신 ({msg.pose.pose.position.x:+.2f}, '
            f'{msg.pose.pose.position.y:+.2f}) — 자동 정렬을 **건너뜁니다**. '
            f'다시 자동으로 맞추려면 /race/relocalize 에 true 를 쏘세요.')

    def _redo_cb(self, msg: Bool):
        if msg.data:
            self.done = False
            self.get_logger().info('재정렬 요청 수신')
            self._try()

    def _try(self):
        if self.done or self.dist is None:
            return
        if self.scan is None:
            self.get_logger().warn('스캔 대기 중…')
            return
        # ★ 움직이는 중에는 절대 재시드하지 않는다 (2026-08-04 실차).
        #   차가 이미 출발한 뒤 /initialpose 를 쏘면 AMCL 이 달리는 도중에
        #   파티클을 통째로 옮겨 위치가 튄다. 실제로 그때 AMCL 이
        #   "Lookup would require extrapolation into the future" 를 뱉었다.
        #   정렬은 정지 상태에서만 의미가 있다 — 스캔 한 장으로 맞추는 것이므로.
        if self.speed > 0.05:
            self.get_logger().warn(
                f'주행 중({self.speed:.2f} m/s)이라 정렬을 미룹니다 — '
                '정지 상태에서만 재시드합니다',
                throttle_duration_sec=3.0)
            return
        self.done = True
        self._align(self.scan)

    # ── 벽 방향 추정 ───────────────────────────────────────────────────────
    #
    # ★ 왜 격자 탐색만으로는 부족한가 (2026-08-04)
    #   출발 지점은 양옆이 벽인 **직선 복도**다. 복도에서 비용 함수는 가로 방향
    #   으로는 뾰족하지만 복도를 따라서는 평평하고(aperture problem), yaw 도
    #   1° 격자로는 잘 안 갈린다. 그래서 실측에서 출발 방향이 -6°~-25° 로
    #   흩어졌다. 그런데 복도에서는 **벽의 방향 자체가 답을 알려준다** —
    #   스캔이 만드는 직선과 맵의 직선을 맞추면 yaw 가 바로 정해진다.
    #   격자 탐색으로 짐작할 이유가 없다.
    #
    # 방향은 180° 주기다(벽은 앞뒤 구분이 없다). 그래서 각도를 2배로 만들어
    # 원형 평균을 내고 다시 반으로 나눈다 — 0°와 179° 가 이웃임을 살리는 표준 기법.

    @staticmethod
    def _dominant_angle(ang, weight=None, bins=180, win_deg=10.0):
        """방향들(0~π)의 지배적 각도와 그 집중도.

        전체 평균(원형 평균)을 쓰면 코너·기둥 같은 다른 방향이 섞여 값이 희석된다
        (실측: 복도 한가운데인데 집중도 0.12). 히스토그램 최빈값을 먼저 찾고,
        그 봉우리 ±win_deg 안에 든 가중치 비율을 집중도로 본다 — 다봉 분포에서도
        "복도 방향이 지배적인가" 를 제대로 판정한다.
        """
        if len(ang) < 10:
            return None, 0.0
        a = ang % math.pi
        w = np.ones_like(a) if weight is None else np.asarray(weight, float)
        h, edges = np.histogram(a, bins=bins, range=(0.0, math.pi), weights=w)
        # 180° 주기이므로 원형으로 3칸 스무딩
        h = h + np.roll(h, 1) + np.roll(h, -1)
        k = int(np.argmax(h))
        peak = (edges[k] + edges[k + 1]) / 2.0
        win = math.radians(win_deg)
        d = np.abs(np.angle(np.exp(2j * (a - peak)))) / 2.0     # 180° 주기 거리
        m = d < win
        conc = float(w[m].sum() / (w.sum() + 1e-9))
        if m.sum() < 5:
            return None, 0.0
        # 봉우리 안쪽만으로 원형 평균 → 격자(1°)보다 고운 해상도
        a2 = 2.0 * a[m]
        ref = math.atan2(float(np.sum(w[m] * np.sin(a2))),
                         float(np.sum(w[m] * np.cos(a2)))) / 2.0
        return ref % math.pi, conc

    @staticmethod
    def _line_pca(pts):
        """점 뭉치의 주방향과 직선다움. (각도 0~π, 0~1) — 아니면 (None, 0)."""
        if len(pts) < 15:
            return None, 0.0
        q = pts - pts.mean(axis=0)
        # 공분산의 고유벡터 = 주축. λ2/λ1 가 작을수록 곧은 직선이다.
        cov = (q.T @ q) / len(q)
        ev, evec = np.linalg.eigh(cov)          # 오름차순
        if ev[1] <= 1e-9:
            return None, 0.0
        v = evec[:, 1]
        return math.atan2(v[1], v[0]) % math.pi, float(1.0 - ev[0] / ev[1])

    def _scan_wall_dir(self, px, py):
        """스캔이 보는 좌우 복도 벽의 방향 (차량 좌표계, 0~π).

        ★ 인접점 차분을 쓰면 안 된다 — 라이다 빔 간격이 1m 에서 5.8mm 라
          거리 노이즈 3cm 가 방향을 완전히 지배한다 (실측 집중도 0.14).
          좌·우 벽을 각각 **직선 피팅(PCA)** 하면 노이즈가 평균으로 지워진다.
        ★ 360° 전체도 쓰면 안 된다. 코너·안쪽 섬까지 섞여 방향성이 희석된다.
          옆쪽(전방축 기준 50~130°)의 가까운 점만 본다 — 차가 복도와 ±25° 안에
          놓여 있다면 그 범위가 곧 좌우 벽이다.
        """
        r = np.hypot(px, py)
        a = np.degrees(np.arctan2(py, px))              # 0=전방, ±180=후방
        near = r < self.wall_range
        left = near & (a > 50.0) & (a < 130.0)
        right = near & (a < -50.0) & (a > -130.0)

        got = []
        for m in (left, right):
            if m.sum() < 15:
                continue
            ang, q = self._line_pca(np.stack([px[m], py[m]], axis=1))
            if ang is not None and q > 0.90:            # 충분히 곧은 면만
                got.append((ang, q, int(m.sum())))
        if not got:
            return None, 0.0
        # 두 벽은 나란하므로 180° 주기 원형 평균으로 합친다
        w = np.array([g[2] for g in got], float)
        a2 = 2.0 * np.array([g[0] for g in got])
        ref = math.atan2(float(np.sum(w * np.sin(a2))),
                         float(np.sum(w * np.cos(a2)))) / 2.0
        # 품질: 각 면의 직선다움 평균 × (양쪽 다 잡혔으면 가산)
        qual = float(np.mean([g[1] for g in got])) * (1.0 if len(got) == 2 else 0.8)
        return ref % math.pi, qual

    def _map_wall_dir(self, x0, y0, radius=1.5):
        """맵에서 init 주변 복도의 방향 (map 좌표계, 0~π).

        ★ 장애물 픽셀 전체로 PCA 하면 안 된다 — 맵의 벽은 **속이 채워진 두꺼운
          덩어리**다(이 맵은 아래 1m·위 3.2m 두께). 뭉치를 보면 방향이 안 나온다.
          라이다는 **표면만** 보므로 맵에서도 표면(자유공간에 닿은 장애물 픽셀)만
          쓰고, 각 표면점의 국소 방향을 Sobel 법선에서 얻어 히스토그램으로 모은다.
          그러면 나란한 두 벽이 같은 봉우리에 쌓인다.
        """
        if self._bnd is None:
            return None, 0.0
        by, bx, bang, bmag = self._bnd
        wx = self.ox + bx * self.res
        wy = self.oy + (self.H - by) * self.res
        m = np.hypot(wx - x0, wy - y0) < radius
        if m.sum() < 30:
            return None, 0.0
        return self._dominant_angle(bang[m], bmag[m])

    # ── 정렬 ───────────────────────────────────────────────────────────────
    def _align(self, scan: LaserScan):
        gp = lambda n: self.get_parameter(n).value

        # 스캔 → base_link 좌표의 점들 (유효한 것만, 균등 다운샘플)
        r = np.asarray(scan.ranges, dtype=float)
        a = scan.angle_min + np.arange(len(r)) * scan.angle_increment
        ok = np.isfinite(r) & (r > scan.range_min) & (r < min(scan.range_max,
                                                              self.max_range))
        if ok.sum() < 20:
            self.get_logger().error(f'쓸 만한 빔이 {int(ok.sum())}개뿐 — 정렬 중단')
            return
        idx = np.where(ok)[0]
        if len(idx) > self.n_beams:
            idx = idx[np.linspace(0, len(idx) - 1, self.n_beams).astype(int)]
        px = r[idx] * np.cos(a[idx]) + self.laser_dx
        py = r[idx] * np.sin(a[idx]) + self.laser_dy      # (N, ) base_link 기준

        # 후보 격자
        sxy = float(gp('search_xy')); step = float(gp('step_xy'))
        syaw = math.radians(float(gp('search_yaw_deg')))
        dyaw = math.radians(float(gp('step_yaw_deg')))
        dx = np.arange(-sxy, sxy + 1e-9, step)
        dy = np.arange(-sxy, sxy + 1e-9, step)
        dth = np.arange(-syaw, syaw + 1e-9, dyaw)
        x0, y0, th0 = self.init

        # ── 벽 방향으로 yaw 를 먼저 확정한다 ────────────────────────────
        # 되면 격자는 그 주변만 미세하게 훑는다. 안 되면(복도가 아니거나 방향성이
        # 약하면) 원래 범위로 되돌아간다 — 조용히 나빠지지 않게 로그를 남긴다.
        if bool(gp('use_walls')):
            sa, sR = self._scan_wall_dir(px, py)
            ma, mR = self._map_wall_dir(x0, y0)
            if (sa is not None and ma is not None
                    and sR > self.wall_scan_min and mR > self.wall_map_min):
                # 방향은 180° 주기라 후보가 둘이다. init_yaw 에 가까운 쪽을 고른다.
                cands = [(ma - sa) % (2 * math.pi), (ma - sa + math.pi) % (2 * math.pi)]
                yaw_w = min(cands, key=lambda t: abs(_wrap(t - th0)))
                off = math.degrees(_wrap(yaw_w - th0))
                th0 = yaw_w
                dth = np.arange(-math.radians(6.0), math.radians(6.0) + 1e-9, dyaw)
                self.get_logger().info(
                    f'벽 방향 정렬: 스캔 {math.degrees(sa):.1f}°(집중도 {sR:.2f}) / '
                    f'맵 {math.degrees(ma):.1f}°(집중도 {mR:.2f}) → '
                    f'yaw {math.degrees(yaw_w):+.1f}° (init 대비 {off:+.1f}°), '
                    f'이후 ±6° 미세탐색')
            else:
                self.get_logger().warn(
                    f'벽 방향을 못 잡음 (스캔 집중도 {sR:.2f}, 맵 {mR:.2f}) — '
                    f'±{gp("search_yaw_deg")}° 격자 탐색으로 진행합니다')

        best = (np.inf, x0, y0, th0)
        # yaw 별로 한 번 회전시켜 놓고 xy 는 브로드캐스트로 훑는다
        for t in dth:
            th = th0 + t
            c, s = math.cos(th), math.sin(th)
            wx = c * px - s * py + x0        # (N,)
            wy = s * px + c * py + y0
            # (nx, ny, N)
            gx = wx[None, None, :] + dx[:, None, None]
            gy = wy[None, None, :] + dy[None, :, None]
            ix = ((gx - self.ox) / self.res).astype(np.int32)
            iy = (self.H - (gy - self.oy) / self.res).astype(np.int32)
            valid = (ix >= 0) & (ix < self.W) & (iy >= 0) & (iy < self.H)
            ixc = np.clip(ix, 0, self.W - 1)
            iyc = np.clip(iy, 0, self.H - 1)
            d = self.dist[iyc, ixc]
            # 맵 밖으로 나간 점은 크게 벌점 (맵 밖은 정보가 없는 게 아니라 틀린 것)
            d = np.where(valid, d, 3.0)
            score = d.mean(axis=2)           # (nx, ny)
            k = int(np.argmin(score))
            i, j = np.unravel_index(k, score.shape)
            if score[i, j] < best[0]:
                best = (float(score[i, j]), x0 + dx[i], y0 + dy[j], th)

        fit, bx, by, bth = best
        d0 = math.hypot(bx - x0, by - y0)
        self.get_logger().info(
            f'정렬 결과: ({bx:+.3f}, {by:+.3f}, {math.degrees(bth):+.1f}°)  '
            f'평균오차 {fit*100:.1f}cm  |  init 대비 {d0*100:.1f}cm '
            f'{math.degrees(bth - th0):+.1f}°')

        if fit > self.fit_warn:
            self.get_logger().warn(
                f'⚠ 평균오차 {fit*100:.1f}cm — 맵과 잘 안 맞습니다. '
                '차가 출발선 근처에 있는지, 맵이 맞는지 확인하세요. '
                '(그래도 init_pose 보다는 나으므로 발행은 합니다)')

        # ── 이 해가 얼마나 뾰족한가 (2026-08-09) ────────────────────────
        # 직선 복도에서는 **복도를 따라가는 방향으로 비용이 평평하다**
        # (aperture problem). 그러면 격자가 그 방향으로는 사실상 아무 데나
        # 고르고, 실행할 때마다 다른 답이 나온다 — "될 때도 있고 안 될 때도
        # 있다" 의 정체다. 평균오차만 보면 그 상태가 안 보이므로 따로 알린다.
        cb, sb = math.cos(bth), math.sin(bth)
        wx = cb * px - sb * py + bx
        wy = sb * px + cb * py + by
        def cost(ddx, ddy):
            ix = ((wx + ddx - self.ox) / self.res).astype(np.int32)
            iy = (self.H - (wy + ddy - self.oy) / self.res).astype(np.int32)
            v = (ix >= 0) & (ix < self.W) & (iy >= 0) & (iy < self.H)
            d = self.dist[np.clip(iy, 0, self.H-1), np.clip(ix, 0, self.W-1)]
            return float(np.where(v, d, 3.0).mean())
        probe = 0.20                       # ±20cm 흔들어 본다
        along = (cost(cb*probe, sb*probe) + cost(-cb*probe, -sb*probe)) / 2 - fit
        cross = (cost(-sb*probe, cb*probe) + cost(sb*probe, -cb*probe)) / 2 - fit
        self.get_logger().info(
            f'해의 뾰족함 (±{probe*100:.0f}cm 흔들 때 오차 증가): '
            f'진행방향 {along*100:+.2f}cm / 옆방향 {cross*100:+.2f}cm')
        if along < 0.02:
            self.get_logger().warn(
                f'⚠ **진행방향으로 비용이 평평하다** (증가 {along*100:.2f}cm). '
                f'직선 복도라 그 방향 위치가 관측되지 않는다 — 이 정렬은 앞뒤로 '
                f'수십 cm 틀릴 수 있고 실행할 때마다 답이 달라진다. '
                f'출발 위치를 코너가 보이는 쪽으로 옮기거나, RViz 2D Pose '
                f'Estimate 로 직접 찍을 것(찍으면 이 노드는 양보한다).')

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(bx)
        msg.pose.pose.position.y = float(by)
        msg.pose.pose.orientation.z = math.sin(bth * 0.5)
        msg.pose.pose.orientation.w = math.cos(bth * 0.5)
        # 탐색 격자 크기 정도의 불확실성을 남긴다 — 0 으로 두면 AMCL 이
        # 파티클을 한 점에 몰아 이후 보정이 둔해진다
        cov = [0.0] * 36
        cov[0] = cov[7] = (2.0 * float(gp('step_xy'))) ** 2
        cov[35] = (2.0 * math.radians(float(gp('step_yaw_deg')))) ** 2
        msg.pose.covariance = cov
        import time as _t
        self._self_pub_t = _t.time()
        self.pub.publish(msg)
        self.fit_pub.publish(Float32(data=float(fit)))
        self.get_logger().info('/initialpose 발행 — AMCL 이 이 자리로 재시드합니다')


def main():
    rclpy.init()
    node = AutoLocalize()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
