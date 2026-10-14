#!/usr/bin/env python3
"""frenet_planner — Frenet 회피 오프셋을 발행하는 독립 노드.

무엇을 하나
──────────
레이싱라인을 기준선으로 후보들(점별 법선으로 d 만큼 민 것)을 만들고, 장애물
여유·트랙 이탈·복귀력·변화량·통과속도(v_max)로 비용을 매겨 **최적 횡오프셋 d** 를
고른다. 로직은 `obstacle_avoid.ObstacleAvoidMixin`.

    발행:  /frenet/offset   Float64   ← 계약. 컨트롤러가 소비한다
           /frenet/status   Float64   0=TRACK 1=AVOID_STATIC 2=BLOCKED
                                      3=FOLLOW 4=RECOVER 5=OVERTAKE
           /frenet/speed_scale Float64
           /frenet/diag     Float64MultiArray  로그용 (driven_path.csv 와 같은 열)
           /frenet_candidates          RViz 마커
           /obstacle                   RViz 장애물 점

왜 경로(Path)가 아니라 스칼라 d 인가
────────────────────────────────────
웨이포인트에는 **속도 프로파일이 실려 있다**. Path 로 보내면 속도를 따로 실어야
하고 컨트롤러가 원본과 다시 짝지어야 한다. d 만 넘기면 각 컨트롤러가
`wps + d·normal` 세 줄로 적용하고 속도가 그대로 유지된다.

★ 컨트롤러별 호라이즌 (이식 시 유일한 함정)
────────────────────────────────────────────
d 에 수렴하는 속도가 컨트롤러마다 다르다.

    Stanley  δ = ψ_e + atan(k·e/v)      전방주시 지연 없음 → 즉시 반응
    MAP/L1   a_lat = 2v²sin(η)/L1       2차계 ζ=0.707, ωn=√2·v/L1
             → 오버슈트는 4.3% 로 작지만 **상승거리 1.8·L1/√2 ≈ 1.15m** (L1=0.9m)

즉 MAP 은 오프셋을 다 옮기는 데 1.15m 를 달려야 한다. Stanley 기준으로 맞춰둔
obs_horizon_min(0.8m)을 그대로 쓰면 **다 피하기 전에 장애물에 닿는다.**
경험칙: **회피 호라이즌 ≥ 2 × L1**. race_script.yaml 의
`frenet.per_controller.<이름>` 에서 컨트롤러별로 덮어쓴다.

안전
────
컨트롤러 쪽에서 **타임아웃 폴백**을 반드시 걸 것 — 이 노드가 죽거나 늦으면
d=0(레이싱라인)으로 돌아가야 한다. 이 저장소의 /slip/steer_limit 과 같은 패턴이다.
"""
import math
import os
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64, Float64MultiArray
from visualization_msgs.msg import Marker, MarkerArray

from race_stack.obstacle_avoid import (
    ObstacleAvoidMixin, _load_map, pack_frenet_diag, PLANNER_MODE_NAME,
    BLOCKED, FOLLOW)

LATCHED_IN = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class FrenetPlanner(ObstacleAvoidMixin, Node):
    def __init__(self):
        super().__init__('frenet_planner')
        d = self.declare_parameter

        d('waypoints_csv', '')
        d('map_yaml',      '')
        d('bound_dir',     '')
        d('scan_topic',    '/scan')
        d('odom_topic',    '/odom')
        d('rate_hz',       50.0)
        d('mode',          'q2')

        # 라이다 장착 위치 (base_link 기준)
        d('laser_dx', 0.27)
        d('laser_dy', 0.0)

        # ── 호라이즌 ─ ★ 컨트롤러에 맞춰 넣을 것 (위 docstring 참고) ──────
        d('obs_horizon_sec',   0.6)
        d('obs_horizon_min',   0.8)
        d('obs_horizon_max',   3.0)
        d('obs_corridor_half', 0.45)
        d('obs_release_scale', 1.3)

        # ── Frenet ────────────────────────────────────────────────────
        d('frenet_dmax',      0.6)
        d('frenet_dstep',     0.10)
        d('frenet_clearance', 0.35)
        d('frenet_clear_cap', 0.8)
        d('frenet_w_offset',  1.0)
        d('frenet_w_change',  0.5)
        d('frenet_w_clear',   2.0)
        # 후보별 v_max/profile (0~1). 0 이면 선택 cost 는 예전과 같고
        # speed_scale 만 v_max 로 바뀐다.
        d('frenet_w_speed',   0.5)
        d('frenet_w_ttc',     0.0)   # 0 = v1 (곡률만). >0 이면 TTC 를 cost 에만
        d('frenet_ttc_ref',   2.0)   # 이보다 길면 ttc_score=0
        d('frenet_ttc_min',   0.5)   # 이하면 ttc_score=1 (AEB 쪽 영역)
        d('frenet_bounds',    True)
        d('frenet_votes',     3)
        d('track_margin',     0.15)
        # ★ 통과 여유 하한 [m] — 차 반폭 + 안전여유. 이보다 좁으면 못 지나간다.
        #   옛 판정(마스크 all())이 레이싱라인 자신을 27% 프레임에서 탈락시켰다.
        #   0.22 면 두 트랙 모두 라인 탈락 0% (_frenet_select 주석 참고).
        d('frenet_min_room',  0.22)
        # 다른 쪽 통로가 이만큼 더 넓어야 갈아탄다 [m]. 좌우 진동 방지.
        d('frenet_gap_hyst',  0.10)
        # 후보 경로가 조향 한계를 넘는 곡률이면 탈락시킨다 (안쪽 헤어핀).
        d('wheelbase',        0.33)
        d('max_steer',        0.36)
        # ── 1대1(race) — 동적 장애물 ──────────────────────────────────
        d('opp_track',      True)   # 상대차 속도 추정 + 도달시각 예측
        d('opp_eps',        0.30)   # 클러스터 거리 [m]
        d('opp_gate',       0.80)   # 프레임 간 대응 게이트 [m]
        d('opp_alpha',      0.6)    # 속도 저역통과
        d('opp_min_obs',    3)      # 이만큼 봐야 속도를 믿는다
        d('opp_min_speed',  0.5)    # 이보다 느리면 정적 취급 [m/s]
        d('opp_pred_max',   1.2)    # 예측 시간 상한 [s]
        # ── 속도 스케일 (컨트롤러가 /frenet/speed_scale 로 받는다) ────
        #   고른 후보의 v_max / 라인 프로파일. 마찰 프로파일 **위로는
        #   절대 안 올린다.** 막히거나 곡률이 크면 줄이고, 아니면 1.0.
        # ── 추월 가능 구간 (1대1) ──────────────────────────────────────
        #   회피는 시작보다 **끝내는 것**이 어렵다. 넓은 구간 끝에서 옆으로
        #   나가면 다 지나가기 전에 좁아져 상대와 벽 사이에 낀다.
        #   상세는 obstacle_avoid._overtake_zone 주석.
        d('ot_zone',       False)  # 켜면 넓은 구간에서만 옆으로 나간다
        d('ot_width',      1.10)   # 나란히 가려면 필요한 복도 폭 [m]
        d('ot_exit_pad',   2.50)   # 좁아지기 전에 미리 끝낼 거리 [m]
        d('ot_min_len',    2.00)   # 이보다 짧은 구간은 버린다 [m]
        # 추월 금지 구간에서 상대 뒤를 따라가는 법칙
        d('follow_gap',    0.80)   # 유지할 s 간격 [m]
        d('follow_k',      1.2)    # 간격오차 → 속도 이득 [1/s]
        d('follow_min',    0.25)   # 속도 배율 하한
        # ── 회피 오프셋 유지 — 다 지나갈 때까지 ────────────────────────
        #   나란히 달리는 동안 상대가 **시야에서 사라진다** (라이다가 앞 0.27m,
        #   화각 ±70° 라 종방향 0.4m 안이면 안 보인다). 그때 오프셋이 풀리면
        #   아직 옆에 있는 상대를 들이받는 쪽으로 조향한다.
        #   상세·수치는 obstacle_avoid._update_hold 주석.
        d('hold',          False)
        d('hold_car_len',  0.50)   # 내 차 길이 [m]
        d('hold_obj_len',  0.50)   # 피할 대상 길이 [m]
        d('hold_margin',   0.30)
        d('hold_max_sec',  3.0)    # 상한 — 잘못 물려 영영 안 풀리는 것 방지
        d('static_lock',   False)  # 08/15 기준. 켜면 정지 장애물 d 고정
        d('candidate_fb',  False)  # 08/15 기준. 켜면 짧은 FB 통과시간
        d('fb_horizon',    5.0)    # FB 전방 거리 [m]
        d('fb_iters',      2)
        d('speed_min_scale', 0.45)
        d('speed_room_ref',  0.45)  # speed_scale_mode=room 일 때만
        d('speed_scale_mode', 'vmax')  # vmax | room (ablation baseline)
        d('grip_util',       0.6)
        d('mu',              0.348)
        d('pac_d',           1.15)
        # ★ speed_scale 의 분모(= 컨트롤러가 낼 속도)를 여기서 다시 푼다.
        #   CSV 열을 쓰면 안 된다 — gen_raceline 이 --mu 0.9 로 계산한 값이라
        #   분모만 1.5배 커져 배율이 항상 0.5 로 깎인다 (_line_speed 주석).
        d('lf',              0.1757)
        d('lr',              0.1441)
        d('min_speed',       0.3)
        d('a_lon_acc',       0.0)
        d('a_lon_brk',       0.0)
        d('hard_cap',        0.0)
        # 컨트롤러의 기준선 램프 속도 [m/s]. 같아야 플래너가 보는 "현재 위치"가
        # 실제와 맞는다 (race_script.yaml frenet.shift_rate 와 동일 값).
        d('shift_rate',       2.0)
        # 컨트롤러의 L1 (회피 완료 거리 계산용). 값이 갈리면 상한이 틀어진다.
        d('l1_min',           1.32)
        d('l1_k',             0.75)

        # ── 레이캐스팅 (믹스인이 self.get_parameter 로 직접 읽는다) ────────
        #   기본값은 stanley_controller 와 동일하게 맞췄다. 갈라지면 같은 코드가
        #   컨트롤러에 따라 다르게 동작한다.
        d('raycast_obs',      True)
        d('raycast_fov_deg',  140.0)
        d('raycast_beam_deg', 0.25)
        d('raycast_deficit',  0.30)
        d('raycast_pose_unc', 0.25)
        # 검사 (B): 빔 끝점이 벽에서 이만큼 떨어지면 장애물. 0 = 끔.
        #   좁은 통로 한가운데 장애물을 잡는다 (_raycast_obstacle_pts 참고).
        d('raycast_free_gap', 0.30)
        # ── 자세 어긋남 자기보정 ──────────────────────────────────────
        #   AMCL 이 맵과 어긋난 만큼 두 검사의 문턱을 같이 올린다.
        #   어긋남은 빔 끝점의 벽거리 중앙값으로 매 스캔 잰다 (공짜).
        #   0 = 끔. 상세는 obstacle_avoid._raycast_obstacle_pts 주석.
        d('skew_gain',     1.0)
        d('skew_max',      0.35)
        # 속도 판정에 쓸 전방 거리 [m]. 0 = 경로 전체(옛 동작 = 이중 감속)
        d('spd_span',      1.20)
        # ── 회피 완료 가능성 속도 상한 ─────────────────────────────────
        #   콘은 2m 안에서야 검출되는데(스캔 평면이 낮다) 오프셋을 다 옮기려면
        #   3.2㎧ 에서 4m 가 필요하다. 그 차이를 속도로 메운다.
        #   상세는 obstacle_avoid._shift_feasible_speed 주석.
        d('shift_cap',     True)
        d('shift_safety',  1.25)
        d('shift_min_v',   0.8)
        # 움직이는 상대는 gap 을 접근속도로 환산한다 (2026-08-16)
        d('shift_close_min', 0.30)
        d('shift_close_max', 6.0)
        d('shift_close_r',   0.60)
        # 남은 이동량이 이보다 작으면 상한을 푼다 (2026-08-17)
        d('shift_done',      0.05)
        # 장애물이 **보이기 전에** 거는 접근속도 상한.
        #   검출 거리는 하드웨어가 정한다(실측 2.7m). 남는 시간은 속도에
        #   반비례하므로, 보인 뒤에 깎으면 제동거리가 모자란다.
        d('shift_preempt', True)
        d('shift_det_m',   2.70)   # 장애물이 보이기 시작하는 거리 [m]
        d('shift_brake',   0.0)    # 제동 능력 [m/s²]. 0 = μ_eff·g 자동
        # ── AMCL 용 스캔 마스킹 (앞차 뒤에 붙었을 때) ──────────────────
        #   앞차가 벽을 가리면 AMCL 이 그 빔을 벽으로 읽어 자세가 끌려간다.
        #   실측(08/15 race): 장애물 0.6m 안이면 전방 빔의 18~43% 가 맵으로
        #   설명이 안 됐다. 상세·안전장치는 obstacle_avoid._amcl_mask 참고.
        d('amcl_mask',     False)   # 켜면 /scan_amcl 발행
        d('amcl_mask_pad', 2)       # 지우는 빔 좌우 여유 [빔]
        d('amcl_mask_max', 0.30)    # 이 비율 넘게 지워야 하면 아예 안 지운다
        d('amcl_scan_topic', '/scan_amcl')
        d('use_bounds',       False)
        d('debug',            True)

        p = lambda n: self.get_parameter(n).value

        # ── 믹스인이 요구하는 속성들 ──────────────────────────────────
        self.mode      = str(p('mode'))
        self.laser_dx  = float(p('laser_dx'))
        self.laser_dy  = float(p('laser_dy'))
        self.obs_h_sec = float(p('obs_horizon_sec'))
        self.obs_h_min = float(p('obs_horizon_min'))
        self.obs_h_max = float(p('obs_horizon_max'))
        self.obs_corr_hw       = float(p('obs_corridor_half'))
        self.obs_release_scale = float(p('obs_release_scale'))
        self.track_margin = float(p('track_margin'))
        self.use_bounds   = bool(p('use_bounds'))
        self.skew_gain    = float(p('skew_gain'))
        self.skew_max     = float(p('skew_max'))
        self.spd_span     = float(p('spd_span'))
        self.shift_cap_on   = bool( p('shift_cap'))
        self.shift_safety   = float(p('shift_safety'))
        self.shift_min_v    = float(p('shift_min_v'))
        self.shift_close_min = float(p('shift_close_min'))
        self.shift_close_max = float(p('shift_close_max'))
        self.shift_close_r   = float(p('shift_close_r'))
        self.shift_done      = float(p('shift_done'))
        self.shift_preempt  = bool( p('shift_preempt'))
        self.shift_det_m    = float(p('shift_det_m'))
        self.shift_brake    = float(p('shift_brake'))
        self.shift_l1_min   = float(p('l1_min'))
        self.shift_l1_k     = float(p('l1_k'))
        self.shift_rate_ref = float(p('shift_rate'))

        self.fr_dmax   = float(p('frenet_dmax'))
        step           = max(float(p('frenet_dstep')), 1e-3)
        n_side         = int(round(self.fr_dmax / step))
        self.fr_grid   = np.arange(-n_side, n_side + 1) * step
        self.fr_clear  = float(p('frenet_clearance'))
        self.fr_clr_cap = float(p('frenet_clear_cap'))
        self.fr_w_off  = float(p('frenet_w_offset'))
        self.fr_w_chg  = float(p('frenet_w_change'))
        self.fr_w_clr  = float(p('frenet_w_clear'))
        self.fr_w_spd  = float(p('frenet_w_speed'))
        self.fr_w_ttc  = float(p('frenet_w_ttc'))
        self.fr_ttc_ref = float(p('frenet_ttc_ref'))
        self.fr_ttc_min = float(p('frenet_ttc_min'))
        self.grip_util = float(p('grip_util'))
        self.mu        = float(p('mu'))
        self.pac_d     = float(p('pac_d'))
        self.lf        = float(p('lf'))
        self.lr        = float(p('lr'))
        self.min_speed = float(p('min_speed'))
        self.a_lon_acc = float(p('a_lon_acc'))
        self.a_lon_brk = float(p('a_lon_brk'))
        self.hard_cap  = float(p('hard_cap'))
        self.amcl_mask_on  = bool( p('amcl_mask'))
        self.amcl_mask_pad = int(  p('amcl_mask_pad'))
        self.amcl_mask_max = float(p('amcl_mask_max'))
        self._amcl_masked = 0
        self.fr_bounds = bool(p('frenet_bounds'))
        self.fr_min_room = float(p('frenet_min_room'))
        self.fr_gap_hyst = float(p('frenet_gap_hyst'))
        self.wheelbase   = float(p('wheelbase'))
        self.max_steer   = float(p('max_steer'))
        self.opp_track     = bool( p('opp_track'))
        self.opp_eps       = float(p('opp_eps'))
        self.opp_gate      = float(p('opp_gate'))
        self.opp_alpha     = float(p('opp_alpha'))
        self.opp_min_obs   = int(  p('opp_min_obs'))
        self.opp_min_speed = float(p('opp_min_speed'))
        self.opp_pred_max  = float(p('opp_pred_max'))
        self._opp_tracks = []
        self._opp_moving = 0
        self.ot_zone_on  = bool( p('ot_zone'))
        self.ot_width    = float(p('ot_width'))
        self.ot_exit_pad = float(p('ot_exit_pad'))
        self.ot_min_len  = float(p('ot_min_len'))
        self.follow_gap  = float(p('follow_gap'))
        self.follow_k    = float(p('follow_k'))
        self.follow_min  = float(p('follow_min'))
        self.hold_on      = bool( p('hold'))
        self.hold_car_len = float(p('hold_car_len'))
        self.hold_obj_len = float(p('hold_obj_len'))
        self.hold_margin  = float(p('hold_margin'))
        self.hold_max_sec = float(p('hold_max_sec'))
        self.static_lock  = bool( p('static_lock'))
        self.candidate_fb = bool( p('candidate_fb'))
        self.fb_horizon   = float(p('fb_horizon'))
        self.fb_iters     = int(  p('fb_iters'))
        self._hold_until = 0.0
        self._hold_d = 0.0
        self._lock_d = None
        self._lock_lost = 0
        self._now_sec = 0.0
        self._ot_allowed = True
        self._follow_gap = float('inf')
        self.sp_min_scale = float(p('speed_min_scale'))
        self.sp_room_ref  = max(float(p('speed_room_ref')), 1e-3)
        self.sp_mode      = str(p('speed_scale_mode')).lower()
        self._fr_vmax = 0.0
        self._fr_v_ref = 1e-3
        self._fr_kappa = 0.0
        self._fr_speed_score = 1.0
        self._fr_cost = 0.0
        self._fr_ttc = 1.0e9
        self._fr_ttc_score = 0.0
        self._fr_costs = deque(maxlen=max(int(p('frenet_votes')), 1))
        self._fr_best_d = 0.0
        self._fr_gap_room = 0.0
        self._fr_status = 0.0
        self._shift_rate = max(float(p('shift_rate')), 1e-3)
        self._dt = 1.0 / max(float(p('rate_hz')), 1.0)
        self._shift_cur = 0.0        # 컨트롤러와 같은 램프로 _tick 에서 따라간다

        self.free_votes = deque(maxlen=max(int(p('frenet_votes')), 1))
        self.lane_free  = True
        self.on_track_pts = np.empty((0, 2))
        self.blocking_pts = []

        self.pos = None
        self.yaw = 0.0
        self.yaw_rate = 0.0
        self.speed = 0.0
        self.latest_scan = None

        self.map_arr = self.map_meta = None
        self.track_mask = None
        self.ray_occ = None
        self.inner_bound = self.outer_bound = None
        self._wp_ds = 0.1
        self.wps = None

        # ── 자원 로드 ─────────────────────────────────────────────────
        self._load_waypoints(str(p('waypoints_csv')))
        my = str(p('map_yaml'))
        if my and os.path.exists(my):
            self.map_arr, self.map_meta = _load_map(my)
            self._build_track_mask(self.track_margin)
            self._build_raycast_map()
            self.get_logger().info(f'맵 로드: {os.path.basename(my)}')
        else:
            self.get_logger().warn(
                f'맵을 못 읽었다 ({my!r}) — 트랙 이탈 검사와 레이캐스팅이 꺼진다. '
                f'후보 선택이 장애물 여유만 보게 되므로 트랙 밖으로 나갈 수 있다.')
            self.fr_bounds = False

        # ── ROS ───────────────────────────────────────────────────────
        self.amcl_scan_pub = (
            self.create_publisher(LaserScan, str(p('amcl_scan_topic')), 5)
            if self.amcl_mask_on else None)
        self.off_pub = self.create_publisher(Float64, '/frenet/offset', 10)
        self.spd_pub = self.create_publisher(Float64, '/frenet/speed_scale', 10)
        self.st_pub  = self.create_publisher(Float64, '/frenet/status', 10)
        self.diag_pub = self.create_publisher(Float64MultiArray, '/frenet/diag', 10)
        # 추월 가능 구간 — 래치로 한 번만 낸다 (기동 시 정해지고 안 바뀐다).
        #   RViz 에서 "지금 여기가 추월 구간인가" 를 눈으로 봐야 로그 없이도
        #   판단이 된다. 오버레이 PNG 와 **같은 함수**로 계산한 결과다.
        self.ot_pub  = self.create_publisher(
            MarkerArray, '/overtake_zone',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.fr_pub  = (self.create_publisher(MarkerArray, '/frenet_candidates', 1)
                        if bool(p('debug')) else None)
        self.obs_pub = (self.create_publisher(MarkerArray, '/obstacle', 1)
                        if bool(p('debug')) else None)

        self.create_subscription(LaserScan, str(p('scan_topic')), self._scan_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 self._pose_cb, 10)
        self.create_subscription(Odometry, str(p('odom_topic')), self._odom_cb, 10)
        self.create_timer(1.0 / max(float(p('rate_hz')), 1.0), self._tick)

        self.get_logger().info(
            f'FrenetPlanner ready — 후보 {len(self.fr_grid)}개 '
            f'(±{self.fr_dmax:.2f}m/{step:.2f}m), 여유 {self.fr_clear:.2f}m, '
            f'호라이즌 {self.obs_h_min:.1f}~{self.obs_h_max:.1f}m '
            f'({self.obs_h_sec:.1f}s)')
        self._publish_overtake_zone()

    def _publish_overtake_zone(self):
        """추월 가능 구간을 RViz 로 (하늘색 = 나가도 되는 곳). 기동 시 1회.

        ★ 안 켜져 있으면 **명시적으로 그 사실을 로그로 남긴다.** 조용히 아무
          마커도 안 뜨면 "안 켠 것" 과 "계산 실패" 를 구분할 수 없다.
        """
        if not self.ot_zone_on:
            self.get_logger().info('[추월구간] ot_zone=false — 어디서든 회피한다')
            return
        zone = self._overtake_zone()
        lut = self._s_lut()
        if zone is None or lut is None:
            self.get_logger().warn('[추월구간] 계산 실패 — 웨이포인트/맵 확인')
            return
        xy = lut['xy']
        arr = MarkerArray()
        mk = Marker()
        mk.header.frame_id = 'map'
        mk.header.stamp = self.get_clock().now().to_msg()
        mk.ns = 'overtake_zone'; mk.id = 0
        mk.type = Marker.LINE_LIST; mk.action = Marker.ADD
        mk.scale.x = 0.10
        mk.color.r, mk.color.g, mk.color.b, mk.color.a = 0.0, 0.75, 1.0, 0.8
        n = len(xy)
        for i in range(n):
            j = (i + 1) % n
            if not (zone[i] and zone[j]):
                continue
            for q in (xy[i], xy[j]):
                pt = Point(); pt.x = float(q[0]); pt.y = float(q[1]); pt.z = 0.03
                mk.points.append(pt)
        arr.markers.append(mk)
        self.ot_pub.publish(arr)
        self.get_logger().info(
            f'[추월구간] {100.0 * zone.mean():.0f}% '
            f'(폭≥{self.ot_width:.2f}m, 끝에서 {self.ot_exit_pad:.1f}m 잘라냄) '
            f'→ RViz /overtake_zone')

    # ── 입력 ─────────────────────────────────────────────────────────
    def _pose_cb(self, msg: PoseWithCovarianceStamped):
        q = msg.pose.pose.orientation
        self.pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _odom_cb(self, msg: Odometry):
        self.speed = float(msg.twist.twist.linear.x)
        self.yaw_rate = float(msg.twist.twist.angular.z)

    # ── 본체 ─────────────────────────────────────────────────────────
    def _tick(self):
        if self.pos is None or self.wps is None:
            return
        # 유지 로직이 쓰는 현재 시각 (믹스인은 노드가 아니라 clock 이 없다)
        self._now_sec = self.get_clock().now().nanoseconds * 1e-9
        t0 = time.perf_counter()
        d = self._frenet_select(float(self.pos[0]), float(self.pos[1]), self.yaw)
        m = Float64();  m.data = float(d);  self.off_pub.publish(m)
        s = Float64();  s.data = float(self._fr_status);  self.st_pub.publish(s)

        # ── 속도 스케일 ──────────────────────────────────────────────
        #   vmax: 고른 후보 v_max / 라인 프로파일 (기본).
        #   room: 옛 통로너비 선형 — path 선택 ablation 의 baseline 속도.
        #   BLOCKED 만 최저속도. FOLLOW 는 추종 법칙이 정한다 (status>=2 로
        #   묶이면 추월금지 구간에서 항상 기어간다).
        v_ref = max(float(getattr(self, '_fr_v_ref', 0.0) or 0.0), 1e-3)
        vmax = float(getattr(self, '_fr_vmax', 0.0) or 0.0)
        mode = int(getattr(self, '_fr_mode', 0) or 0)
        if mode == BLOCKED:
            scale = self.sp_min_scale
        elif self.sp_mode == 'room':
            r = float(np.clip(self._fr_gap_room / self.sp_room_ref, 0.0, 1.0))
            scale = self.sp_min_scale + (1.0 - self.sp_min_scale) * r
        else:
            scale = float(np.clip(vmax / v_ref, self.sp_min_scale, 1.0))

        # ★ 옆으로 다 옮길 수 있는 속도로 상한을 건다 (회피 중일 때만).
        #   콘이 2m 안에서야 보이는데 오프셋 이동에 4m 가 필요하면, 남는 방법은
        #   속도를 줄여 거리를 버는 것뿐이다.
        vcap = self._shift_feasible_speed(float(self.pos[0]), float(self.pos[1]), d)
        if np.isfinite(vcap):
            scale = min(scale, float(np.clip(vcap / v_ref, self.sp_min_scale, 1.0)))

        # ★ 그리고 **보이기 전에도** 접근속도를 제한한다. 검출 거리는 하드웨어가
        #   정하므로(2.7m), 그 거리 안에 콘이 나타나도 피할 수 있는 속도로만 달린다.
        acap = self._approach_speed_cap()
        if np.isfinite(acap):
            scale = min(scale, float(np.clip(acap / v_ref, self.sp_min_scale, 1.0)))

        # ★ 추월 금지 구간이면 상대 뒤를 따라간다 — 옆으로 못 푸니 종으로 푼다.
        if mode == FOLLOW or (self.ot_zone_on and not getattr(self, '_ot_allowed', True)):
            scale = min(scale, self._follow_scale(float(self.pos[0]),
                                                  float(self.pos[1])))
        sc = Float64();  sc.data = float(scale);  self.spd_pub.publish(sc)

        ms = (time.perf_counter() - t0) * 1e3
        diag = self.frenet_diag_dict(scale, ms)
        arr = Float64MultiArray(); arr.data = pack_frenet_diag(diag)
        self.diag_pub.publish(arr)

        # ★ 컨트롤러가 실제로 옮기고 있는 위치를 여기서도 따라간다.
        #   예전엔 _shift_cur 이 **영원히 0** 이었다. 그러면
        #     · 비용의 w_change·|d − 현재| 가 w_offset·|d| 와 똑같아져
        #       히스테리시스 항이 아무 일도 안 한다 (오프셋 벌점만 두 배)
        #     · "지금 어느 통로에 있나" 를 몰라 좌우 유지 판정이 불가능하다
        #   컨트롤러와 같은 램프 속도를 써야 실제 위치와 어긋나지 않는다.
        step = self._shift_rate * self._dt
        self._shift_cur += float(np.clip(d - self._shift_cur, -step, step))

        ttc = float(diag.get('min_ttc', 1e9))
        ttc_s = 'inf' if ttc >= 1e8 else f'{ttc:.2f}s'
        self.get_logger().info(
            f'[frenet] {PLANNER_MODE_NAME.get(mode, mode)}  '
            f'd={d:+.2f}m  n_valid={int(diag["n_valid"])}/{int(diag["n_candidates"])}  '
            f'n_obs={int(diag["n_obs"])}  '
            f'vmax={vmax:.2f}/{v_ref:.2f}  ttc={ttc_s}  '
            f'{ms:.1f}ms  속도×{scale:.2f}'
            + ('' if not self.ot_zone_on else
               ('  [추월가능]' if getattr(self, '_ot_allowed', True)
                else f'  [추월금지 추종 gap={self._follow_gap:.2f}m]'))
            + (f'  동적 {self._opp_moving}개' if self._opp_moving else '')
            + ('' if getattr(self, '_lock_d', None) is None else
               f'  lock={self._lock_d:+.2f}m'),
            throttle_duration_sec=0.5)

    def _load_waypoints(self, path: str):
        import csv as _csv
        if not path or not os.path.exists(path):
            self.get_logger().error(f'웨이포인트를 못 읽었다: {path!r} — 회피 불가')
            return
        rows = []
        with open(path) as f:
            for r in _csv.reader(f):
                if not r or r[0].lstrip().startswith('#'):
                    continue
                try:
                    rows.append([float(x) for x in r[:3]])
                except ValueError:
                    continue
        if len(rows) < 5:
            self.get_logger().error(f'웨이포인트가 너무 적다 ({len(rows)}개)')
            return
        self.wps = np.array(rows)
        ds = np.hypot(np.diff(self.wps[:, 0]), np.diff(self.wps[:, 1]))
        self._wp_ds = float(np.median(ds)) if len(ds) else 0.1
        self.get_logger().info(
            f'웨이포인트 {len(self.wps)}개, 간격 {self._wp_ds:.3f}m')


def main():
    rclpy.init()
    node = FrenetPlanner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
