"""race.launch.py — 자율주행 모드 (q1 / q2 / q3 / race).

RACE_SCRIPT 환경변수(기본: ~/race_script.yaml)의 설정을 읽어
전체 스택을 한 번에 기동한다:

  f1tenth_stack 브링업 (LiDAR + VESC + odom + mux)
  → map_server + AMCL (저장된 맵 위에서 위치 추정)
  → mission_manager (랩 카운트, 자동 정지)
  → safety_node (TTC 기반 AEB)
  → 컨트롤러 (모드에 따라 pure_pursuit 또는 gap_follower)
"""
import os
import yaml
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, DeclareLaunchArgument,
                            TimerAction, ExecuteProcess)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# ★ race 는 2026-08-13 부터 map 이다. gap_follower 는 레이싱라인을 **아예 안 쓰고**
#   매 프레임 가장 넓은 갭으로만 달리므로 1대1 에서 상대차를 피할 수는 있어도
#   빠르지 않고, 회피 후 최적경로로 돌아오지도 않는다. 1대1 은
#   "레이싱라인 + 동적 회피" 여야 하므로 map + frenet_planner 조합을 쓴다.
DEFAULT_CONTROLLER = {'q1': 'pure_pursuit', 'q2': 'gap_follower',
                      'q3': 'pure_pursuit', 'race': 'map'}

# 'map' = MAP(Model-based Adaptive Pure pursuit) 컨트롤러. 트랙 맵(map_name)과
# 무관한 알고리즘 이름이다 — race_script.yaml 의 [map] 절 참조.
VALID_CONTROLLERS = {'pure_pursuit', 'gap_follower', 'stanley', 'map'}


def generate_launch_description():
    # ---------- race_script.yaml 로드 ----------
    cfg_path = os.environ.get(
        'RACE_SCRIPT', os.path.expanduser('~/race_script.yaml'))
    with open(cfg_path) as f:
        cfg = yaml.safe_load(f)

    # ── 차량별 값 덮어쓰기 ────────────────────────────────────────────
    # race_script.yaml 은 두 차량 공용이고, 이 차만의 값(캘리브레이션·기하·
    # 튜닝)은 config/vehicle_<차>.yaml 에서 온다. race.sh 가 apply_vehicle.py
    # 로 펼쳐 놓은 것을 여기서 얹는다.
    #   ★ "어느 키가 어디로 가는가" 는 apply_vehicle.py 에만 있다. 여기서는
    #     통째로 deep-merge 만 한다 — 매핑을 두 군데 두면 반드시 어긋난다.
    _ovl_path = os.environ.get('RACE_VEHICLE_OVERLAY', '')
    if _ovl_path and os.path.exists(_ovl_path):
        def _deep(dst, src):
            for k, v in src.items():
                if isinstance(v, dict) and isinstance(dst.get(k), dict):
                    _deep(dst[k], v)
                else:
                    dst[k] = v
        with open(_ovl_path) as f:
            _ovl = yaml.safe_load(f) or {}
        _deep(cfg, _ovl)
        print(f'[race.launch] 차량 덮어쓰기 적용: {_ovl_path}')

    # mode:=/controller:=/bench:= 오버라이드는 race.sh가 launch arg로 준다.
    # sys.argv를 직접 파싱한다 (LaunchConfiguration은 파이썬 로직에서 못 씀)
    import sys
    mode = cfg['mode']
    controller = cfg.get('controller', 'auto')
    bench = False
    for a in sys.argv:
        if a.startswith('mode:='):
            mode = a.split(':=')[1]
        elif a.startswith('controller:='):
            controller = a.split(':=')[1]
        elif a.startswith('bench:='):
            bench = a.split(':=')[1].lower() in ('true', '1', 'yes')
    if mode not in DEFAULT_CONTROLLER:
        raise RuntimeError(f'race.launch.py: 잘못된 mode "{mode}"')

    # ── 속도 프로파일 공통 설정 (2026-08-09) ───────────────────────────
    #  grip_util 은 컨트롤러 셋(map/stanley/gap_follower)이 **공유**한다.
    #  모드별로 위험도를 다르게 두고 싶으면 limits.grip_util.<mode> 로 준다 —
    #  q*_max 같은 평평한 상한 대신 **마찰 예산 사용률**로 표현하는 것이 물리적이다.
    #
    #  ★ 반드시 mode:= 오버라이드를 **적용한 뒤에** 풀어야 한다 (2026-08-09).
    #    예전엔 이 블록이 위에 있어서 `./race.sh q2` 가 yaml 의 mode(q1) 로
    #    grip_util 을 골랐다. 컨트롤러는 q2(회피)로 뜨는데 속도는 q1(η=0.8) 이라
    #    실차에서 4.9㎧ 로 장애물에 돌진했고, 회피가 "전혀 안 되는" 것처럼 보였다.
    #    로그상 q2 주행의 cmd_v 중앙이 q1 과 동일한 3.02㎧ 로 찍혀 드러났다.
    _lim = cfg.get('limits', {})
    _mp0 = cfg.get('map', {})
    _gu  = _lim.get('grip_util', 0.6)
    grip_util = float(_gu.get(mode, _gu.get('default', 0.6))
                      if isinstance(_gu, dict) else _gu)
    a_lon_acc = float(_lim.get('a_lon_acc', 0.0))
    a_lon_brk = float(_lim.get('a_lon_brk', 0.0))
    launch_acc  = float(_lim.get('launch_acc',  0.0))
    launch_time = float(_lim.get('launch_time', 3.0))
    hard_cap  = float(cfg.get('speed', {}).get('hard_cap', 0.0))
    veh = dict(mu=float(_mp0.get('mu', 0.348)), pac_d=float(_mp0.get('pac_d', 1.15)),
               lf=float(_mp0.get('lf', 0.1757)), lr=float(_mp0.get('lr', 0.1441)))

    if controller == 'auto':
        controller = DEFAULT_CONTROLLER[mode]
    if controller not in VALID_CONTROLLERS:
        raise RuntimeError(
            f'race.launch.py: invalid controller "{controller}". '
            f'Valid: {sorted(VALID_CONTROLLERS)}')

    # ★ q*_max 는 삭제됐다 (2026-08-09) — 속도는 물리(limits.grip_util)가 정한다.
    #   max_speed 는 컨트롤러가 더 이상 클램프에 쓰지 않지만, 파라미터 선언이
    #   남아 있어 값은 넘겨준다. hard_cap(0=끔) 이 유일한 천장이다.
    max_speed = float(cfg['speed'].get('hard_cap', 0.0)) or 99.0
    min_speed = float(cfg['speed'].get('min_speed', 0.3))

    # ---------- bench(박스 위) 모드 ----------
    # 차를 박스에 올려두고 토픽/RViz/조향만 확인하는 용도. 바퀴는 절대 돌리지 않는다.
    # 바퀴가 돌면 VESC 엔코더 odom 은 "전진 중"이라 하고 LiDAR 는 "제자리"라 해서
    # AMCL 이 요동친다 → 정작 확인하려던 RViz 위치가 쓰레기가 되고, 출발선 반경을
    # 들락날락하며 mission_manager 가 가짜 랩을 세어 /race/enabled 를 꺼버린다.
    #
    # ★★ 2026-08-09 사고 — bench 가 **완전히 무력화돼 있었다.** ★★
    #   예전 구현은 "세 컨트롤러가 최종 속도를 clip(v, min_speed, max_speed) 하므로
    #   둘 다 0 이면 정지" 였다. 그런데 08/09 속도 개편(81730cf/d8324a9)에서
    #   **그 클램프를 전부 없앴다.** 속도는 이제 마찰 예산 프로파일이 정하고
    #   max_speed 를 아무도 안 본다 → bench 가 아무 효과가 없다.
    #   동시에 아래 start_gate 를 `and not bench` 로 꺼두고 있어서,
    #   `./race.sh box` 가 **시작 버튼도 없이 풀속도 자율주행**이 됐다.
    #   실차에서 실제로 발생했다.
    #
    #   교훈: 안전 기능을 "다른 코드가 우연히 하는 일"에 얹으면 안 된다.
    #   그 코드는 다른 이유로 사라진다. 이제 bench 는 **각 컨트롤러의 발행 직전**
    #   한 곳에서 속도를 0 으로 못박는다(`bench` 파라미터). 리팩터링으로 지워질 수
    #   있는 중간 계산이 아니라 최종 출력점이다.
    #
    #   max_speed/min_speed 0 은 남겨둔다 — 지금은 min_speed 만 프로파일 바닥으로
    #   쓰이지만, 층을 하나 더 두는 비용이 0 이다.
    if bench:
        max_speed = 0.0
        min_speed = 0.0

    race_share = get_package_share_directory('race_stack')
    f1tenth_share = get_package_share_directory('f1tenth_stack')

    # 맵/웨이포인트/경계는 빌드 시점 스냅샷(share)이 아니라 repo src 에서 직접 읽는다.
    # RACE_WS 가 있고 그 src/race_stack/maps 가 실재하면 이를 우선 사용 → 새 맵·
    # 웨이포인트를 만들어도 colcon build 없이 즉시 인식된다 (gen_waypoints 와 동일 규약).
    # 없으면 기존처럼 share 로 폴백.
    _ws = os.environ.get('RACE_WS')
    _src = os.path.join(_ws, 'src', 'race_stack') if _ws else None
    race_data = _src if _src and os.path.isdir(os.path.join(_src, 'maps')) \
                else race_share

    map_yaml = os.path.join(race_data, 'maps', cfg['map_name'] + '.yaml')
    wp_csv = os.path.join(race_data, 'waypoints', cfg['waypoints'] + '.csv')
    amcl_yaml = os.path.join(race_share, 'config', 'amcl.yaml')

    if not os.path.exists(map_yaml):
        raise RuntimeError(
            f'맵 파일 없음: {map_yaml}\n'
            f'→ 주최측 맵을 src/race_stack/maps/에 넣고 colcon build 하거나,\n'
            f'  ./race.sh mapping 으로 직접 맵을 만든 뒤 ./race.sh savemap 하세요.')

    nodes = []

    # ---------- 1. 차량 브링업 ----------
    nodes.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(f1tenth_share, 'launch', 'bringup_launch.py'))))

    # ---------- 1-1. odom TF 발행자 (IMU yaw 융합) ----------
    # ★ vesc.yaml 에서 vesc_to_odom_node.publish_tf: false 로 껐기 때문에
    #   odom→base_link TF 는 이 노드만 발행한다. 빠지면 AMCL 이 전부 멈춘다.
    #   imu_yaw: false 면 들어온 odom 을 그대로 중계해 기존 동작과 동일해진다.
    loc = cfg.get('localization', {})
    # 컨트롤러·AEB·deskew 가 모두 같은 오도메트리를 봐야 한다. 원본 /odom 은
    # 각속도가 서보 명령 기반이라 진로 예측이 틀어진다 (2026-08-04 전환).
    odom_topic = str(loc.get('odom_topic', '/odom_imu'))
    nodes.append(Node(
        package='race_stack', executable='imu_odom', name='imu_odom',
        output='screen',
        parameters=[{
            'imu_yaw':             bool( loc.get('imu_yaw',             True)),
            'imu_topic':           str(  loc.get('imu_topic',           '/sensors/imu/raw')),
            'gyro_units':          str(  loc.get('imu_gyro_units',      'deg')),
            'yaw_axis':            str(  loc.get('imu_yaw_axis',        'z')),
            'yaw_sign':            float(loc.get('imu_yaw_sign',        1.0)),
            # 병진 속도 부호 (차량별). vehicle_car*.yaml 의 localization.speed_sign.
            'speed_sign':          float(loc.get('speed_sign',          1.0)),
            'bias_warmup_sec':     float(loc.get('imu_bias_warmup_sec', 3.0)),
            'bias_alpha':          float(loc.get('imu_bias_alpha',      0.002)),
            'stationary_speed':    float(loc.get('imu_stationary_speed', 0.05)),
            'stationary_rate_deg': float(loc.get('imu_stationary_rate_deg', 2.0)),
        }]))

    # ---------- 1-2. 스캔 모션 보정 (deskew) ----------
    # 라이다 한 스캔(25ms) 동안 차가 움직이면 첫 빔과 마지막 빔이 다른 위치에서
    # 찍힌 것인데 메시지는 같은 순간처럼 담긴다. 코너에서 ω=2rad/s 면 스캔 도중
    # 2.9° 회전 → 5m 점이 25cm 어긋난다. AMCL 이 그 찌그러진 점을 맵에 맞추려다
    # 위치가 튄다. 속도를 올릴수록 정확히 비례해 나빠지므로 미리 넣어 둔다.
    deskew_on = bool(loc.get('scan_deskew', False))
    scan_topic = '/scan_deskewed' if deskew_on else '/scan'
    # ★ AMCL 만 마스킹된 스캔을 본다. 컨트롤러·AEB·회피는 원본을 그대로 쓴다 —
    #   빔을 지운 스캔으로 장애물을 찾으면 자기가 지운 것을 못 보게 된다.
    #   frenet_planner 가 뜨는 모드(q2·race)에서만 켤 수 있다.
    amcl_mask_on = (bool(loc.get('amcl_mask', False))
                    and bool(cfg.get('frenet', {}).get('enabled', True))
                    and mode in ('q2', 'race'))
    amcl_scan_topic = '/scan_amcl'
    amcl_input_topic = amcl_scan_topic if amcl_mask_on else scan_topic
    if deskew_on:
        nodes.append(Node(
            package='race_stack', executable='scan_deskew', name='scan_deskew',
            output='screen',
            parameters=[{
                'scan_in':       '/scan',
                'scan_out':      scan_topic,
                'odom_topic':    odom_topic,
                'stamp_at':      str(  loc.get('deskew_stamp_at',    'start')),
                'min_yaw_rate':  float(loc.get('deskew_min_yaw_rate', 0.05)),
                'min_speed':     float(loc.get('deskew_min_speed',    0.10)),
            }]))

    # ---------- 1-2b. 위치추정 품질 감시 ----------
    # 읽기 전용이다 — TF 도 /drive 도 건드리지 않으므로 켜 두어도 주행에 영향이
    # 없다. 매 스캔마다 "이 자세가 맞다면 라이다는 뭘 봐야 하는가" 를 맵
    # 레이캐스팅으로 구해 실측과 비교한다. 그 잔차가 위치추정 오차이고,
    # 잔차가 거리에 비례해 자라면 맵 스케일 문제다 (08/02 부터 미해결).
    # 어떤 필터(AMCL·SynPF)를 쓰든 같은 잣대라, 교체 효과를 재는 근거가 된다.
    if bool(loc.get('monitor', True)) and map_yaml:
        nodes.append(Node(
            package='race_stack', executable='localization_monitor',
            name='localization_monitor', output='screen',
            parameters=[{
                'map_yaml':    map_yaml,
                'scan_topic':  scan_topic,
                'odom_topic':  odom_topic,
                'fov_deg':     float(loc.get('monitor_fov_deg',   180.0)),
                'beam_deg':    float(loc.get('monitor_beam_deg',    2.0)),
                'max_range':   float(loc.get('monitor_max_range',   6.0)),
                'reject_m':    float(loc.get('monitor_reject_m',    1.0)),
                'report_sec':  float(loc.get('monitor_report_sec',  2.0)),
                'spatial_cell':       float(loc.get('monitor_cell',       0.4)),
                'spatial_report_sec': float(loc.get('monitor_map_sec',   20.0)),
            }]))

    # ---------- 1-3. 초기 위치 자동 정렬 ----------
    # AMCL 은 정지 상태에서 필터를 갱신하지 않아 /amcl_pose 를 아예 안 낸다.
    # 그래서 예전엔 RViz 2D Pose Estimate 를 사람이 찍어야 했는데, 대회
    # 출발선에서 노트북을 여는 건 현실적이지 않다. 이 노드가 스캔을 맵에
    # 맞춰 init_pose 주변을 훑고 /initialpose 를 대신 발행한다.
    # AMCL 이 활성화된 뒤여야 하므로 지연 기동한다.
    if bool(loc.get('auto_localize', True)):
        nodes.append(TimerAction(period=7.0, actions=[Node(
            package='race_stack', executable='auto_localize',
            name='auto_localize', output='screen',
            parameters=[{
                'map_yaml':       map_yaml,
                'scan_topic':     scan_topic,
                'init_x':         float(loc.get('init_x',   0.0)),
                'init_y':         float(loc.get('init_y',   0.0)),
                'init_yaw':       float(loc.get('init_yaw', 0.0)),
                'search_xy':      float(loc.get('align_search_xy',      0.5)),
                'search_yaw_deg': float(loc.get('align_search_yaw_deg', 15.0)),
                'step_xy':        float(loc.get('align_step_xy',        0.05)),
                'step_yaw_deg':   float(loc.get('align_step_yaw_deg',   1.0)),
                'fit_warn':       float(loc.get('align_fit_warn',       0.20)),
                'use_walls':      bool( loc.get('align_use_walls',      True)),
                'laser_offset_x': 0.27, 'laser_offset_y': 0.0,
            }])]))

    # ---------- 2. 맵 서버 + AMCL + lifecycle ----------
    nodes.append(Node(
        package='nav2_map_server', executable='map_server', name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_yaml,
                     'topic_name': 'map', 'frame_id': 'map'}]))
    nodes.append(Node(
        package='nav2_amcl', executable='amcl', name='amcl', output='screen',
        parameters=[amcl_yaml,
                    # ★ AMCL 만 마스킹된 스캔을 본다 (amcl_mask 켰을 때).
                    #   컨트롤러·AEB·회피는 원본 그대로 — 지운 스캔으로
                    #   장애물을 찾으면 자기가 지운 것을 못 보게 된다.
                    {'scan_topic': amcl_input_topic.lstrip('/'),
                     'initial_pose.x': float(loc.get('init_x', 0.0)),
                     'initial_pose.y': float(loc.get('init_y', 0.0)),
                     'initial_pose.yaw': float(loc.get('init_yaw', 0.0)),
                     'set_initial_pose': True}]))
    # lifecycle_manager 는 5초 지연 기동한다.
    # 동시에 띄우면 manager 가 map_server 보다 먼저 Configuring 을 호출하는 경쟁이
    # 생긴다. map_server 가 아직 lifecycle 서비스를 등록하기 전이라 전환이 실패하고,
    # map_server/amcl 이 unconfigured 로 멈춰 /map 발행자 0 · map→odom TF 없음이 된다.
    # (증상: RViz 에 맵이 안 뜨고 차 위치도 안 나옴. 실행할 때마다 되다 안 되다 함)
    nodes.append(TimerAction(period=5.0, actions=[Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_localization', output='screen',
        parameters=[{'autostart': True,
                     'bond_timeout': 10.0,
                     'attempt_respawn_reconnection': True,
                     'node_names': ['map_server', 'amcl']}])]))

    # ---------- 2-1. 조이스틱 수동 개입 ----------
    # 자율주행 중 LB 를 누르면 사람이 조종을 가져간다. 컨트롤러만 멈추고
    # AMCL·LiDAR·맵은 그대로 두므로 위치 추정이 끊기지 않고, 다시 LB 를 누르면
    # 그 자리에서 자율주행이 이어진다. 트랙 복귀·회수에 쓴다.
    #   · 자율로 시작(start_auto) → 평소에는 /teleop 을 안 낸다
    #   · 개입 시 /teleop(mux priority 100) 이 /drive(10) 를 덮는다
    #   · /race/enabled 는 건드리지 않는다 — mission_manager 가 /race/manual 을
    #     받아 해석한다 (한 토픽을 두 노드가 쓰면 상태가 뒤집힌다)
    jc = cfg.get('joystick', {})
    pp0 = cfg.get('pure_pursuit', {})
    if bool(jc.get('manual_override', True)):
        nodes.append(TimerAction(period=4.0, actions=[Node(
            package='race_stack', executable='joystick_teleop',
            name='joystick_teleop', output='screen',
            parameters=[{
                'axis_steer':       int(  jc.get('axis_steer',       0)),
                'axis_speed':       int(  jc.get('axis_speed',       5)),
                'btn_reverse':      int(  jc.get('btn_reverse',      5)),
                'btn_toggle':       int(  jc.get('btn_toggle',       4)),
                'steer_invert':     bool( jc.get('steer_invert',     False)),
                'speed_invert':     bool( jc.get('speed_invert',     False)),
                'trigger_invert':   bool( jc.get('trigger_invert',   True)),
                'trigger_deadzone': float(jc.get('trigger_deadzone', 0.05)),
                'trigger_expo':     float(jc.get('trigger_expo',     1.0)),
                'steer_deadzone':   float(jc.get('steer_deadzone',   0.08)),
                'arm_on_neutral':   bool( jc.get('arm_on_neutral',   True)),
                'debug':            False,
                'allow_toggle':     True,
                'start_auto':       True,     # 자율로 시작
                'max_speed':        float(jc.get('max_speed',
                                                 cfg['speed'].get('mapping_max', 1.0))),
                'max_steer':        float(pp0.get('max_steer', 0.36)),
            }])]))

    # ---------- 3. 미션 매니저 ----------
    m = cfg['mission']
    time_limit = int(m.get('time_limit_sec', 0))
    laps = int(m.get('laps', 0))
    if mode == 'q3' and time_limit == 0:
        time_limit = 120   # Q3 규정: 2분
    if bench:
        # 자동 정지 없음 — bench 는 사람이 Ctrl+C 로 끝낸다.
        laps = 0
        time_limit = 0
    nodes.append(Node(
        package='race_stack', executable='mission_manager',
        name='mission_manager', output='screen',
        parameters=[{'laps': laps,
                     'time_limit_sec': time_limit,
                     # ★ 2026-08-09: 예전엔 `and not bench` 로 bench 에서 출발
                     #   게이트를 껐다. "어차피 속도 0 이니 버튼 누를 필요 없다"는
                     #   전제였는데 그 전제가 깨지자(위 주석) **버튼 없이 풀속도로
                     #   출발**했다. bench 도 실주행과 똑같이 출발 버튼을 요구한다 —
                     #   차이는 바퀴가 안 도는 것뿐이어야 한다.
                     'start_gate':   bool(m.get('start_gate',   False)),
                     'start_button': int( m.get('start_button', 1)),
                     'start_key':    bool(m.get('start_key',    True)),
                     'start_x': float(m.get('start_x', 0.0)),
                     'start_y': float(m.get('start_y', 0.0)),
                     'start_radius': float(m.get('start_radius', 1.5))}]))

    # ---------- 4. 안전장치 (AEB) ----------
    s = cfg.get('safety', {})
    if s.get('enabled', True):
        nodes.append(Node(
            package='race_stack', executable='safety_node',
            name='safety_node', output='screen',
            parameters=[{
                'ttc_threshold':       float(s.get('ttc_threshold',       0.15)),
                'hold_sec':            float(s.get('hold_sec',            1.0)),
                'corridor_half_width': float(s.get('corridor_half_width', 0.30)),
                'brake_decel':         float(s.get('brake_decel',         3.0)),
                'min_range':           float(s.get('min_range',           0.12)),
                'max_lookahead':       float(s.get('max_lookahead',       6.0)),
                'consec_frames':       int(  s.get('consec_frames',       3)),
                'use_arc_corridor':    bool( s.get('use_arc_corridor',    True)),
                'publish_debug':       bool( s.get('publish_debug',       True)),
                'odom_topic':          odom_topic,
            }]))

    # ---------- 5. 컨트롤러 (AMCL 수렴 시간 확보를 위해 8초 지연 기동) ----------
    # lifecycle_manager 가 5초에 뜬 뒤 map_server→amcl 활성화까지 마쳐야 하므로
    # 그보다 뒤여야 한다. (앞서면 pose 가 None 이라 컨트롤러가 헛돈다)
    # ★ Q2 장애물 회피 기하 파라미터(차폭/여유/통로폭 등)는 stanley 와
    #   pure_pursuit 이 공유한다 (같은 트랙·같은 차라 튜닝을 하나로 유지).
    #   경계 파일도 마찬가지로 maps/<map_name>/ 하나를 같이 본다.
    sc = cfg.get('stanley', {})
    fr = cfg.get('frenet', {})
    bound_dir = sc.get('bound_dir', '') or \
        os.path.join(race_data, 'maps', cfg['map_name'])

    if controller == 'pure_pursuit':
        pp = cfg['pure_pursuit']
        ctrl = Node(
            package='race_stack', executable='pure_pursuit',
            name='pure_pursuit', output='screen',
            parameters=[{'mode': mode,
                         'waypoint_file': wp_csv,
                         'max_speed': max_speed, 'min_speed': min_speed,
                         # ★ bench 는 여기서 끝난다 — 컨트롤러가 발행 직전에 속도를 0 으로
                         #   못박는다. 중간 계산(클램프)에 얹지 말 것 (2026-08-09 사고).
                         'bench': bench,
                         'lookahead': float(pp['lookahead']),
                         'lookahead_speed_gain': float(pp['lookahead_speed_gain']),
                         'wheelbase': float(pp['wheelbase']),
                         'max_steer': float(pp['max_steer']),
                         'curvature_slowdown': float(pp['curvature_slowdown']),
                         # MAP 과 같은 프로파일·L 로 A/B (yaml 로 끔)
                         'use_profile': bool(pp.get('use_profile', True)),
                         'adaptive_ld': bool(pp.get('adaptive_ld', True)),
                         'l1_min': float(_mp0.get('l1_min', pp.get('l1_min', 1.4983))),
                         'l1_k':   float(_mp0.get('l1_k',   pp.get('l1_k',   0.75))),
                         'l1_kappa': float(pp.get('l1_kappa', 0.0)),
                         'grip_util': grip_util,
                         'mu': veh['mu'], 'pac_d': veh['pac_d'],
                         'lf': veh['lf'], 'lr': veh['lr'],
                         'a_lon_acc': a_lon_acc, 'a_lon_brk': a_lon_brk,
                         'hard_cap': hard_cap,
                         'speed_lag': float(_lim.get('speed_lag', 0.25)),
                         # ── Q2 장애물 회피 (옵션) ──────────────────────────
                         # pp.obstacle_avoid_enabled 하나로 껐다/켠다
                         # (race_script.yaml 의 pure_pursuit 절 참조).
                         'obstacle_avoid_enabled':
                             bool(pp.get('obstacle_avoid_enabled', False)),
                         'map_yaml':        map_yaml,
                         'bound_dir':       bound_dir,
                         'car_width':       float(sc.get('car_width',       0.30)),
                         'obstacle_margin': float(sc.get('obstacle_margin', 0.12)),
                         'obstacle_decel':  float(sc.get('obstacle_decel',  0.85)),
                         'obstacle_shift_rate': float(sc.get('obstacle_shift_rate', 0.9)),
                         'obs_corridor_half_width': float(sc.get('obs_corridor_half_width', 0.30)),
                         'obs_release_scale':       float(sc.get('obs_release_scale',       1.6)),
                         'obs_horizon_sec': float(sc.get('obs_horizon_sec', 0.3)),
                         'obs_horizon_min': float(sc.get('obs_horizon_min', 0.5)),
                         'obs_horizon_max': float(sc.get('obs_horizon_max', 3.0)),
                         'obs_vote_frames': int(  sc.get('obs_vote_frames',  5)),
                         'odom_topic':      odom_topic,
                         'laser_offset_x':  0.27, 'laser_offset_y': 0.0,
                         'grid_xmin':       0.35, 'grid_xmax':      5.0,
                         'grid_ymin':      -2.5,  'grid_ymax':      2.5,
                         'grid_res':        0.04, 'grid_safe_dist':  0.12,
                         'track_margin':    float(sc.get('track_margin', 0.28)),
                         'bound_ignore_px': 7.5}])

    elif controller == 'stanley':
        pp = cfg.get('pure_pursuit', {})
        wa = cfg.get('wall_avoid', {})

        ctrl = Node(
            package='race_stack', executable='stanley_controller',
            name='stanley_controller', output='screen',
            parameters=[{
                'mode':            mode,
                'waypoint_file':   wp_csv,
                'max_speed':       max_speed,
                # ★ bench 는 여기서 끝난다 — 컨트롤러가 발행 직전에 속도를 0 으로
                #   못박는다. 중간 계산(클램프)에 얹지 말 것 (2026-08-09 사고).
                'bench':           bench,
                'min_speed':       min_speed,
                # ★ 마찰원 속도 프로파일 (map_controller 와 같은 코드)
                'grip_util':       grip_util,
                'a_lon_acc':       a_lon_acc,
                'a_lon_brk':       a_lon_brk,
                # 출발 직후만 가속을 더 조인다 (map_controller 와 같은 코드)
                'launch_acc':      launch_acc,
                'launch_time':     launch_time,
                'hard_cap':        hard_cap,
                # 구동계 지연 보상 — 감속만 앞당긴다 (MAP 과 같은 preview_speed)
                'speed_lag':       float(_lim.get('speed_lag', 0.25)),
                'mu':              veh['mu'],
                'pac_d':           veh['pac_d'],
                'lf':              veh['lf'],
                'lr':              veh['lr'],
                'max_steer':       float(pp.get('max_steer', 0.36)),
                'wheelbase':       float(pp.get('wheelbase',  0.33)),
                'stanley_gain':    float(sc.get('gain',           0.8)),
                'stanley_k':       float(sc.get('k',              1.8)),
                'stanley_ks':      float(sc.get('ks',             1.3)),
                'vel_scale':       float(sc.get('vel_scale',      1.0)),
                'obstacle_decel':  float(sc.get('obstacle_decel', 0.85)),
                'obs_corridor_half_width': float(sc.get('obs_corridor_half_width', 0.30)),
                'obs_release_scale':       float(sc.get('obs_release_scale',       1.6)),
                'car_width':       float(sc.get('car_width',       0.30)),
                'obstacle_margin': float(sc.get('obstacle_margin', 0.12)),
                'obstacle_shift_rate': float(sc.get('obstacle_shift_rate', 0.9)),
                'obs_horizon_sec': float(sc.get('obs_horizon_sec', 0.3)),
                'obs_horizon_min': float(sc.get('obs_horizon_min', 0.5)),
                'obs_horizon_max': float(sc.get('obs_horizon_max', 3.0)),
                'obs_vote_frames': int(  sc.get('obs_vote_frames',  5)),
                # Frenet 후보 궤적 회피
                'frenet_enabled':    bool( fr.get('enabled',    True)),
                'frenet_candidates': int(  fr.get('candidates', 9)),
                'frenet_d_max':      float(fr.get('d_max',      0.60)),
                'frenet_clearance':  float(fr.get('clearance',  0.25)),
                'frenet_w_offset':   float(fr.get('w_offset',   1.0)),
                'frenet_w_change':   float(fr.get('w_change',   0.8)),
                'frenet_w_clear':    float(fr.get('w_clear',    2.0)),
                'frenet_clear_cap':  float(fr.get('clear_cap',  0.35)),
                'frenet_bounds':     bool( fr.get('bounds',     True)),
                'frenet_debug':      bool( fr.get('debug',      True)),
                'static_lock':       bool( fr.get('static_lock',  False)),
                'candidate_fb':      bool( fr.get('candidate_fb', False)),
                'fb_horizon':        float(fr.get('fb_horizon',   5.0)),
                'fb_iters':          int(  fr.get('fb_iters',     2)),
                'odom_topic':        odom_topic,
                # AMCL 첫 pose 를 받기 전까지의 위치 가정 (AMCL 과 같은 값)
                'init_x':            float(loc.get('init_x',   0.0)),
                'init_y':            float(loc.get('init_y',   0.0)),
                'init_yaw':          float(loc.get('init_yaw', 0.0)),
                'interp_scale':    int(  sc.get('interp_scale',   20)),
                'lookahead_min':   float(sc.get('lookahead_min',  0.5)),
                'lookahead_max':   float(sc.get('lookahead_max',  2.0)),
                'lookahead_gain':  float(sc.get('lookahead_gain', 7.0)),
                'ff_gain':         float(sc.get('ff_gain',        1.0)),
                'ff_preview_sec':  float(sc.get('ff_preview_sec', 0.10)),
                'steer_lpf':       float(sc.get('steer_lpf',      1.0)),
                # 벽 스침 회피
                'wall_avoid_enabled':    bool(wa.get('enabled',      False)),
                'wall_avoid_half_width': float(wa.get('half_width',  0.17)),
                'wall_avoid_clearance':  float(wa.get('clearance',   0.18)),
                'wall_avoid_lookahead':  float(wa.get('lookahead',   1.2)),
                'wall_avoid_gain':       float(wa.get('gain',        0.6)),
                'wall_avoid_max':        float(wa.get('max_steer',   0.10)),
                'wall_avoid_lpf':        float(wa.get('lpf',         0.4)),
                'wall_avoid_min_range':  float(wa.get('min_range',   0.12)),
                # Q2 obstacle detection
                'map_yaml':        map_yaml,
                'bound_dir':       bound_dir,
                # xmin 은 라이다 위치(0.27) 뒤쪽을 제외한다. 그 안쪽 셀은
                # 라이다 기준으로 "뒤"라 엉뚱한 빔을 참조하고, 어차피 그 거리의
                # 장애물은 피할 수도 없다. (실차에서 차 앞 0.11m 노이즈 검출됨)
                'grid_xmin':       0.35, 'grid_xmax':      5.0,
                'grid_ymin':      -2.5,  'grid_ymax':      2.5,
                'grid_res':        0.04, 'grid_safe_dist':  0.12,
                'bound_ignore_px': 7.5,
                'track_margin':    float(sc.get('track_margin', 0.28)),
                'use_bounds':      bool( sc.get('use_bounds',   False)),
                # 빔별 기대거리 비교 (레이캐스팅) — 벽 오검출 대책
                'raycast_obs':      bool( sc.get('raycast_obs',      True)),
                'raycast_pose_unc': float(sc.get('raycast_pose_unc', 0.25)),
                'raycast_deficit':  float(sc.get('raycast_deficit',  0.30)),
                'raycast_fov_deg':  float(sc.get('raycast_fov_deg',  140.0)),
                'raycast_beam_deg': float(sc.get('raycast_beam_deg', 0.25)),
                # 검사 (B) — 좁은 통로 한가운데 장애물. frenet_planner 와 같은 값
                'raycast_free_gap': float(sc.get('raycast_free_gap', 0.30)),
            }])

    elif controller == 'map':
        # MAP 컨트롤러 — 시뮬(f1tenth_racing/map_controller_node.py)에서 개발한
        # 것을 실차로 옮긴 것. 조향은 L1 가이던스 + Pacejka 역산, 속도는 CSV
        # 속도의 역방향 스윕 프로파일. 차량 기하(wheelbase/max_steer)는
        # pure_pursuit 절과 공유한다 — 같은 차이므로 두 군데 두지 않는다.
        pp = cfg.get('pure_pursuit', {})
        mp = cfg.get('map', {})
        fr = cfg.get('frenet', {})
        # ★ per_controller 는 컨트롤러 이름으로 찾되, **모드 키가 있으면 그게
        #   이긴다.** 1대1(race)은 같은 map 컨트롤러를 쓰지만 상대가 마주 달려
        #   와 접근속도가 두 배라 호라이즌이 더 필요하다 (2026-08-13).
        #   예전엔 'map' 으로 하드코딩돼 있어 race 전용 값을 넣어도 안 읽혔다.
        _pc = fr.get('per_controller', {})
        frc = dict(_pc.get('map', {}))
        frc.update(_pc.get(mode, {}))
        st = cfg.get('stanley', {})
        ctrl = Node(
            package='race_stack', executable='map_controller',
            name='map_controller', output='screen',
            parameters=[{
                'mode':          mode,
                'waypoint_file': wp_csv,
                'max_speed':     max_speed,
                # ★ bench 는 여기서 끝난다 — 컨트롤러가 발행 직전에 속도를 0 으로
                #   못박는다. 중간 계산(클램프)에 얹지 말 것 (2026-08-09 사고).
                'bench':         bench,
                'min_speed':     min_speed,
                'odom_topic':    odom_topic,
                'wheelbase':     float(pp.get('wheelbase', 0.33)),
                'max_steer':     float(pp.get('max_steer', 0.36)),
                'l1_min':        float(mp.get('l1_min',      0.2725)),
                'l1_k':          float(mp.get('l1_k',        0.2750)),
                # 코너에서만 L1 상한 (0 = 끔). 안쪽 파고듦 대책 — 2026-08-16
                'l1_curv_k':     float(mp.get('l1_curv_k',    1.0)),
                # 그 상한의 시간 하한 — 없으면 진입에서 조향이 포화돼 바깥으로 밀린다
                'l1_curv_tmin':  float(mp.get('l1_curv_tmin', 0.50)),
                'mass':          float(mp.get('mass',        3.74)),
                'lf':            float(mp.get('lf',          0.15875)),
                'lr':            float(mp.get('lr',          0.17145)),
                # ★ 폴백도 yaml [map] 과 같은 값이어야 한다. mu × pac_d = 0.40 (실효 그립).
                'mu':            float(mp.get('mu',          0.348)),
                'pac_b':         float(mp.get('pac_b',       3.8)),
                'pac_c':         float(mp.get('pac_c',       1.3)),
                'pac_d':         float(mp.get('pac_d',       1.15)),
                'pac_e':         float(mp.get('pac_e',      -0.5)),
                # ★ 속도 프로파일의 유일한 노브. 마찰 예산에 곱한다 (속도가 아니라).
                'grip_util':     grip_util,
                'hard_cap':      hard_cap,
                # 0 이면 물리에서 자동: 가속=μ_eff·g·lf/L(후륜하중), 제동=μ_eff·g
                'a_lon_acc':     a_lon_acc,
                'a_lon_brk':     a_lon_brk,
                # 출발 직후만 가속을 더 조인다 (휠 스핀 → 종방향 허위거리 방지)
                'launch_acc':    launch_acc,
                'launch_time':   launch_time,
                # 구동계 지연 + 제동거리 미리보기 (실측 지연 0.25~0.41s, 제동 ~2㎧²)
                'speed_lag':     float(_lim.get('speed_lag', 0.25)),
                # ↓ 폐기. 옛 설정이 죽지 않게만 남겨둔다 (컨트롤러가 경고하고 무시)
                'a_brake':       float(mp.get('a_brake',     0.0)),
                'speed_scale':   float(mp.get('speed_scale', 1.0)),
                'mu_cap':        bool( mp.get('mu_cap',      True)),
                'steer_alpha':   float(mp.get('steer_alpha', 0.1981)),
                # 슬립보정 이득 (1.0=모델값). 코너 과조향이면 낮춘다
                'slip_gain':     float(mp.get('slip_gain',   1.0)),
                # AMCL 보정 슬루 제한 (0=끔). 맵-실제 불일치 시 조향 킥 방지
                'pose_corr_rate':     float(loc.get('pose_corr_rate',     4.0)),
                'pose_corr_yaw_rate': float(loc.get('pose_corr_yaw_rate', 2.1)),
                'reverse_check': bool( mp.get('reverse_check', True)),
                # AMCL 첫 pose 전까지의 위치 가정 (stanley 와 같은 값을 쓴다).
                # 없으면 출발선에서 pose 를 기다리다 영영 못 움직인다.
                'init_x':        float(loc.get('init_x',   0.0)),
                'init_y':        float(loc.get('init_y',   0.0)),
                'init_yaw':      float(loc.get('init_yaw', 0.0)),
                'frenet_enabled':    bool( fr.get('enabled', True)),
                'frenet_timeout':    float(fr.get('timeout', 0.5)),
                'frenet_shift_rate': float(fr.get('shift_rate', 0.9)),
            }])

        # ── Frenet 회피 노드 (컨트롤러 공용) ────────────────────────────
        # MAP 은 회피 로직을 직접 갖지 않는다. Stanley 가 쓰던 것과 **같은 코드**를
        # (obstacle_avoid 믹스인) 독립 노드로 돌리고, 결과인 횡오프셋만 받는다.
        #
        # ★ 호라이즌은 컨트롤러마다 달라야 한다. MAP/L1 은 오프셋에 수렴하는 데
        #   1.8·L1/√2 ≈ 1.15m (L1=0.9m) 를 달려야 하는데, Stanley 기준으로 맞춰둔
        #   0.8m 를 그대로 쓰면 다 피하기 전에 장애물에 닿는다. 경험칙은
        #   **호라이즌 ≥ 2 × L1**. per_controller 에서 덮어쓴다.
        if bool(fr.get('enabled', True)) and mode in ('q2', 'race'):
            l1m = float(mp.get('l1_min', 0.90))
            nodes.append(Node(
                package='race_stack', executable='frenet_planner',
                name='frenet_planner', output='screen',
                parameters=[{
                    'mode':          mode,
                    'waypoints_csv': wp_csv,
                    'map_yaml':      map_yaml,
                    'odom_topic':    odom_topic,
                    # 컨트롤러별 오버라이드 → 없으면 2×L1 규칙으로 자동
                    'obs_horizon_sec': float(frc.get('obs_horizon_sec',
                                                     fr.get('obs_horizon_sec', 1.0))),
                    'obs_horizon_min': float(frc.get('obs_horizon_min',
                                                     max(2.0 * l1m, 0.8))),
                    'obs_horizon_max': float(frc.get('obs_horizon_max',
                                                     fr.get('obs_horizon_max', 3.0))),
                    'frenet_dmax':      float(fr.get('dmax', fr.get('d_max', 0.6))),
                    'frenet_dstep':     float(fr.get('dstep',     0.10)),
                    'frenet_clearance': float(fr.get('clearance', 0.35)),
                    'frenet_clear_cap': float(fr.get('clear_cap', 0.8)),
                    'frenet_w_offset':  float(fr.get('w_offset',  1.0)),
                    'frenet_w_change':  float(fr.get('w_change',  0.5)),
                    'frenet_w_clear':   float(fr.get('w_clear',   2.0)),
                    'frenet_w_speed':   float(fr.get('w_speed',   0.5)),
                    'frenet_w_ttc':     float(fr.get('w_ttc',     0.0)),
                    'frenet_ttc_ref':   float(fr.get('ttc_ref',   2.0)),
                    'frenet_ttc_min':   float(fr.get('ttc_min',   0.5)),
                    'frenet_bounds':    bool( fr.get('bounds',    True)),
                    'frenet_votes':     int(  fr.get('votes',     3)),
                    # 통과 여유 하한 = 차 반폭 + 안전여유. 옛 마스크 판정이
                    # 레이싱라인 자신을 27% 프레임에서 탈락시켰다 (2026-08-11)
                    'frenet_min_room':  float(fr.get('min_room',  0.22)),
                    'frenet_gap_hyst':  float(fr.get('gap_hyst',  0.10)),
                    # ── 1대1(race) 동적 장애물 ──
                    'opp_track':      bool( fr.get('opp_track',     mode == 'race')),
                    'opp_eps':        float(fr.get('opp_eps',       0.30)),
                    'opp_gate':       float(fr.get('opp_gate',      0.80)),
                    'opp_alpha':      float(fr.get('opp_alpha',     0.6)),
                    'opp_min_obs':    int(  fr.get('opp_min_obs',   3)),
                    'opp_min_speed':  float(fr.get('opp_min_speed', 0.5)),
                    'opp_pred_max':   float(fr.get('opp_pred_max',  1.2)),
                    # 추월 가능 구간 (1대1). 기본은 race 모드에서만 켠다
                    # ★ frc(모드별) → fr(공용) → 모드 기본 순. q2 는 꺼야 한다:
                    #   콘은 짧아서 살짝 비키면 되고, 구간을 걸면 좁은 데 놓인
                    #   콘을 아예 못 피한다 (2026-08-15).
                    'ot_zone':        bool( frc.get('ot_zone',
                                            fr.get('ot_zone', mode == 'race'))),
                    'ot_width':       float(fr.get('ot_width',     1.10)),
                    'ot_exit_pad':    float(fr.get('ot_exit_pad',  2.50)),
                    'ot_min_len':     float(fr.get('ot_min_len',   2.00)),
                    'follow_gap':     float(fr.get('follow_gap',   0.80)),
                    'follow_k':       float(fr.get('follow_k',     1.2)),
                    'follow_min':     float(fr.get('follow_min',   0.25)),
                    # 다 지나갈 때까지 오프셋 유지 (상대가 시야에서 사라져도)
                    'hold':           bool( fr.get('hold',         mode == 'race')),
                    'hold_car_len':   float(fr.get('hold_car_len', 0.50)),
                    'hold_obj_len':   float(fr.get('hold_obj_len', 0.50)),
                    'hold_margin':    float(fr.get('hold_margin',  0.30)),
                    'hold_max_sec':   float(fr.get('hold_max_sec', 3.0)),
                    # 테스트 하이브리드 — yaml 로 A/B. 둘 다 기본 켬.
                    'static_lock':    bool( fr.get('static_lock',  False)),
                    'candidate_fb':   bool( fr.get('candidate_fb', False)),
                    'fb_horizon':     float(fr.get('fb_horizon',   5.0)),
                    'fb_iters':       int(  fr.get('fb_iters',     2)),
                    'speed_min_scale':float(fr.get('speed_min_scale', 0.45)),
                    'speed_room_ref': float(fr.get('speed_room_ref',  0.45)),
                    'speed_scale_mode': str(fr.get('speed_scale_mode', 'vmax')),
                    'grip_util':        grip_util,
                    'mu':               veh['mu'],
                    'pac_d':            veh['pac_d'],
                    # speed_scale 분모를 컨트롤러와 **같은 물리**로 풀기 위한 값들
                    'lf':               veh['lf'],
                    'lr':               veh['lr'],
                    'min_speed':        min_speed,
                    'a_lon_acc':        a_lon_acc,
                    'a_lon_brk':        a_lon_brk,
                    'hard_cap':         hard_cap,
                    # 컨트롤러의 램프 속도와 같아야 한다 — 플래너가 "지금 어디에
                    # 있는지" 를 이걸로 안다 (히스테리시스·좌우 유지)
                    'shift_rate':       float(fr.get('shift_rate', 2.0)),
                    # 컨트롤러의 L1 — 회피 완료 거리 계산용. 갈리면 상한이 틀어진다
                    # ⚠ 2026-08-16 부터 컨트롤러는 코너에서 L1 을 l1_curv_k/|κ| 로
                    #   자른다(map.l1_curv_k). 플래너는 그 상한을 **안 본다** —
                    #   즉 코너에서 실제 L1 보다 길게 잡는다. 이동거리
                    #   1.8·L1/√2 를 과대평가하는 쪽이라 회피 속도상한이 필요보다
                    #   보수적으로 나오는 것뿐이라 안전 방향이고, 그래서 지금은
                    #   일부러 안 맞췄다. 코너 회피가 과하게 느리면 여기부터 볼 것.
                    'l1_min':           float(mp.get('l1_min', 1.32)),
                    'l1_k':             float(mp.get('l1_k',   0.75)),
                    'spd_span':         float(fr.get('spd_span',     1.20)),
                    'shift_cap':        bool( fr.get('shift_cap',    True)),
                    'shift_safety':     float(fr.get('shift_safety', 1.25)),
                    'shift_min_v':      float(fr.get('shift_min_v',  0.8)),
                    # 움직이는 상대는 gap 을 접근속도로 환산 — 2026-08-16
                    'shift_close_min':  float(fr.get('shift_close_min', 0.30)),
                    'shift_close_max':  float(fr.get('shift_close_max', 6.0)),
                    'shift_close_r':    float(fr.get('shift_close_r',   0.60)),
                    'shift_done':       float(fr.get('shift_done',      0.05)),
                    'shift_preempt':    bool( fr.get('shift_preempt', True)),
                    'shift_det_m':      float(fr.get('shift_det_m',   2.70)),
                    'shift_brake':      float(fr.get('shift_brake',   0.0)),
                    # 후보 경로 곡률이 조향 한계를 넘으면 탈락 (안쪽 오프셋)
                    'wheelbase':        float(pp.get('wheelbase', 0.33)),
                    'max_steer':        float(pp.get('max_steer', 0.36)),
                    'debug':            bool( fr.get('debug',     True)),
                    # 레이캐스팅은 [stanley] 값을 재사용한다 — 같은 트랙·같은 차다
                    'raycast_obs':      bool( st.get('raycast_obs',      True)),
                    # ★ 화각은 모드별로 다르다 (2026-08-15).
                    #   q2: 콘을 **절대** 안 건드려야 하니 넓게 본다(140°).
                    #   race: 옆(±70°)에 있는 상대는 이미 나란히 달리는 중이라
                    #     피할 것이 없다 — 그 구간은 hold(오프셋 유지)가 덮는다.
                    #     넓게 보면 코너 안쪽 벽만 더 주워 담아 손해다.
                    'raycast_fov_deg':  float(frc.get('raycast_fov_deg',
                                          st.get('raycast_fov_deg', 140.0))),
                    'raycast_beam_deg': float(st.get('raycast_beam_deg', 0.25)),
                    'raycast_deficit':  float(st.get('raycast_deficit',  0.30)),
                    'raycast_pose_unc': float(st.get('raycast_pose_unc', 0.25)),
                    # 검사 (B) — 빔 끝점이 벽에서 이만큼 떨어지면 장애물.
                    # 좁은 통로 한가운데 장애물의 원거리 검출률을 3~10배로 올린다
                    'raycast_free_gap': float(st.get('raycast_free_gap', 0.30)),
                    # AMCL 어긋남 자기보정 (0=끔)
                    'skew_gain':        float(st.get('skew_gain', 1.0)),
                    'skew_max':         float(st.get('skew_max',  0.35)),
                    # AMCL 용 스캔 마스킹 (앞차가 벽을 가릴 때)
                    'amcl_mask':        bool( loc.get('amcl_mask',      False)),
                    'amcl_mask_pad':    int(  loc.get('amcl_mask_pad',  2)),
                    'amcl_mask_max':    float(loc.get('amcl_mask_max',  0.30)),
                    'amcl_scan_topic':  amcl_scan_topic,
                    'track_margin':     float(st.get('track_margin',     0.15)),
                }]))

    else:
        gf = cfg['gap_follower']
        ctrl = Node(
            package='race_stack', executable='gap_follower',
            name='gap_follower', output='screen',
            parameters=[{'max_speed': max_speed, 'min_speed': min_speed,
                         # ★ bench 는 여기서 끝난다 — 컨트롤러가 발행 직전에 속도를 0 으로
                         #   못박는다. 중간 계산(클램프)에 얹지 말 것 (2026-08-09 사고).
                         'bench': bench,
                         # ★ 조향각 → 곡률 → 마찰 예산으로 속도를 제한한다
                         'grip_util': grip_util, 'hard_cap': hard_cap,
                         'mu': veh['mu'], 'pac_d': veh['pac_d'],
                         'wheelbase': float(cfg['pure_pursuit']['wheelbase']),
                         'car_width': float(gf['car_width']),
                         'disparity_threshold': float(gf['disparity_threshold']),
                         'max_range': float(gf['max_range']),
                         'fov_deg': float(gf['fov_deg']),
                         'steer_gain': float(gf['steer_gain']),
                         'speed_gain': float(gf['speed_gain']),
                         'max_steer': float(cfg['pure_pursuit']['max_steer'])}])

    nodes.append(TimerAction(period=8.0, actions=[ctrl]))

    nodes.append(ExecuteProcess(
        cmd=['echo', f'[race.launch] mode={mode} controller={controller} '
                     f'max_speed={max_speed} map={cfg["map_name"]}'
                     + (' BENCH(바퀴 정지)' if bench else '')],
        output='screen'))

    return LaunchDescription(
        [DeclareLaunchArgument('mode', default_value=mode),
         DeclareLaunchArgument('controller', default_value=controller),
         DeclareLaunchArgument('bench', default_value=str(bench).lower())]
        + nodes)
