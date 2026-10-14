"""sim_race.launch.py — 통합 시뮬레이션 런치.

race_script.yaml 의 설정을 그대로 사용해 시뮬레이터에서 실차 알고리즘을 구동한다.

  physics: gym     → f1tenth_gym_ros (기본, 선형 타이어)
  physics: bicycle → vehicle_sim_node (Pacejka 자전거 모델, race_script.yaml map: 파라미터)

  controller 는 race_script.yaml controller: 필드를 따른다.
  (map / stanley / pure_pursuit 지원)

실행:
  export RACE_WS=~/f1tenth_ws
  export RACE_SCRIPT=~/yax/race_script.yaml
  source ~/yax/f1TENTH-contest_full_stack/sim_ws/install/setup.bash
  source ~/f1tenth_ws/install/setup.bash
  ros2 launch race_stack sim_race.launch.py
"""
import math
import os
import yaml

from launch import LaunchDescription
from launch.actions import TimerAction
from launch.substitutions import Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ── race_script.yaml 로드 ────────────────────────────────────────────
    cfg_path = os.environ.get('RACE_SCRIPT',
                              os.path.expanduser('~/race_script.yaml'))
    if not os.path.exists(cfg_path):
        raise RuntimeError(
            f'race_script.yaml 없음: {cfg_path}\n'
            'RACE_SCRIPT 환경변수를 설정하거나 ./race.sh sim 으로 실행하세요.')
    with open(cfg_path) as f:
        cfg = yaml.safe_load(f)

    # ── 맵·웨이포인트 경로 ────────────────────────────────────────────────
    _ws  = os.environ.get('RACE_WS')
    _src = os.path.join(_ws, 'src', 'race_stack') if _ws else None
    race_data = (_src if _src and os.path.isdir(os.path.join(_src, 'maps'))
                 else get_package_share_directory('race_stack'))

    map_name = cfg['map_name']
    map_base = os.path.join(race_data, 'maps', map_name)
    map_yaml = map_base + '.yaml'
    wp_csv   = os.path.join(race_data, 'waypoints', cfg['waypoints'] + '.csv')

    if not os.path.exists(map_yaml):
        raise RuntimeError(f'맵 파일 없음: {map_yaml}')
    if not os.path.exists(wp_csv):
        raise RuntimeError(f'웨이포인트 없음: {wp_csv}')

    map_img_ext = '.png' if os.path.exists(map_base + '.png') else '.pgm'

    # ── 스폰 위치 ─────────────────────────────────────────────────────────
    sim_cfg = cfg.get('sim', {})
    sx      = float(sim_cfg.get('sx',     0.853))
    sy      = float(sim_cfg.get('sy',     4.111))
    stheta  = float(sim_cfg.get('stheta', math.pi))
    physics = sim_cfg.get('physics', 'gym')   # 'gym' | 'bicycle'

    # ── 컨트롤러 선택 ─────────────────────────────────────────────────────
    mode       = cfg.get('mode', 'q1')
    controller = cfg.get('controller', 'map')
    # ★ q*_max 삭제됨 (2026-08-09). 속도는 limits.grip_util 이 정한다.
    max_speed  = float(cfg['speed'].get('hard_cap', 0.0)) or 99.0
    min_speed  = float(cfg['speed'].get('min_speed', 0.3))
    _lim = cfg.get('limits', {})
    _gu  = _lim.get('grip_util', 0.6)
    grip_util = float(_gu.get(mode, _gu.get('default', 0.6))
                      if isinstance(_gu, dict) else _gu)
    a_lon_acc = float(_lim.get('a_lon_acc', 0.0))
    a_lon_brk = float(_lim.get('a_lon_brk', 0.0))
    hard_cap  = float(cfg['speed'].get('hard_cap', 0.0))

    # ── map: 섹션 파라미터 ────────────────────────────────────────────────
    mc = cfg.get('map', {})
    map_params = {
        'odom_topic':   '/odom',
        'waypoint_file': wp_csv,
        'mode':          mode,
        'max_speed':     max_speed,
        'min_speed':     min_speed,
        'grip_util':     grip_util,
        'a_lon_acc':     a_lon_acc,
        'a_lon_brk':     a_lon_brk,
        'hard_cap':      hard_cap,
        'speed_lag':     float(_lim.get('speed_lag', 0.25)),
        'mu':            float(mc.get('mu',          1.0)),
        'mass':          float(mc.get('mass',         3.354)),
        'lf':            float(mc.get('lf',           0.1757)),
        'lr':            float(mc.get('lr',           0.1441)),
        'pac_b':         float(mc.get('pac_b',        3.8)),
        'pac_c':         float(mc.get('pac_c',        1.3)),
        'pac_d':         float(mc.get('pac_d',        1.15)),
        'pac_e':         float(mc.get('pac_e',       -0.5)),
        'l1_min':        float(mc.get('l1_min',       0.90)),
        'l1_k':          float(mc.get('l1_k',         0.35)),
        'speed_scale':   float(mc.get('speed_scale',  2.0)),
        'steer_alpha':   float(mc.get('steer_alpha',  0.1981)),
        'a_brake':       float(mc.get('a_brake',      6.0902)),
        'wheelbase':     float(mc.get('lf', 0.1757)) + float(mc.get('lr', 0.1441)),
        'max_steer':     float(cfg.get('pure_pursuit', {}).get('max_steer', 0.36)),
        'init_x':        sx,
        'init_y':        sy,
        'init_yaw':      stheta,
    }

    # ── RViz / RSP / 맵 서버 (공통) ──────────────────────────────────────
    sim_share = get_package_share_directory('f1tenth_gym_ros')

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz',
        arguments=['-d', os.path.join(sim_share, 'launch', 'gym_bridge.rviz')])

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='ego_robot_state_publisher',
        parameters=[{'robot_description': Command(
            ['xacro ', os.path.join(sim_share, 'launch', 'ego_racecar.xacro')])}],
        remappings=[('/robot_description', 'ego_robot_description')])

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{'yaml_filename': map_yaml,
                     'topic_name': 'map', 'frame_id': 'map'}])
    lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_sim',
        output='screen',
        parameters=[{'autostart': True, 'node_names': ['map_server']}])

    _start_gate = bool(cfg.get('mission', {}).get('start_gate', False))
    sim_bridge = Node(
        package='race_stack',
        executable='sim_bridge',
        name='sim_bridge',
        output='screen',
        parameters=[{'start_gate': _start_gate}])

    # ── 물리 시뮬레이터 선택 ──────────────────────────────────────────────
    if physics == 'bicycle':
        physics_node = Node(
            package='race_stack',
            executable='vehicle_sim_node',
            name='vehicle_sim',
            output='screen',
            parameters=[{
                'mu':      map_params['mu'],
                'mass':    map_params['mass'],
                'lf':      map_params['lf'],
                'lr':      map_params['lr'],
                'pac_b':   map_params['pac_b'],
                'pac_c':   map_params['pac_c'],
                'pac_d':   map_params['pac_d'],
                'pac_e':   map_params['pac_e'],
                'sx':      sx,
                'sy':      sy,
                'stheta':  stheta,
                'a_max':   float(cfg.get('sim_vehicle', {}).get('a_max', 9.51)),
                'a_brake': map_params['a_brake'],
                's_max':   map_params['max_steer'],
                'tau_v':   float(cfg.get('sim_vehicle', {}).get('tau_v', 0.25)),
            }])
    else:
        sv = cfg.get('sim_vehicle', {})
        veh_params = {
            'veh_mu':          float(sv.get('mu',          1.0489)),
            'veh_C_Sf':        float(sv.get('C_Sf',        4.718)),
            'veh_C_Sr':        float(sv.get('C_Sr',        5.4562)),
            'veh_lf':          float(sv.get('lf',          0.1757)),
            'veh_lr':          float(sv.get('lr',          0.1441)),
            'veh_h':           float(sv.get('h',           0.074)),
            'veh_m':           float(sv.get('m',           3.74)),
            'veh_I':           float(sv.get('I',           0.04712)),
            'veh_s_max':       float(sv.get('s_max',       0.4189)),
            'veh_sv_max':      float(sv.get('sv_max',      3.2)),
            'veh_a_max':       float(sv.get('a_max',       9.51)),
            'veh_v_max':       float(sv.get('v_max',       20.0)),
            'veh_steer_delay': int(  sv.get('steer_delay', 2)),
        }
        physics_node = Node(
            package='f1tenth_gym_ros',
            executable='gym_bridge',
            name='bridge',
            output='screen',
            parameters=[os.path.join(sim_share, 'config', 'sim.yaml'),
                        {'map_path':    map_base,
                         'map_img_ext': map_img_ext,
                         'num_agent':   1,
                         'kb_teleop':   False,
                         'sx': sx, 'sy': sy, 'stheta': stheta,
                         **veh_params}])

    # ── 컨트롤러 노드 선택 ────────────────────────────────────────────────
    sc = cfg.get('stanley', {})
    pp = cfg.get('pure_pursuit', {})
    bound_dir = sc.get('bound_dir', '') or os.path.join(race_data, 'maps', map_name)

    if controller == 'map':
        ctrl_node = Node(
            package='race_stack',
            executable='map_controller',
            name='map_controller',
            output='screen',
            parameters=[map_params])

    elif controller == 'pure_pursuit':
        ctrl_node = Node(
            package='race_stack',
            executable='pure_pursuit',
            name='pure_pursuit',
            output='screen',
            parameters=[{
                'odom_topic':   '/odom',
                'waypoint_file': wp_csv,
                'mode':          mode,
                'max_speed':     max_speed,
                'min_speed':     min_speed,
                'lookahead': float(pp.get('lookahead', 1.8)),
                'lookahead_speed_gain': float(pp.get('lookahead_speed_gain', 0.3)),
                'use_profile': bool(pp.get('use_profile', True)),
                'adaptive_ld': bool(pp.get('adaptive_ld', True)),
                'l1_min':  float(mc.get('l1_min', pp.get('l1_min', 0.90))),
                'l1_k':    float(mc.get('l1_k',   pp.get('l1_k',   0.35))),
                'l1_kappa': float(pp.get('l1_kappa', 0.0)),
                'max_steer': float(pp.get('max_steer', 0.36)),
                'wheelbase': float(pp.get('wheelbase', 0.33)),
                'curvature_slowdown': float(pp.get('curvature_slowdown', 1.5)),
                'grip_util': grip_util,
                'mu': float(mc.get('mu', 1.0)),
                'pac_d': float(mc.get('pac_d', 1.15)),
                'lf': float(mc.get('lf', 0.1757)),
                'lr': float(mc.get('lr', 0.1441)),
                'a_lon_acc': a_lon_acc, 'a_lon_brk': a_lon_brk,
                'hard_cap': hard_cap,
                'speed_lag': float(_lim.get('speed_lag', 0.25)),
                'init_x': sx, 'init_y': sy, 'init_yaw': stheta,
            }])

    else:  # stanley
        ctrl_node = Node(
            package='race_stack',
            executable='stanley_controller',
            name='stanley_controller',
            output='screen',
            parameters=[{
                'mode':            mode,
                'waypoint_file':   wp_csv,
                'max_speed':       max_speed,
                'min_speed':       min_speed,
                'max_steer':       float(pp.get('max_steer',       0.36)),
                'wheelbase':       float(pp.get('wheelbase',       0.33)),
                'stanley_gain':    float(sc.get('gain',            0.8)),
                'stanley_k':       float(sc.get('k',               1.0)),
                'stanley_ks':      float(sc.get('ks',              1.5)),
                'vel_scale':       float(sc.get('vel_scale',       1.0)),
                'obstacle_decel':  float(sc.get('obstacle_decel',  0.85)),
                'obs2lane_dist':   float(sc.get('obs2lane_dist',   0.25)),
                'interp_scale':    int(  sc.get('interp_scale',    20)),
                'lookahead_min':   float(sc.get('lookahead_min',   0.5)),
                'lookahead_max':   float(sc.get('lookahead_max',   2.0)),
                'lookahead_gain':  float(sc.get('lookahead_gain',  7.0)),
                'ff_gain':         float(sc.get('ff_gain',         1.0)),
                'ff_preview_sec':  float(sc.get('ff_preview_sec',  0.10)),
                'steer_lpf':       float(sc.get('steer_lpf',       1.0)),
                'map_yaml':        map_yaml,
                'bound_dir':       bound_dir,
            }])

    # ── mission_manager (start_gate=True 일 때만) ─────────────────────────
    m = cfg.get('mission', {})
    start_gate = bool(m.get('start_gate', False))

    mission_node = Node(
        package='race_stack', executable='mission_manager',
        name='mission_manager', output='screen',
        parameters=[{
            'laps':          int(  m.get('laps',          0)),
            'time_limit_sec': int( m.get('time_limit_sec', 0)),
            'start_gate':    start_gate,
            'start_button':  int(  m.get('start_button',  1)),
            'start_key':     bool( m.get('start_key',     True)),
            'start_x':       float(m.get('start_x',       0.0)),
            'start_y':       float(m.get('start_y',       0.0)),
            'start_radius':  float(m.get('start_radius',  1.5)),
        }]) if start_gate else None

    nodes = [
        physics_node, rviz, rsp, map_server, lifecycle,
        sim_bridge,
        TimerAction(period=3.0, actions=[ctrl_node]),
    ]
    if mission_node:
        nodes.append(mission_node)

    return LaunchDescription(nodes)
