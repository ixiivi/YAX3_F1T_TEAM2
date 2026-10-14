"""slip.launch.py — 조이스틱 조종 + 슬립 테스트 리그를 한 번에.

`./race.sh slip` 하나로 끝난다. joystick.launch.py 를 quiet 모드로 포함해
차량 브링업과 조종을 올리고, 그 위에 slip_logger 를 얹는다.

★ 왜 quiet 인가
  slip_logger 는 매 0.3초 전체 화면을 지우고 패널을 다시 그린다. 같은 터미널에
  다른 노드가 stdout 으로 끼어들면 패널이 깨진다. 그래서 joystick_teleop 은
  로그 파일로 돌리고, 대신 그 상태(트리거 중립 대기 등)를 패널 안에 띄운다.
  브링업 로그는 기동 직후에만 나오고 이후 조용하다.
"""
import os
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import yaml


def generate_launch_description():
    share = get_package_share_directory('race_stack')
    cfg_path = os.environ.get('RACE_SCRIPT', os.path.expanduser('~/race_script.yaml'))
    with open(cfg_path) as f:
        cfg = yaml.safe_load(f)

    # ★ 차량별 값 덮어쓰기 — race.launch.py 와 같은 규약.
    #   여기서 안 얹으면 pure_pursuit.wheelbase 를 공용값으로 읽어, 휠베이스가
    #   다른 차에서 반경↔조향각 환산이 틀어진다 (2026-08-10 점검에서 발견).
    _ovl = os.environ.get('RACE_VEHICLE_OVERLAY', '')
    if _ovl and os.path.exists(_ovl):
        def _deep(dst, src):
            for k, v in src.items():
                if isinstance(v, dict) and isinstance(dst.get(k), dict):
                    _deep(dst[k], v)
                else:
                    dst[k] = v
        with open(_ovl) as f:
            _deep(cfg, yaml.safe_load(f) or {})

    sl = cfg.get('slip', {})
    vesc = os.path.join(os.path.dirname(os.path.dirname(share)), 'config', 'vesc.yaml')

    # 서보 캘리브레이션은 config/vesc.yaml 이 원본이다. 여기서 다시 적지 않는다 —
    # 두 곳에 적으면 반드시 갈라진다.
    gain, offset = -0.761, 0.425
    s_min, s_max = 0.15, 0.85       # 기구 스토퍼 — 이것도 vesc.yaml 이 원본이다
    for p in (os.path.join(os.path.expanduser(os.environ.get('RACE_WS', '~/yax')),
                           'config', 'vesc.yaml'), vesc):
        try:
            v = yaml.safe_load(open(p))['/**']['ros__parameters']
            gain = float(v['steering_angle_to_servo_gain'])
            offset = float(v['steering_angle_to_servo_offset'])
            s_min = float(v.get('servo_min', s_min))
            s_max = float(v.get('servo_max', s_max))
            break
        except Exception:
            continue

    # ★ 슬립 리그는 조향·속도 상한을 **자기 것**으로 갖는다 (2026-08-06).
    #   예전엔 joystick.max_speed 와 pure_pursuit.max_steer 를 그대로 물려받아서,
    #   슬립을 내려고 상한을 올리면 자율주행 컨트롤러의 상한까지 같이 올라갔다.
    #   게다가 joystick_teleop 은 자기 상한을 넘는 slip 요청값을 **조용히 무시**한다
    #   (speeds 의 3.0 > max_speed 2.5 → 무시되고 2.5 로 주행) — 그래서 격자 칸이
    #   영원히 안 채워지고 "속도가 안 올라간다" 로 보였다.
    #
    #   ★ 전달은 환경변수로 한다. launch_arguments 로 넘기면 joystick.launch.py
    #     에는 닿지 않는다 — 거기서는 sys.argv 를 보는데, include 는 argv 를 거치지
    #     않기 때문이다. (그래서 quiet:=true 도 여태 동작하지 않고 있었다.
    #     slip 패널이 다른 노드 출력에 깨지던 원인)
    #     include 는 같은 프로세스에서 늦게 평가되므로 os.environ 이 그대로 보인다.
    slip_mode = str(sl.get('mode', 'rig'))   # 기본: 기존 원-그리기 리그

    os.environ['RACE_QUIET'] = '1'
    if 'max_speed' in sl:
        os.environ['RACE_SLIP_MAX_SPEED'] = str(float(sl['max_speed']))
    if 'max_steer' in sl:
        os.environ['RACE_SLIP_MAX_STEER'] = str(float(sl['max_steer']))
    # ★ friction 모드는 조향을 정확히 고정해야 하므로 스틱을 디지털로 쓴다
    #   (직진 0 또는 선택각). 사람 손으로 비례 스틱을 일정하게 유지할 수 없고,
    #   조향이 흔들리면 준정상상태 표본이 통째로 날아간다.
    #   ※ include 를 만들기 **전에** 설정한다 — 다른 env 와 같은 자리.
    os.environ.pop('RACE_SLIP_STEER_DISCRETE', None)
    if slip_mode == 'friction':
        os.environ['RACE_SLIP_STEER_DISCRETE'] = '1'

    nodes = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'joystick.launch.py')))]

    # 브링업·teleop 이 자리를 잡은 뒤 패널을 띄운다 (그래야 첫 화면이 안 깨진다)
    wb = float(cfg.get('pure_pursuit', {}).get('wheelbase', 0.33))

    if slip_mode == 'friction':
        # ── LiDAR+IMU 마찰계수 추정기 (2509.15423) ─────────────────
        nodes.append(TimerAction(period=5.0, actions=[Node(
            package='race_stack', executable='slip_friction',
            name='slip_friction', output='screen',
            parameters=[{
                'servo_gain':    gain,
                'servo_offset':  offset,
                'servo_min':     s_min,
                'servo_max':     s_max,
                'wheelbase':     wb,
                # base_link→laser static TF (f1tenth_stack bringup: 0.27 0 0.11)
                'lidar_x':       float(sl.get('lidar_x', 0.27)),
                'enable_lin_slip': bool(sl.get('enable_lin_slip', False)),
                'bias_sec':      float(sl.get('bias_sec',   3.0)),
                'cal_sec':       float(sl.get('cal_sec',    5.0)),
                'delta_lin':     float(sl.get('delta_lin', -1.0)),
                'delta_ang':     float(sl.get('delta_ang', -1.0)),
                'deskew':        bool(sl.get('deskew', False)),
                'max_scan_m':    float(sl.get('max_scan_m', 10.0)),
                'icp_points':    int(sl.get('icp_points',   400)),
                'icp_hz':        float(sl.get('icp_hz',    10.0)),
                'lpf_hz':        float(sl.get('lpf_hz',     8.0)),
                'lidar_lpf_hz':  float(sl.get('lidar_lpf_hz', 2.0)),
                'min_speed':     float(sl.get('min_speed',  0.3)),
                # 줄자 실측(odom_scale 6회). vesc.yaml 의 speed_to_erpm_gain 을
                # 고치면 이 값을 1.0 으로 되돌릴 것 — 아니면 이중 보정된다.
                'speed_scale':   float(sl.get('speed_scale', 1.0)),
                # D-pad 로 고를 반경/속도. 값은 slip.max_steer / max_speed 를
                # 넘으면 조용히 무시되므로 그 안에 두어야 한다.
                'radii':         [float(x) for x in sl.get(
                                     'radii', [0.8, 1.0, 1.2, 1.5, 2.0])],
                'speeds':        [float(x) for x in sl.get(
                                     'speeds', [1.0, 1.5, 2.0, 2.5, 3.0, 3.5])],
                'dpad_x_invert': bool(sl.get('dpad_x_invert', False)),
                'dpad_y_invert': bool(sl.get('dpad_y_invert', False)),
            }])]))

        # ── 원시 토픽 기록 ────────────────────────────────────────────
        # ICP 튜닝(max_scan_m, icp_hz, 이상치 제거)은 CSV 로는 못 한다 —
        # CSV 에는 이미 추정이 끝난 결과만 남는다. 원시 /scan 을 남겨야
        # 젯슨에서 차를 굴리지 않고 노트북에서 재생하며 고칠 수 있다.
        if bool(sl.get('record_bag', True)):
            repo = os.path.expanduser(os.environ.get('RACE_REPO', '~/yax'))
            out  = os.path.join(repo, 'logs', 'slip_tests',
                                time.strftime('bag_%Y%m%d_%H%M%S'))
            nodes.append(ExecuteProcess(
                cmd=['ros2', 'bag', 'record', '-o', out,
                     '/scan', '/odom', '/sensors/imu/raw',
                     '/commands/servo/position'],
                output='log'))
    else:
        # ── 기존 원-그리기 슬립 리그 ────────────────────────────────
        nodes.append(TimerAction(period=5.0, actions=[Node(
            package='race_stack', executable='slip_logger',
            name='slip_logger', output='screen',
            parameters=[{
                'servo_gain':   gain,
                'servo_offset': offset,
                'wheelbase':    wb,
                'radii':        [float(x) for x in sl.get('radii', [1.0, 1.5, 2.0, 3.0])],
                'speeds':       [float(x) for x in sl.get('speeds',
                                                          [1.0, 1.5, 2.0, 2.5, 3.0])],
                'target_runs':  int(sl.get('target_runs', 3)),
                'lever_x':      float(sl.get('lever_x', 0.0)),
                'mu':           float(sl.get('mu', 0.7)),
                'dpad_x_invert': bool(sl.get('dpad_x_invert', False)),
                'dpad_y_invert': bool(sl.get('dpad_y_invert', False)),
            }])]))
    return LaunchDescription(nodes)
