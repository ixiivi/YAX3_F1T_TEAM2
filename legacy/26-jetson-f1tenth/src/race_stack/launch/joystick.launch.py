"""joystick.launch.py — 조이스틱 수동 조종 + 차량 브링업.

f1tenth_stack bringup (joy 드라이버 포함) + joystick_teleop 을 한 번에 기동.
race_script.yaml의 joystick 섹션 파라미터 사용.

조작법 (Logitech F710, Mode 버튼 OFF):
  LB              → 수동 ↔ 자율 모드 토글
  RT 당김          → 전진 (당긴 만큼 선형 비례, 놓으면 즉시 정지)
  RB 누른 채 RT    → 후진
  L스틱 좌/우      → 조향

★ joy 드라이버는 bringup_launch.py 가 이미 띄운다.
  여기서 joy_node 를 또 띄우면 /joy 퍼블리셔가 2개가 되어 deadzone 설정이
  뒤섞이므로, 대신 bringup 의 joy_config 인자를 덮어써서 설정만 주입한다.
"""
import os
import re
import tempfile
import yaml
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _write_joy_config(joy_cfg: dict) -> str:
    """bringup 의 joy / joy_teleop 노드에 넘길 파라미터 파일을 생성."""
    dev = str(joy_cfg.get('device', '/dev/input/js0'))
    m = re.search(r'(\d+)\s*$', dev)
    device_id = int(m.group(1)) if m else 0

    params = {
        # joy 드라이버: deadzone 0 → 트리거 응답을 왜곡 없이 그대로 전달.
        # (데드존은 joystick_teleop 이 정규화된 0~1 스로틀 기준으로 처리한다.
        #  joy 의 deadzone 은 축의 '중앙'을 기준으로 잡히는데, 트리거는 중앙이
        #  절반쯤 당긴 지점이라 그대로 두면 스로틀 중간에 평평한 구간이 생긴다.)
        # autorepeat_rate 20Hz → RT를 고정한 채 있어도 /joy 가 계속 발행되어
        #  ackermann_mux 의 timeout(0.2s)에 걸려 차가 끊기지 않는다.
        'joy': {'ros__parameters': {
            'device_id':       device_id,
            'deadzone':        0.0,
            'autorepeat_rate': 20.0,
        }},
        # f1tenth_stack 기본 joy_teleop 도 /teleop 에 발행한다.
        # 원래 데드맨이 LB(4)인데 race_stack 은 LB 를 모드 토글로 쓰므로 충돌한다.
        # 데드맨을 RS(10)로 옮겨 충돌만 없애고, 수동 오버라이드 기능은 남겨둔다.
        'joy_teleop': {'ros__parameters': {
            'human_control': {
                'type':            'topic',
                'interface_type':  'ackermann_msgs/msg/AckermannDriveStamped',
                'topic_name':      'teleop',
                'deadman_buttons': [10],
                'axis_mappings': {
                    'drive-speed':          {'axis': 1, 'scale': 1.0,  'offset': 0.0},
                    'drive-steering_angle': {'axis': 3, 'scale': 0.34, 'offset': 0.0},
                },
            },
        }},
    }

    path = os.path.join(tempfile.gettempdir(), 'race_joy_params.yaml')
    with open(path, 'w') as f:
        yaml.safe_dump(params, f)
    return path


def generate_launch_description():
    # quiet 면 화면 출력을 로그 파일로 돌린다. slip.launch.py 가 이걸 쓴다 —
    # 슬립 리그는 전체 화면 패널을 그리므로 다른 노드가 stdout 에 끼어들면 깨진다.
    # ★ RACE_QUIET(환경변수)를 먼저 본다. include 로 불릴 때는 sys.argv 에
    #   quiet:=true 가 들어오지 않아 argv 검사만으로는 절대 참이 되지 않았다.
    import sys
    quiet = (os.environ.get('RACE_QUIET', '') == '1'
             or any(a.lower() in ('quiet:=true', 'quiet:=1') for a in sys.argv))
    out = 'log' if quiet else 'screen'

    cfg_path = os.environ.get(
        'RACE_SCRIPT', os.path.expanduser('~/race_script.yaml'))
    with open(cfg_path) as f:
        cfg = yaml.safe_load(f)

    # ★ 차량별 덮어쓰기 — race.launch.py 와 **같은 규약**이어야 한다.
    #   joystick 절은 차량 파일이 소유하는데(steer_invert/축·버튼/max_speed),
    #   여기서 안 얹으면 ./race.sh joy 만 race_script.yaml 의 공용값으로 돈다.
    #   두 차의 조이스틱 설정이 다르면 조용히 남의 차 값으로 주행하게 된다
    #   (2026-08-10 발견: max_speed 를 차량 파일에서 낮췄는데 joy 에 안 먹었다).
    _ovl_path = os.environ.get('RACE_VEHICLE_OVERLAY', '')
    if _ovl_path and os.path.exists(_ovl_path):
        def _deep(dst, src):
            for k, v in src.items():
                if isinstance(v, dict) and isinstance(dst.get(k), dict):
                    _deep(dst[k], v)
                else:
                    dst[k] = v
        with open(_ovl_path) as f:
            _deep(cfg, yaml.safe_load(f) or {})
        if not quiet:
            print(f'[joystick.launch] 차량 덮어쓰기 적용: {_ovl_path}')

    joy_cfg = cfg.get('joystick', {})
    speed   = cfg.get('speed', {})

    f1tenth_share = get_package_share_directory('f1tenth_stack')
    joy_config    = _write_joy_config(joy_cfg)

    nodes = []

    # 차량 브링업 (LiDAR + VESC + odom + mux + joy 드라이버)
    nodes.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(f1tenth_share, 'launch', 'bringup_launch.py')),
        launch_arguments={'joy_config': joy_config}.items()))

    # joystick_teleop (3초 지연: 브링업 후 mux 준비 시간 확보)
    nodes.append(TimerAction(period=3.0, actions=[Node(
        package='race_stack', executable='joystick_teleop',
        name='joystick_teleop', output=out,
        parameters=[{
            'axis_steer':       int(  joy_cfg.get('axis_steer',       0)),
            'axis_speed':       int(  joy_cfg.get('axis_speed',       5)),
            'btn_reverse':      int(  joy_cfg.get('btn_reverse',      5)),
            'btn_toggle':       int(  joy_cfg.get('btn_toggle',       4)),
            # ★ 수동 전용 모드다. 자율로 넘길 대상(/drive 를 내는 컨트롤러)이
            #   애초에 없으므로 LB 토글을 끈다. 예전엔 토글 로그만 뜨고 차는
            #   가만히 있어서 조용히 실패했다 (2026-08-04).
            'allow_toggle':     False,
            'start_auto':       False,
            'steer_invert':     bool( joy_cfg.get('steer_invert',     False)),
            'speed_invert':     bool( joy_cfg.get('speed_invert',     False)),
            'trigger_invert':   bool( joy_cfg.get('trigger_invert',   True)),
            'trigger_deadzone': float(joy_cfg.get('trigger_deadzone', 0.05)),
            'trigger_expo':     float(joy_cfg.get('trigger_expo',     1.0)),
            'steer_deadzone':   float(joy_cfg.get('steer_deadzone',   0.08)),
            'arm_on_neutral':   bool( joy_cfg.get('arm_on_neutral',   True)),
            'debug':            bool( joy_cfg.get('debug',            False)),
            # ./race.sh joy 전용 최고속도. 없으면 mapping_max 로 폴백.
            # (맵핑은 천천히 돌아야 품질이 좋아 값을 분리해 둔다. mapping 과
            #  mapping-joy 는 race.sh 가 speed.mapping_max 를 직접 읽으므로
            #  이 값에 영향받지 않는다)
            # ★ 슬립 리그(./race.sh slip)는 자기 상한을 환경변수로 주입한다.
            #   슬립을 내려면 자율주행보다 높은 속도가 필요한데, 예전엔 이 값을
            #   공유해서 리그 상한을 올리면 자율주행 상한까지 같이 올라갔다.
            'max_speed':        float(os.environ.get(
                'RACE_SLIP_MAX_SPEED',
                joy_cfg.get('max_speed', speed.get('mapping_max', 1.0)))),
            'max_steer':        float(os.environ.get(
                'RACE_SLIP_MAX_STEER',
                cfg.get('pure_pursuit', {}).get('max_steer', 0.36))),
            # 디지털 조향 — Voser 원형 주행용. slip.launch.py 가 env 로 켠다.
            'steer_discrete':   os.environ.get('RACE_SLIP_STEER_DISCRETE', '') == '1',
        }])]))

    return LaunchDescription(nodes)
