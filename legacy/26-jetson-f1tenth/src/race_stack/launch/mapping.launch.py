"""mapping.launch.py — SLAM 맵 생성 모드.

f1tenth_stack 브링업(센서+VESC+mux) + slam_toolbox(online async, mapping).
키보드 텔레옵은 tty가 필요해 race.sh가 포그라운드로 따로 띄운다.

★ imu_odom 도 함께 띄운다. vesc.yaml 에서 vesc_to_odom_node.publish_tf 를 껐기
  때문에 odom→base_link TF 를 발행하는 것이 이 노드뿐이고, slam_toolbox 는 odom
  TF 가 없으면 맵을 만들지 못한다. 맵 품질 자체도 자이로 yaw 쪽이 낫다.
"""
import os

import yaml
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    f1tenth_share = get_package_share_directory('f1tenth_stack')

    # SLAM 파라미터는 race_stack 소유 사본을 쓴다.
    #   f1tenth_stack/config/f1tenth_online_async.yaml 은 업스트림 패키지 소유라
    #   우리가 고치면 패키지 갱신 때 덮이고, 두 차가 공유해 실험도 못 한다.
    #   race_stack/config/slam_mapping.yaml 로 복사해 거기서 튜닝한다.
    #   ★ 되돌리려면 아래 fallback 경로를 그대로 쓰면 된다 (파일만 지우면 자동).
    race_share = get_package_share_directory('race_stack')
    slam_params = os.path.join(race_share, 'config', 'slam_mapping.yaml')
    if not os.path.exists(slam_params):
        slam_params = os.path.join(f1tenth_share, 'config',
                                   'f1tenth_online_async.yaml')
    print(f'[mapping.launch] SLAM 파라미터: {slam_params}')

    # race.launch.py 와 같은 경로 규약 (race.sh 가 RACE_SCRIPT 를 export 한다)
    cfg_path = os.environ.get(
        'RACE_SCRIPT', os.path.expanduser('~/race_script.yaml'))
    try:
        with open(cfg_path) as f:
            cfg = yaml.safe_load(f) or {}
    except OSError:
        cfg = {}

    # ★ 차량별 덮어쓰기 — race.launch.py 와 **같은 규약**이어야 한다.
    #   이게 없으면 vehicle_car*.yaml 의 localization 값(speed_sign 등)이
    #   매핑에서만 통째로 무시된다. race_script.yaml 에 그 키가 아예 없으면
    #   조용히 기본값이 들어가서, 값을 고쳐도 "안 먹는다" 로만 보인다
    #   (2026-08-10: speed_sign -1 을 넣었는데 매핑에서 계속 +1 이었다).
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
        print(f'[mapping.launch] 차량 덮어쓰기 적용: {_ovl_path}')

    loc = cfg.get('localization', {}) or {}

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(f1tenth_share, 'launch', 'bringup_launch.py')))

    imu_odom = Node(
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
        }])

    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params])

    return LaunchDescription([bringup, imu_odom, slam])
