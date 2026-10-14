"""steer_trim.launch.py — 조향 트림 자동 캘리브레이션 브링업.

curvature_probe.launch.py 와 같은 패턴: 차량 브링업(f1tenth_stack)만 띄운다.
steer_trim 노드는 키보드 raw 입력이 필요해 race.sh 가 포그라운드에서
`ros2 run` 으로 직접 띄운다.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    f1tenth_share = get_package_share_directory('f1tenth_stack')
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(f1tenth_share, 'launch', 'bringup_launch.py')))
    return LaunchDescription([bringup])
