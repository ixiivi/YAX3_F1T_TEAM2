"""curvature_probe.launch.py — 곡률-속도 계단 캘리브레이션 브링업.

mapping.launch.py 와 같은 패턴: 여기선 차량 브링업(f1tenth_stack)만 백그라운드로
띄운다. curvature_probe 노드는 키보드 raw 입력(tty)이 필요해 race.sh 가
포그라운드에서 `ros2 run` 으로 직접 띄운다 (ros2 launch 로 띄우면 stdin이
launch 프로세스로 가서 키 입력을 못 받는다).

slip.launch.py/slip_logger.py 는 건드리지 않는다 — 완전히 별개 도구.
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
