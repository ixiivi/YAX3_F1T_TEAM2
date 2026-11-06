"""Gamepad only: F710 X mode reader and joy_teleop, for starting them apart from bringup (use_joy:=false)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('f1tenth_stack'), 'config', 'joy_teleop.yaml')
    return LaunchDescription([
        Node(package='f1tenth_stack', executable='f710_xinput', name='joy', parameters=[config]),
        Node(package='joy_teleop', executable='joy_teleop', name='joy_teleop', parameters=[config]),
    ])
