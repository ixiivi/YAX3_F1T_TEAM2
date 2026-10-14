import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    stack = get_package_share_directory('f1tenth_stack')
    pkg = get_package_share_directory('contest_autonomy')
    slam_cfg = os.path.join(stack, 'config', 'f1tenth_online_async.yaml')
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(stack, 'launch', 'bringup_launch.py')),
            launch_arguments={'joy_config': os.path.join(
                pkg, 'config', 'mapping_joy.yaml')}.items()),
        Node(package='slam_toolbox', executable='async_slam_toolbox_node',
             name='slam_toolbox', output='screen', parameters=[slam_cfg]),
    ])
