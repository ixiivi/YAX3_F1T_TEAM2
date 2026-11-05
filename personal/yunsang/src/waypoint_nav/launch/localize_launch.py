"""map_server + AMCL on a saved map. global_localize finds the start pose from the first
scan (car standing still); refine with RViz "2D Pose Estimate" if needed."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    amcl_yaml = os.path.join(get_package_share_directory('waypoint_nav'), 'config', 'amcl.yaml')
    map_arg = DeclareLaunchArgument(
        'map', description='map .yaml saved by map_saver_cli (absolute path)')
    # start pose in the map frame; refine it with RViz "2D Pose Estimate"
    init_args = [DeclareLaunchArgument(n, default_value='0.0') for n in ('x', 'y', 'yaw')]
    initial_pose = {'initial_pose': {
        n: ParameterValue(LaunchConfiguration(n), value_type=float) for n in ('x', 'y', 'yaw')}}
    auto_arg = DeclareLaunchArgument(
        'auto', default_value='true', description='find the start pose by scan matching')
    return LaunchDescription([
        map_arg, auto_arg, *init_args,
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen',
             parameters=[{'yaml_filename': LaunchConfiguration('map')}]),
        Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen',
             parameters=[amcl_yaml, initial_pose]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization', output='screen',
             parameters=[{'autostart': True, 'node_names': ['map_server', 'amcl']}]),
        Node(package='waypoint_nav', executable='global_localize', name='global_localize',
             output='screen', condition=IfCondition(LaunchConfiguration('auto')),
             parameters=[{'map': LaunchConfiguration('map')}]),
    ])
