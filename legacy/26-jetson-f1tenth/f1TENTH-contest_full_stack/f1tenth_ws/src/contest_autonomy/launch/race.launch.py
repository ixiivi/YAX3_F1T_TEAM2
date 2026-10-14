import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('contest_autonomy')
    stack = get_package_share_directory('f1tenth_stack')
    map_arg = LaunchConfiguration('map')
    wp_arg = LaunchConfiguration('waypoints')
    x = LaunchConfiguration('x'); y = LaunchConfiguration('y'); yaw = LaunchConfiguration('yaw')
    params = os.path.join(pkg, 'config', 'race.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('map'), DeclareLaunchArgument('waypoints'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(stack, 'launch', 'bringup_launch.py'))),
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             parameters=[{'yaml_filename': map_arg}]),
        Node(package='nav2_amcl', executable='amcl', name='amcl',
             parameters=[os.path.join(pkg, 'config', 'amcl.yaml'), {
                 'initial_pose.x': x, 'initial_pose.y': y, 'initial_pose.yaw': yaw}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='localization_lifecycle', parameters=[{
                 'autostart': True, 'node_names': ['map_server', 'amcl']}]),
        Node(package='contest_autonomy', executable='mission_guard',
             name='mission_guard', output='screen', parameters=[params]),
        TimerAction(period=5.0, actions=[Node(
            package='contest_autonomy', executable='hybrid_driver',
            name='hybrid_driver', output='screen', parameters=[params,
            {'waypoint_file': wp_arg}])]),
    ])
