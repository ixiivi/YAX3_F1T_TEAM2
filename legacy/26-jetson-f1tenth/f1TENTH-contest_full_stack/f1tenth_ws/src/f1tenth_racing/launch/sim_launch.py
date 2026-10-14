
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='f1tenth_racing',
            executable='pure_pursuit_node',
            name='pure_pursuit_node',
            output='screen',
            
        ),
    ])
