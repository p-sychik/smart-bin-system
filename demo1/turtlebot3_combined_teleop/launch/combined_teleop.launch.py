from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlebot3_combined_teleop',
            executable='combined_teleop_node',
            name='combined_teleop_node',
            output='screen',
            prefix='xterm -e',
        ),
    ])
