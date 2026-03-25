from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='curbie_smart_bin_operator',
            executable='combined_teleop_node',
            name='combined_teleop_node',
            output='screen',
            prefix='xterm -e',
        ),
    ])
