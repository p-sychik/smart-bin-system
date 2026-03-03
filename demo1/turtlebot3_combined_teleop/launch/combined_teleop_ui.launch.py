from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlebot3_combined_teleop',
            executable='combined_teleop_ui_node',
            name='combined_teleop_ui_node',
            output='screen',
        ),
    ])
