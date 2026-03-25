from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='curbie_smart_bin_operator',
            executable='operator_ui',
            name='curbie_smart_bin_operator_ui',
            output='screen',
        ),
    ])
