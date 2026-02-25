from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlebot3_marker_seek_demo2',
            executable='marker_seek_node',
            name='marker_seek_node',
            output='screen',
        ),
    ])
