from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlebot3_marker_seek_demo2',
            executable='marker_seek_node',
            name='marker_seek_node',
            output='screen',
            parameters=[{
                'spin_speed_rad_s': 0.18,
                'approach_turn_cap_rad_s': 0.12,
                'yaw_kp': 0.45,
                'approach_speed_cap_m_s': 0.08,
                'distance_kp': 0.45,
                'stop_distance_tolerance_m': 0.03,
                'lost_marker_timeout_s': 2.0,
                'heading_align_threshold_rad': 0.14,
                'heading_stop_yaw_rad': 0.50,
                'min_linear_scale': 0.35,
                'angular_filter_alpha': 0.20,
                'yaw_filter_alpha': 0.18,
                'yaw_deadband_rad': 0.12,
                'marker_x_sign': -1.0,
                'autostart': False,
                'obstacle_avoidance_enabled': True,
                'obstacle_threshold_cm': 30.0,
                'obstacle_trigger_count': 3,
                'obstacle_clear_count': 3,
                'sonar_pins': [5, 16, 18, 22, 24],
            }],
        ),
    ])
