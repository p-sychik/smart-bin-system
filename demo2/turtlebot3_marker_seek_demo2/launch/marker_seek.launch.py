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
                'rear_image_topic': '/rear_camera/image_raw',
                'rear_camera_info_topic': '/rear_camera/camera_info',
                'pickup_camera_name': 'front',
                'dropoff_camera_name': 'rear',
                'manual_camera_name': 'front',
                'manual_use_rear_when_hooked': True,
                # Tuned to avoid left/right dithering and keep advancing.
                'spin_speed_rad_s': 0.18,
                'approach_turn_cap_rad_s': 0.12,
                'yaw_kp': 0.45,
                'approach_speed_cap_m_s': 0.08,
                'distance_kp': 0.45,
                # Accept success slightly before exact stop distance.
                'stop_distance_tolerance_m': 0.03,
                'lost_marker_timeout_s': 2.0,
                'heading_align_threshold_rad': 0.14,
                'heading_stop_yaw_rad': 0.50,
                'min_linear_scale': 0.35,
                'angular_filter_alpha': 0.20,
                'yaw_filter_alpha': 0.18,
                'yaw_deadband_rad': 0.12,
                # Camera x axis is right-positive; yaw command must be opposite.
                'marker_x_sign': -1.0,
                # Start paused; begin behavior via /marker_seek/start service.
                'autostart': False,
            }],
        ),
    ])
