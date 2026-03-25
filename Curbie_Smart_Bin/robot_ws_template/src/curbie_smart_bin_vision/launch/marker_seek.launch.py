from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='curbie_smart_bin_vision',
            executable='marker_seek_node',
            name='curbie_smart_bin_vision',
            output='screen',
            parameters=[{
                'rear_image_topic': '/curbie/rear_camera/image_raw',
                'rear_camera_info_topic': '/curbie/rear_camera/camera_info',
                'pickup_camera_name': 'rear',
                'dropoff_camera_name': 'front',
                'manual_camera_name': 'rear',
                'manual_use_rear_when_hooked': False,
                'obstacle_avoidance_enabled': False,
                'stop_distance_m': 0.15,
                # Tuned to avoid left/right dithering and keep advancing.
                'spin_speed_rad_s': 0.18,
                'approach_turn_cap_rad_s': 0.12,
                'yaw_kp': 0.45,
                'approach_speed_cap_m_s': 0.07,
                'distance_kp': 0.40,
                # Accept success slightly before exact stop distance.
                'stop_distance_tolerance_m': 0.03,
                'lost_marker_timeout_s': 2.0,
                'measurement_retain_s': 3.0,
                'align_lost_scan_timeout_s': 3.0,
                'align_max_lost_retries': 10,
                'align_search_spin_speed_rad_s': 0.20,
                'align_distance_tolerance_m': 0.05,
                'align_lateral_tolerance_m': 0.05,
                'align_yaw_tolerance_rad': 0.22,
                'align_success_hold_steps': 4,
                'align_max_angular_speed_rad_s': 0.30,
                'align_use_marker_yaw': False,
                'pickup_never_give_up': True,
                'heading_align_threshold_rad': 0.14,
                'heading_stop_yaw_rad': 0.50,
                'min_linear_scale': 0.35,
                'angular_filter_alpha': 0.20,
                'yaw_filter_alpha': 0.18,
                'yaw_deadband_rad': 0.12,
                # Camera x axis is right-positive; yaw command must be opposite.
                'marker_x_sign': -1.0,
                # Start paused; begin behavior via /curbie/seek/start service.
                'autostart': False,
                # Smooth marker seek/align velocity output.
                'cmd_smoothing_enabled': True,
                'cmd_linear_accel_limit_m_s2': 0.18,
                'cmd_linear_decel_limit_m_s2': 0.70,
                'cmd_angular_accel_limit_rad_s2': 0.80,
                'cmd_angular_decel_limit_rad_s2': 2.80,
            }],
        ),
    ])
