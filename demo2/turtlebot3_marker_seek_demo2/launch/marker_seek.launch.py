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
                # Tuned to avoid left/right dithering and keep advancing.
                'spin_speed_rad_s': 0.18,
                'approach_turn_cap_rad_s': 0.12,
                'yaw_kp': 0.45,
                'approach_speed_cap_m_s': 0.08,
                'distance_kp': 0.45,
                'lost_marker_timeout_s': 2.0,
                'heading_align_threshold_rad': 0.14,
                'heading_stop_yaw_rad': 0.50,
                'min_linear_scale': 0.35,
                'angular_filter_alpha': 0.20,
                'yaw_filter_alpha': 0.18,
                'yaw_deadband_rad': 0.12,
                # Camera x axis is right-positive; yaw command must be opposite.
                'marker_x_sign': -1.0,
                # Deploy hook automatically when marker seek succeeds.
                'hook_auto_deploy': True,
                'hook_servo_enable': True,
                'hook_servo_topic': 'servo_cmd',
                'hook_servo_rate_hz': 20.0,
                'hook_deploy_left': 0.7,
                'hook_deploy_right': 0.0,
                'hook_stow_left': 0.0,
                'hook_stow_right': 0.0,
                'hook_stow_on_start': True,
                'hook_stow_on_stop': False,
                # OpenCR typically uses /dev/ttyACM0, Pico is usually /dev/ttyACM1.
                'pico_serial_enable': True,
                'pico_serial_port': '/dev/serial/by-id/usb-MicroPython_Board_in_FS_mode_e663682593756333-if00',
                'pico_serial_baud': 115200,
                'pico_serial_timeout_s': 0.40,
                'pico_check_on_start': False,
                'pico_lock_command': 'LOCK',
                'pico_unlock_command': 'UNLOCK',
                'autostart': True,
            }],
        ),
    ])
