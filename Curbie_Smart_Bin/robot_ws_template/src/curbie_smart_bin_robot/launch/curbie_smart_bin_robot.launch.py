from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


# Functional roles are intentionally swapped here:
# the physical rear camera now publishes as the functional front camera,
# and the physical front camera now publishes as the functional rear camera.
FRONT_CAMERA_DEVICE = '/dev/v4l/by-id/usb-046d_C270_HD_WEBCAM_200901010001-video-index0'
REAR_CAMERA_DEVICE = '/dev/v4l/by-id/usb-046d_0825_B10563F0-video-index0'


def generate_launch_description():
    turtlebot_launch = os.path.join(
        get_package_share_directory('turtlebot3_bringup'),
        'launch',
        'robot.launch.py',
    )
    servo_launch = os.path.join(
        get_package_share_directory('turtlebot3_pico_servo'),
        'launch',
        'servo_driver_launch.py',
    )
    marker_seek_launch = os.path.join(
        get_package_share_directory('curbie_smart_bin_vision'),
        'launch',
        'marker_seek.launch.py',
    )

    turtlebot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(turtlebot_launch)
    )
    servo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(servo_launch)
    )
    marker_seek = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(marker_seek_launch)
    )

    front_camera = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='curbie_front_camera',
        parameters=[{
            'video_device': FRONT_CAMERA_DEVICE,
            'image_size': [320, 240],
            'pixel_format': 'YUYV',
            'output_encoding': 'mono8',
            # Keep camera streams non-blocking on weak links.
            'qos_overrides./image_raw.publisher.reliability': 'best_effort',
            'qos_overrides./image_raw.publisher.history': 'keep_last',
            'qos_overrides./image_raw.publisher.depth': 1,
            'qos_overrides./camera_info.publisher.reliability': 'best_effort',
            'qos_overrides./camera_info.publisher.history': 'keep_last',
            'qos_overrides./camera_info.publisher.depth': 1,
        }],
        remappings=[
            ('image_raw', '/curbie/front_camera/image_raw'),
            ('camera_info', '/curbie/front_camera/camera_info'),
        ],
        output='screen',
    )

    rear_camera = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='curbie_rear_camera',
        parameters=[{
            'video_device': REAR_CAMERA_DEVICE,
            'image_size': [320, 240],
            'pixel_format': 'YUYV',
            'output_encoding': 'mono8',
            # Keep camera streams non-blocking on weak links.
            'qos_overrides./image_raw.publisher.reliability': 'best_effort',
            'qos_overrides./image_raw.publisher.history': 'keep_last',
            'qos_overrides./image_raw.publisher.depth': 1,
            'qos_overrides./camera_info.publisher.reliability': 'best_effort',
            'qos_overrides./camera_info.publisher.history': 'keep_last',
            'qos_overrides./camera_info.publisher.depth': 1,
        }],
        remappings=[
            ('image_raw', '/curbie/rear_camera/image_raw'),
            ('camera_info', '/curbie/rear_camera/camera_info'),
        ],
        output='screen',
    )

    sonars = Node(
        package='curbie_smart_bin_robot',
        executable='sonar_publisher',
        name='curbie_sonar_publisher',
        output='screen',
    )

    return LaunchDescription([
        turtlebot,
        servo,
        marker_seek,
        front_camera,
        rear_camera,
        sonars,
    ])
