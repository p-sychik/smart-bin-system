"""ROS 2 node wrapping marker seek plus target-aware ArUco alignment."""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.action import ActionServer
from rclpy.action import CancelResponse
from rclpy.action import GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

from bin_collection_msgs.action import AlignToMarker
from bin_collection_msgs.msg import AlignmentStatus
from bin_collection_msgs.msg import HookStatus
from turtlebot3_marker_seek_demo2.align_to_marker_core import AlignMeasurement
from turtlebot3_marker_seek_demo2.align_to_marker_core import AlignState
from turtlebot3_marker_seek_demo2.align_to_marker_core import AlignToMarkerCore
from turtlebot3_marker_seek_demo2.align_to_marker_core import select_target_measurement
from turtlebot3_marker_seek_demo2.camera_routing import active_camera_name
from turtlebot3_marker_seek_demo2.camera_routing import camera_for_align_mode
from turtlebot3_marker_seek_demo2.camera_routing import normalize_camera_name
from turtlebot3_marker_seek_demo2.marker_seek_core import MarkerSeekCore
from turtlebot3_marker_seek_demo2.marker_seek_core import SeekState

try:
    import cv2
except ImportError as exc:
    cv2 = None
    IMPORT_ERROR = exc
else:
    IMPORT_ERROR = None

try:
    from grove.grove_ultrasonic_ranger import GroveUltrasonicRanger
    _SONAR_AVAILABLE = True
except ImportError:
    _SONAR_AVAILABLE = False


@dataclass
class CameraStreamState:
    """Per-camera calibration and filtering state."""

    name: str
    image_topic: str
    camera_info_topic: str
    camera_matrix: Optional[np.ndarray] = None
    dist_coeffs: Optional[np.ndarray] = None
    camera_info_logged: bool = False
    empty_distortion_warned: bool = False
    intrinsics_fallback_warned: bool = False
    filtered_yaw_rad: Optional[float] = None


class MarkerSeekNode(Node):
    """Autonomous marker search node plus mission alignment action server."""

    _DEFAULT_SONAR_PINS: List[int] = [5, 16, 18, 22, 24]

    def __init__(self) -> None:
        super().__init__('marker_seek_node')
        self._callback_group = ReentrantCallbackGroup()

        if cv2 is None:
            raise RuntimeError(
                f'OpenCV import failed: {IMPORT_ERROR}. Install python3-opencv first.'
            )
        if not hasattr(cv2, 'aruco'):
            raise RuntimeError('OpenCV ArUco module is unavailable.')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera_info')
        self.declare_parameter('rear_image_topic', '/rear_camera/image_raw')
        self.declare_parameter('rear_camera_info_topic', '/rear_camera/camera_info')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('pickup_camera_name', 'front')
        self.declare_parameter('dropoff_camera_name', 'rear')
        self.declare_parameter('manual_camera_name', 'front')
        self.declare_parameter('manual_use_rear_when_hooked', True)
        self.declare_parameter('hook_status_topic', '/hook/status')
        self.declare_parameter('aruco_dictionary', 'DICT_4X4_50')
        self.declare_parameter('marker_size_m', 0.10)
        self.declare_parameter('stop_distance_m', 0.25)
        self.declare_parameter('stop_distance_tolerance_m', 0.03)
        self.declare_parameter('spin_speed_rad_s', 0.60)
        self.declare_parameter('approach_turn_cap_rad_s', 0.0)
        self.declare_parameter('search_timeout_s', 12.0)
        self.declare_parameter('max_retries', 3)
        self.declare_parameter('retry_nudge_speed_m_s', 0.10)
        self.declare_parameter('retry_nudge_duration_s', 1.5)
        self.declare_parameter('approach_speed_cap_m_s', 0.10)
        self.declare_parameter('yaw_kp', 1.5)
        self.declare_parameter('distance_kp', 0.8)
        self.declare_parameter('lost_marker_timeout_s', 0.6)
        self.declare_parameter('min_linear_scale', 0.0)
        self.declare_parameter('angular_filter_alpha', 1.0)
        self.declare_parameter('heading_align_threshold_rad', 0.05)
        self.declare_parameter('heading_stop_yaw_rad', 0.15)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('camera_horizontal_fov_rad', 1.10)
        self.declare_parameter('yaw_filter_alpha', 0.30)
        self.declare_parameter('yaw_deadband_rad', 0.05)
        self.declare_parameter('marker_x_sign', -1.0)
        self.declare_parameter('autostart', False)
        self.declare_parameter('obstacle_avoidance_enabled', True)
        self.declare_parameter('obstacle_threshold_cm', 30.0)
        self.declare_parameter('obstacle_trigger_count', 3)
        self.declare_parameter('obstacle_clear_count', 3)
        self.declare_parameter('obstacle_fail_timeout_s', 5.0)
        self.declare_parameter('sonar_pins', self._DEFAULT_SONAR_PINS)
        self.declare_parameter('align_search_spin_speed_rad_s', 0.45)
        self.declare_parameter('align_distance_tolerance_m', 0.04)
        self.declare_parameter('align_lateral_tolerance_m', 0.04)
        self.declare_parameter('align_yaw_tolerance_rad', 0.10)
        self.declare_parameter('align_success_hold_steps', 5)
        self.declare_parameter('align_lost_scan_timeout_s', 1.5)
        self.declare_parameter('align_max_lost_retries', 3)
        self.declare_parameter('align_max_reverse_speed_m_s', 0.05)
        self.declare_parameter('align_max_angular_speed_rad_s', 0.8)

        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.aruco_dictionary_name = str(self.get_parameter('aruco_dictionary').value)
        self.marker_size_m = float(self.get_parameter('marker_size_m').value)
        self.control_rate_hz = max(5.0, float(self.get_parameter('control_rate_hz').value))
        self.camera_horizontal_fov_rad = float(
            self.get_parameter('camera_horizontal_fov_rad').value
        )
        self.yaw_filter_alpha = float(self.get_parameter('yaw_filter_alpha').value)
        self.yaw_deadband_rad = float(self.get_parameter('yaw_deadband_rad').value)
        self.marker_x_sign = float(self.get_parameter('marker_x_sign').value)
        self.autostart = bool(self.get_parameter('autostart').value)
        self.manual_use_rear_when_hooked = bool(
            self.get_parameter('manual_use_rear_when_hooked').value
        )
        self.hook_status_topic = str(self.get_parameter('hook_status_topic').value)
        self.obstacle_fail_timeout_s = float(
            self.get_parameter('obstacle_fail_timeout_s').value
        )

        self._camera_streams = {
            'front': CameraStreamState(
                name='front',
                image_topic=str(self.get_parameter('image_topic').value),
                camera_info_topic=str(self.get_parameter('camera_info_topic').value),
            ),
            'rear': CameraStreamState(
                name='rear',
                image_topic=str(self.get_parameter('rear_image_topic').value),
                camera_info_topic=str(
                    self.get_parameter('rear_camera_info_topic').value
                ),
            ),
        }
        self.pickup_camera_name = self._resolve_camera_name(
            str(self.get_parameter('pickup_camera_name').value),
            default='front',
            parameter_name='pickup_camera_name',
        )
        self.dropoff_camera_name = self._resolve_camera_name(
            str(self.get_parameter('dropoff_camera_name').value),
            default='rear',
            parameter_name='dropoff_camera_name',
        )
        self.manual_camera_name = self._resolve_camera_name(
            str(self.get_parameter('manual_camera_name').value),
            default='front',
            parameter_name='manual_camera_name',
        )

        self._align_defaults = {
            'search_spin_speed_rad_s': float(
                self.get_parameter('align_search_spin_speed_rad_s').value
            ),
            'distance_tolerance_m': float(
                self.get_parameter('align_distance_tolerance_m').value
            ),
            'lateral_tolerance_m': float(
                self.get_parameter('align_lateral_tolerance_m').value
            ),
            'yaw_tolerance_rad': float(
                self.get_parameter('align_yaw_tolerance_rad').value
            ),
            'success_hold_steps': int(
                self.get_parameter('align_success_hold_steps').value
            ),
            'lost_scan_timeout_s': float(
                self.get_parameter('align_lost_scan_timeout_s').value
            ),
            'max_lost_retries': int(
                self.get_parameter('align_max_lost_retries').value
            ),
            'max_reverse_speed_m_s': float(
                self.get_parameter('align_max_reverse_speed_m_s').value
            ),
            'max_angular_speed_rad_s': float(
                self.get_parameter('align_max_angular_speed_rad_s').value
            ),
        }

        if abs(self.marker_x_sign) < 1e-6:
            self.get_logger().warn('marker_x_sign cannot be 0.0; using -1.0.')
            self.marker_x_sign = -1.0

        self.core = MarkerSeekCore(
            stop_distance_m=float(self.get_parameter('stop_distance_m').value),
            stop_distance_tolerance_m=float(
                self.get_parameter('stop_distance_tolerance_m').value
            ),
            spin_speed_rad_s=float(self.get_parameter('spin_speed_rad_s').value),
            approach_turn_cap_rad_s=float(
                self.get_parameter('approach_turn_cap_rad_s').value
            ),
            search_timeout_s=float(self.get_parameter('search_timeout_s').value),
            max_retries=int(self.get_parameter('max_retries').value),
            retry_nudge_speed_m_s=float(
                self.get_parameter('retry_nudge_speed_m_s').value
            ),
            retry_nudge_duration_s=float(
                self.get_parameter('retry_nudge_duration_s').value
            ),
            approach_speed_cap_m_s=float(
                self.get_parameter('approach_speed_cap_m_s').value
            ),
            yaw_kp=float(self.get_parameter('yaw_kp').value),
            distance_kp=float(self.get_parameter('distance_kp').value),
            min_linear_scale=float(self.get_parameter('min_linear_scale').value),
            angular_filter_alpha=float(
                self.get_parameter('angular_filter_alpha').value
            ),
            yaw_deadband_rad=self.yaw_deadband_rad,
            lost_marker_timeout_s=float(
                self.get_parameter('lost_marker_timeout_s').value
            ),
            heading_align_threshold_rad=float(
                self.get_parameter('heading_align_threshold_rad').value
            ),
            heading_stop_yaw_rad=float(
                self.get_parameter('heading_stop_yaw_rad').value
            ),
        )
        self._last_state = self.core.state

        self._measurements_lock = threading.Lock()
        self._latest_measurements: Dict[str, Dict[int, AlignMeasurement]] = {
            'front': {},
            'rear': {},
        }
        self._align_action_active = False
        self._align_camera_name: Optional[str] = None
        self._hook_attached = False

        self._aruco_dict = self._build_dictionary(self.aruco_dictionary_name)
        self._aruco_params = self._build_detector_parameters()
        self._aruco_detector = self._build_detector()

        self._obstacle_enabled = bool(
            self.get_parameter('obstacle_avoidance_enabled').value
        )
        self._obstacle_threshold_cm = float(
            self.get_parameter('obstacle_threshold_cm').value
        )
        self._obstacle_trigger_count = max(
            1, int(self.get_parameter('obstacle_trigger_count').value)
        )
        self._obstacle_clear_count = max(
            1, int(self.get_parameter('obstacle_clear_count').value)
        )
        sonar_pins = [int(pin) for pin in self.get_parameter('sonar_pins').value]
        self._sonars: List = []
        self._sonar_labels: List[str] = []
        if self._obstacle_enabled and _SONAR_AVAILABLE:
            for i, pin in enumerate(sonar_pins):
                try:
                    sensor = GroveUltrasonicRanger(pin)
                    self._sonars.append(sensor)
                    self._sonar_labels.append(f'sonar_{i + 1} (pin {pin})')
                except Exception as exc:
                    self.get_logger().warn(f'Failed to init sonar on pin {pin}: {exc}')
            if not self._sonars:
                self._obstacle_enabled = False
        elif self._obstacle_enabled:
            self.get_logger().warn(
                'grove library not found — obstacle avoidance disabled. Install grove.py.'
            )
            self._obstacle_enabled = False
        self._reset_obstacle_state()

        self.cmd_vel_pub = self.create_publisher(TwistStamped, self.cmd_vel_topic, 10)
        self.event_pub = self.create_publisher(String, '/marker_seek/events', 10)
        self.image_subs = {
            camera_name: self.create_subscription(
                Image,
                stream.image_topic,
                lambda msg, camera_name=camera_name: self._image_callback(msg, camera_name),
                10,
                callback_group=self._callback_group,
            )
            for camera_name, stream in self._camera_streams.items()
        }
        self.camera_info_subs = {
            camera_name: self.create_subscription(
                CameraInfo,
                stream.camera_info_topic,
                lambda msg, camera_name=camera_name: self._camera_info_callback(
                    msg,
                    camera_name,
                ),
                10,
                callback_group=self._callback_group,
            )
            for camera_name, stream in self._camera_streams.items()
        }
        self.hook_status_sub = self.create_subscription(
            HookStatus,
            self.hook_status_topic,
            self._hook_status_callback,
            10,
            callback_group=self._callback_group,
        )
        self.start_srv = self.create_service(
            Trigger,
            '/marker_seek/start',
            self._handle_start,
            callback_group=self._callback_group,
        )
        self.stop_srv = self.create_service(
            Trigger,
            '/marker_seek/stop',
            self._handle_stop,
            callback_group=self._callback_group,
        )
        self.align_action_server = ActionServer(
            self,
            AlignToMarker,
            '/align_to_marker',
            execute_callback=self._execute_align_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )
        self.control_timer = self.create_timer(
            1.0 / self.control_rate_hz,
            self._control_timer_cb,
            callback_group=self._callback_group,
        )
        self._publish_cmd(0.0, 0.0)

        if self.autostart:
            success, _ = self.core.start(now_s=time.monotonic())
            if success:
                self.get_logger().info('marker_seek_node ready. Autostart enabled.')
        else:
            self.get_logger().info(
                'marker_seek_node ready. Call /marker_seek/start or use /align_to_marker.'
            )

    def _resolve_camera_name(
        self,
        value: str,
        default: str,
        parameter_name: str,
    ) -> str:
        resolved = normalize_camera_name(value, default=default)
        if resolved != (value or '').strip().lower():
            self.get_logger().warn(
                f'{parameter_name}={value!r} is unsupported; using {resolved!r}.'
            )
        return resolved

    def _reset_obstacle_state(self) -> None:
        self._consecutive_obstacle_hits = 0
        self._consecutive_clears = 0
        self._obstacle_active = False

    def _goal_callback(self, goal_request: AlignToMarker.Goal):
        if self._align_action_active or self.core.state != SeekState.IDLE:
            return GoalResponse.REJECT
        if int(goal_request.target_marker_id) <= 0:
            return GoalResponse.REJECT
        try:
            camera_for_align_mode(
                goal_request.mode,
                pickup_camera_name=self.pickup_camera_name,
                dropoff_camera_name=self.dropoff_camera_name,
            )
        except ValueError:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        del goal_handle
        return CancelResponse.ACCEPT

    def _build_dictionary(self, dictionary_name: str):
        dictionary_map: Dict[str, int] = {
            'DICT_4X4_50': cv2.aruco.DICT_4X4_50,
            'DICT_5X5_50': cv2.aruco.DICT_5X5_50,
            'DICT_6X6_50': cv2.aruco.DICT_6X6_50,
        }
        dictionary_id = dictionary_map.get(dictionary_name, cv2.aruco.DICT_4X4_50)
        if hasattr(cv2.aruco, 'getPredefinedDictionary'):
            return cv2.aruco.getPredefinedDictionary(dictionary_id)
        return cv2.aruco.Dictionary_get(dictionary_id)

    def _build_detector_parameters(self):
        if hasattr(cv2.aruco, 'DetectorParameters'):
            return cv2.aruco.DetectorParameters()
        return cv2.aruco.DetectorParameters_create()

    def _build_detector(self):
        if hasattr(cv2.aruco, 'ArucoDetector'):
            return cv2.aruco.ArucoDetector(self._aruco_dict, self._aruco_params)
        return None

    def _hook_status_callback(self, msg: HookStatus) -> None:
        previous = self._hook_attached
        self._hook_attached = bool(msg.bin_attached)
        if previous != self._hook_attached and self.core.state in {
            SeekState.SEARCHING,
            SeekState.APPROACHING,
        }:
            self._reset_all_camera_filters()
            active_camera = self._active_camera_name()
            if active_camera is not None:
                self.get_logger().info(
                    f'Manual marker seek using {active_camera} camera '
                    f'(hook_attached={self._hook_attached}).'
                )

    def _camera_info_callback(self, msg: CameraInfo, camera_name: str) -> None:
        if len(msg.k) != 9:
            return
        stream = self._camera_streams[camera_name]

        camera_matrix = np.array(msg.k, dtype=np.float64).reshape((3, 3))
        if np.allclose(camera_matrix, 0.0):
            width = max(1.0, float(msg.width))
            height = max(1.0, float(msg.height))
            camera_matrix = np.array(
                [
                    [width, 0.0, (width - 1.0) * 0.5],
                    [0.0, width, (height - 1.0) * 0.5],
                    [0.0, 0.0, 1.0],
                ],
                dtype=np.float64,
            )
            if not stream.intrinsics_fallback_warned:
                self.get_logger().warn(
                    f'{camera_name} CameraInfo intrinsic matrix is all zeros; '
                    'using fallback intrinsics.'
                )
                stream.intrinsics_fallback_warned = True

        stream.camera_matrix = camera_matrix
        if len(msg.d) == 0:
            stream.dist_coeffs = np.zeros((5, 1), dtype=np.float64)
            if not stream.empty_distortion_warned:
                self.get_logger().warn(
                    f'{camera_name} CameraInfo distortion coefficients are empty; '
                    'assuming zero distortion.'
                )
                stream.empty_distortion_warned = True
        else:
            stream.dist_coeffs = np.array(msg.d, dtype=np.float64).reshape((-1, 1))

        if not stream.camera_info_logged:
            self.get_logger().info(
                f'Camera calibration received for {camera_name} camera.'
            )
            stream.camera_info_logged = True

    def _detect_markers(self, gray_image: np.ndarray):
        gray_image = np.ascontiguousarray(gray_image, dtype=np.uint8)
        if self._aruco_detector is not None:
            return self._aruco_detector.detectMarkers(gray_image)
        return cv2.aruco.detectMarkers(gray_image, self._aruco_dict)

    def _estimate_poses(self, corners, stream: CameraStreamState):
        if stream.camera_matrix is None or stream.dist_coeffs is None:
            return None
        corners = [np.ascontiguousarray(corner, dtype=np.float32) for corner in corners]
        result = cv2.aruco.estimatePoseSingleMarkers(
            corners,
            self.marker_size_m,
            np.ascontiguousarray(stream.camera_matrix, dtype=np.float64),
            np.ascontiguousarray(stream.dist_coeffs, dtype=np.float64),
        )
        if len(result) == 3:
            rvecs, tvecs, _ = result
        else:
            rvecs, tvecs = result
        return rvecs, tvecs

    def _estimate_marker_yaw(self, rvec: np.ndarray) -> float:
        rotation_matrix, _ = cv2.Rodrigues(
            np.ascontiguousarray(np.array(rvec, dtype=np.float64).reshape((3, 1)))
        )
        normal = rotation_matrix[:, 2]
        return float(self.marker_x_sign * math.atan2(float(normal[0]), float(normal[2])))

    def _image_to_gray(self, msg: Image) -> Optional[np.ndarray]:
        width = int(msg.width)
        height = int(msg.height)
        step = int(msg.step)
        if width <= 0 or height <= 0 or step <= 0:
            return None
        expected_min = step * height
        if len(msg.data) < expected_min:
            return None

        raw = np.frombuffer(msg.data, dtype=np.uint8)
        encoding = msg.encoding.lower()
        if encoding in {'mono8', '8uc1'}:
            gray = raw.reshape((height, step))[:, :width]
            return np.ascontiguousarray(gray, dtype=np.uint8)

        if encoding in {'bgr8', 'rgb8'}:
            channels = 3
            line_width = width * channels
            if step < line_width:
                return None
            image = raw.reshape((height, step))[:, :line_width].reshape((height, width, channels))
            color_code = cv2.COLOR_RGB2GRAY if encoding == 'rgb8' else cv2.COLOR_BGR2GRAY
            gray = cv2.cvtColor(image, color_code)
            return np.ascontiguousarray(gray, dtype=np.uint8)
        return None

    def _manual_seek_active(self) -> bool:
        return self.core.state in {
            SeekState.SEARCHING,
            SeekState.APPROACHING,
        }

    def _active_camera_name(self) -> Optional[str]:
        return active_camera_name(
            align_camera_name=self._align_camera_name,
            manual_seek_active=self._manual_seek_active(),
            manual_camera_name=self.manual_camera_name,
            hook_attached=self._hook_attached,
            use_rear_camera_when_hooked=self.manual_use_rear_when_hooked,
        )

    def _reset_measurements_for_camera(self, camera_name: str) -> None:
        with self._measurements_lock:
            self._latest_measurements[camera_name] = {}
        self._camera_streams[camera_name].filtered_yaw_rad = None

    def _reset_all_camera_filters(self) -> None:
        with self._measurements_lock:
            for camera_name in self._latest_measurements:
                self._latest_measurements[camera_name] = {}
        for stream in self._camera_streams.values():
            stream.filtered_yaw_rad = None

    def _image_callback(self, msg: Image, camera_name: str) -> None:
        active_camera = self._active_camera_name()
        if active_camera != camera_name:
            return
        stream = self._camera_streams[camera_name]
        if stream.camera_matrix is None or stream.dist_coeffs is None:
            return

        frame = self._image_to_gray(msg)
        if frame is None:
            return
        corners, ids, _ = self._detect_markers(frame)
        if ids is None or len(corners) == 0:
            with self._measurements_lock:
                self._latest_measurements[camera_name] = {}
            return

        pose_result = self._estimate_poses(corners, stream)
        if pose_result is None:
            return

        now_s = time.monotonic()
        observations: Dict[int, AlignMeasurement] = {}
        nearest_manual: Optional[AlignMeasurement] = None
        ids_flat = np.array(ids, dtype=np.int32).reshape(-1)
        rvecs, tvecs = pose_result
        for index, marker_id in enumerate(ids_flat):
            vec = np.array(tvecs[index], dtype=np.float64).reshape(-1)
            if vec.size < 3:
                continue
            z_m = float(vec[2])
            if z_m <= 0.0:
                continue
            x_m = float(self.marker_x_sign * vec[0])
            marker_yaw = self._estimate_marker_yaw(rvecs[index])
            if index < len(corners):
                yaw_from_pixel = self._compute_yaw_from_marker_center(
                    corners[index],
                    frame.shape[1],
                )
                if yaw_from_pixel is not None and np.isfinite(yaw_from_pixel):
                    filtered = yaw_from_pixel
                    if stream.filtered_yaw_rad is not None:
                        alpha = self.yaw_filter_alpha
                        filtered = (
                            (1.0 - alpha) * stream.filtered_yaw_rad
                            + alpha * yaw_from_pixel
                        )
                    if abs(filtered) < self.yaw_deadband_rad:
                        filtered = 0.0
                    stream.filtered_yaw_rad = filtered
                    x_m = z_m * math.tan(self.marker_x_sign * filtered)
            observation = AlignMeasurement(
                marker_id=int(marker_id),
                x_m=x_m,
                z_m=z_m,
                marker_yaw_rad=marker_yaw,
                stamp_s=now_s,
            )
            existing = observations.get(int(marker_id))
            if existing is None or observation.z_m < existing.z_m:
                observations[int(marker_id)] = observation
            if nearest_manual is None or observation.z_m < nearest_manual.z_m:
                nearest_manual = observation

        with self._measurements_lock:
            self._latest_measurements[camera_name] = observations

        if nearest_manual is not None and self._manual_seek_active():
            self.core.set_marker_measurement(
                x_m=nearest_manual.x_m,
                z_m=nearest_manual.z_m,
                now_s=now_s,
            )

    def _compute_yaw_from_marker_center(
        self,
        marker_corners: np.ndarray,
        image_width: int,
    ) -> Optional[float]:
        if image_width <= 1:
            return None
        points = np.array(marker_corners, dtype=np.float64).reshape(-1, 2)
        if points.shape[0] < 4:
            return None
        center_u = float(np.mean(points[:, 0]))
        half_width = 0.5 * float(image_width - 1)
        normalized_offset = (center_u - half_width) / max(1.0, half_width)
        half_fov = 0.5 * self.camera_horizontal_fov_rad
        return float(np.clip(normalized_offset * half_fov, -half_fov, half_fov))

    def _poll_obstacle_state(self) -> Tuple[bool, str]:
        if not self._obstacle_enabled or not self._sonars:
            return False, ''

        min_distance = float('inf')
        label = ''
        for sensor, sensor_label in zip(self._sonars, self._sonar_labels):
            try:
                distance_cm = sensor.get_distance()
            except Exception:
                continue
            if distance_cm < min_distance:
                min_distance = distance_cm
                label = sensor_label

        obstacle_raw = min_distance < self._obstacle_threshold_cm
        if obstacle_raw:
            self._consecutive_obstacle_hits += 1
            self._consecutive_clears = 0
        else:
            self._consecutive_clears += 1
            self._consecutive_obstacle_hits = 0

        if (
            not self._obstacle_active
            and self._consecutive_obstacle_hits >= self._obstacle_trigger_count
        ):
            self._obstacle_active = True
            return True, f'Obstacle detected at {min_distance:.1f} cm ({label})'
        if self._obstacle_active and self._consecutive_clears >= self._obstacle_clear_count:
            self._obstacle_active = False
            return False, 'Obstacle cleared'
        return self._obstacle_active, ''

    def _publish_cmd(self, linear_x: float, angular_z: float) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = float(linear_x)
        msg.twist.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(msg)

    def _publish_event(self, event_name: str) -> None:
        msg = String()
        msg.data = event_name
        self.event_pub.publish(msg)

    def _handle_start(self, request, response):
        del request
        if self._align_action_active:
            response.success = False
            response.message = 'Cannot start marker seek while AlignToMarker is active.'
            return response
        success, message = self.core.start(now_s=time.monotonic())
        response.success = success
        response.message = message
        if success:
            self._reset_all_camera_filters()
            self._reset_obstacle_state()
            active_camera = self._active_camera_name()
            self.get_logger().info(
                f'Marker seek started using {active_camera or self.manual_camera_name} camera.'
            )
        return response

    def _handle_stop(self, request, response):
        del request
        was_active = self.core.state != SeekState.IDLE
        success, message = self.core.stop(now_s=time.monotonic())
        response.success = success
        response.message = message
        self._publish_cmd(0.0, 0.0)
        if success and was_active:
            self._publish_event('STOPPED')
        self.get_logger().info('Marker seek stopped.')
        return response

    def _get_target_measurement(
        self,
        target_marker_id: int,
        camera_name: str,
    ) -> Optional[AlignMeasurement]:
        with self._measurements_lock:
            measurements = dict(self._latest_measurements.get(camera_name, {}))
        measurement = select_target_measurement(measurements, target_marker_id)
        return measurement

    def _build_alignment_status(
        self,
        core: AlignToMarkerCore,
        measurement: Optional[AlignMeasurement],
    ) -> AlignmentStatus:
        status = AlignmentStatus()
        status.surface_detected = measurement is not None
        status.hole_detected = False
        status.aligned = core.state in {AlignState.HOLDING, AlignState.SUCCEEDED}
        status.x_offset = float(core.last_lateral_error_m)
        status.y_offset = float(core.last_yaw_error_rad)
        status.distance_to_surface = float(measurement.z_m if measurement else 0.0)
        status.state = core.state.value
        return status

    def _execute_align_callback(self, goal_handle):
        goal = goal_handle.request
        try:
            align_camera_name = camera_for_align_mode(
                goal.mode,
                pickup_camera_name=self.pickup_camera_name,
                dropoff_camera_name=self.dropoff_camera_name,
            )
        except ValueError as exc:
            goal_handle.abort()
            result = AlignToMarker.Result()
            result.success = False
            result.message = str(exc)
            return result

        self._align_action_active = True
        self._align_camera_name = align_camera_name
        self._reset_measurements_for_camera(align_camera_name)
        self._reset_obstacle_state()
        self._publish_cmd(0.0, 0.0)
        self.get_logger().info(
            f'AlignToMarker started in {goal.mode!r} mode using {align_camera_name} camera.'
        )
        controller = AlignToMarkerCore(
            stand_off_m=float(goal.stand_off_m),
            lateral_offset_m=float(goal.lateral_offset_m),
            yaw_offset_rad=float(goal.yaw_offset_rad),
            timeout_s=float(goal.timeout_s),
            lost_marker_timeout_s=float(
                self.get_parameter('lost_marker_timeout_s').value
            ),
            max_linear_speed_m_s=float(
                self.get_parameter('approach_speed_cap_m_s').value
            ),
            distance_kp=float(self.get_parameter('distance_kp').value),
            search_spin_speed_rad_s=self._align_defaults['search_spin_speed_rad_s'],
            distance_tolerance_m=self._align_defaults['distance_tolerance_m'],
            lateral_tolerance_m=self._align_defaults['lateral_tolerance_m'],
            yaw_tolerance_rad=self._align_defaults['yaw_tolerance_rad'],
            success_hold_steps=self._align_defaults['success_hold_steps'],
            lost_scan_timeout_s=self._align_defaults['lost_scan_timeout_s'],
            max_lost_retries=self._align_defaults['max_lost_retries'],
            max_reverse_speed_m_s=self._align_defaults['max_reverse_speed_m_s'],
            max_angular_speed_rad_s=self._align_defaults['max_angular_speed_rad_s'],
        )
        controller.start(now_s=time.monotonic())
        obstacle_started_s: Optional[float] = None
        sleep_s = 1.0 / self.control_rate_hz

        try:
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result = AlignToMarker.Result()
                    result.success = False
                    result.message = 'Alignment canceled.'
                    return result

                now_s = time.monotonic()
                obstacle_active, obstacle_message = self._poll_obstacle_state()
                measurement = self._get_target_measurement(
                    int(goal.target_marker_id),
                    camera_name=align_camera_name,
                )
                if obstacle_active:
                    if obstacle_started_s is None:
                        obstacle_started_s = now_s
                        if obstacle_message:
                            self.get_logger().warn(obstacle_message)
                    self._publish_cmd(0.0, 0.0)
                    feedback = AlignToMarker.Feedback()
                    feedback.alignment = self._build_alignment_status(controller, measurement)
                    feedback.phase = 'OBSTACLE_WAIT'
                    goal_handle.publish_feedback(feedback)
                    if now_s - obstacle_started_s > self.obstacle_fail_timeout_s:
                        goal_handle.abort()
                        result = AlignToMarker.Result()
                        result.success = False
                        result.message = 'Obstacle persisted during alignment.'
                        return result
                    time.sleep(sleep_s)
                    continue

                obstacle_started_s = None
                command = controller.step(now_s=now_s, measurement=measurement)
                self._publish_cmd(command.linear_x, command.angular_z)
                feedback = AlignToMarker.Feedback()
                feedback.alignment = self._build_alignment_status(controller, measurement)
                feedback.phase = controller.state.value
                goal_handle.publish_feedback(feedback)

                if controller.state == AlignState.SUCCEEDED:
                    goal_handle.succeed()
                    result = AlignToMarker.Result()
                    result.success = True
                    result.message = f'Aligned to marker {goal.target_marker_id}.'
                    return result
                if controller.state == AlignState.FAILED:
                    goal_handle.abort()
                    result = AlignToMarker.Result()
                    result.success = False
                    result.message = (
                        f'Failed to align to marker {goal.target_marker_id}.'
                    )
                    return result
                time.sleep(sleep_s)
        finally:
            self._publish_cmd(0.0, 0.0)
            self._align_action_active = False
            self._align_camera_name = None

    def _control_timer_cb(self) -> None:
        if self._align_action_active:
            return

        now_s = time.monotonic()
        obstacle_active, obstacle_message = self._poll_obstacle_state()
        if obstacle_active and self.core.is_in_moving_state:
            if self.core.set_obstacle_detected(now_s) and obstacle_message:
                self.get_logger().warn(obstacle_message)
                self._publish_event('OBSTACLE_DETECTED')
        elif not obstacle_active and self.core.state == SeekState.OBSTACLE_WAIT:
            if self.core.clear_obstacle(now_s):
                if obstacle_message:
                    self.get_logger().info(obstacle_message)
                self._publish_event('OBSTACLE_CLEARED')

        previous_state = self.core.state
        command = self.core.step(now_s=now_s)
        if previous_state in {
            SeekState.SEARCHING,
            SeekState.APPROACHING,
            SeekState.RETRY_NUDGE,
            SeekState.OBSTACLE_WAIT,
        } or self.core.state in {
            SeekState.SEARCHING,
            SeekState.APPROACHING,
            SeekState.RETRY_NUDGE,
            SeekState.OBSTACLE_WAIT,
        }:
            self._publish_cmd(command.linear_x, command.angular_z)

        if self.core.state != self._last_state:
            self.get_logger().info(
                f'State transition: {self._last_state.value} -> {self.core.state.value}'
            )
            self._last_state = self.core.state

        if self.core.state == SeekState.SUCCEEDED:
            self.get_logger().info('Marker seek SUCCEEDED.')
            self._publish_event('SUCCEEDED')
            self._publish_cmd(0.0, 0.0)
            self.core.stop(now_s=now_s)
            self._last_state = self.core.state
        elif self.core.state == SeekState.FAILED:
            self.get_logger().warn('Marker seek FAILED.')
            self._publish_event('FAILED')
            self._publish_cmd(0.0, 0.0)
            self.core.stop(now_s=now_s)
            self._last_state = self.core.state

    def shutdown(self) -> None:
        self._publish_cmd(0.0, 0.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MarkerSeekNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
