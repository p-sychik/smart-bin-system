# Copyright 2024 Fola
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""ROS 2 node wrapping marker-seek state machine with ArUco perception."""

import math
import time
from typing import Dict, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger

from turtlebot3_marker_seek_demo2.marker_seek_core import MarkerSeekCore
from turtlebot3_marker_seek_demo2.marker_seek_core import SeekState

try:
    import cv2
except ImportError as exc:
    cv2 = None
    IMPORT_ERROR = exc
else:
    IMPORT_ERROR = None


class MarkerSeekNode(Node):
    """Autonomous marker search and approach node."""

    def __init__(self) -> None:
        """Initialize ROS interfaces and perception pipeline."""
        super().__init__('marker_seek_node')

        if cv2 is None:
            raise RuntimeError(
                f'OpenCV import failed: {IMPORT_ERROR}. '
                'Install python3-opencv first.'
            )
        if not hasattr(cv2, 'aruco'):
            raise RuntimeError('OpenCV ArUco module is unavailable.')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera_info')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('aruco_dictionary', 'DICT_4X4_50')
        self.declare_parameter('marker_size_m', 0.10)
        self.declare_parameter('stop_distance_m', 0.25)
        self.declare_parameter('spin_speed_rad_s', 0.60)
        self.declare_parameter('search_timeout_s', 12.0)
        self.declare_parameter('max_retries', 3)
        self.declare_parameter('retry_nudge_speed_m_s', 0.10)
        self.declare_parameter('retry_nudge_duration_s', 1.5)
        self.declare_parameter('approach_speed_cap_m_s', 0.10)
        self.declare_parameter('yaw_kp', 1.5)
        self.declare_parameter('distance_kp', 0.8)
        self.declare_parameter('lost_marker_timeout_s', 0.6)
        self.declare_parameter('heading_align_threshold_rad', 0.05)
        self.declare_parameter('heading_stop_yaw_rad', 0.15)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('camera_horizontal_fov_rad', 1.10)

        self.image_topic = str(self.get_parameter('image_topic').value)
        self.camera_info_topic = str(
            self.get_parameter('camera_info_topic').value
        )
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.aruco_dictionary_name = str(
            self.get_parameter('aruco_dictionary').value
        )
        self.marker_size_m = float(self.get_parameter('marker_size_m').value)
        self.control_rate_hz = float(
            self.get_parameter('control_rate_hz').value
        )
        self.camera_horizontal_fov_rad = float(
            self.get_parameter('camera_horizontal_fov_rad').value
        )
        if self.camera_horizontal_fov_rad <= 0.1:
            self.get_logger().warn(
                'camera_horizontal_fov_rad too small; using 1.10 rad.'
            )
            self.camera_horizontal_fov_rad = 1.10

        core_config = {
            'stop_distance_m': float(
                self.get_parameter('stop_distance_m').value
            ),
            'spin_speed_rad_s': float(
                self.get_parameter('spin_speed_rad_s').value
            ),
            'search_timeout_s': float(
                self.get_parameter('search_timeout_s').value
            ),
            'max_retries': int(self.get_parameter('max_retries').value),
            'retry_nudge_speed_m_s': float(
                self.get_parameter('retry_nudge_speed_m_s').value
            ),
            'retry_nudge_duration_s': float(
                self.get_parameter('retry_nudge_duration_s').value
            ),
            'approach_speed_cap_m_s': float(
                self.get_parameter('approach_speed_cap_m_s').value
            ),
            'yaw_kp': float(self.get_parameter('yaw_kp').value),
            'distance_kp': float(self.get_parameter('distance_kp').value),
            'lost_marker_timeout_s': float(
                self.get_parameter('lost_marker_timeout_s').value
            ),
            'heading_align_threshold_rad': float(
                self.get_parameter('heading_align_threshold_rad').value
            ),
            'heading_stop_yaw_rad': float(
                self.get_parameter('heading_stop_yaw_rad').value
            ),
        }
        self.core = MarkerSeekCore(**core_config)
        self._last_state = self.core.state

        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None
        self._camera_info_logged = False
        self._empty_distortion_warned = False
        self._intrinsics_fallback_warned = False

        self._aruco_dict = self._build_dictionary(self.aruco_dictionary_name)
        self._aruco_params = self._build_detector_parameters()
        self._aruco_detector = self._build_detector()

        self.cmd_vel_pub = self.create_publisher(
            TwistStamped,
            self.cmd_vel_topic,
            10,
        )
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self._image_callback,
            10,
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self._camera_info_callback,
            10,
        )
        self.start_srv = self.create_service(
            Trigger,
            '/marker_seek/start',
            self._handle_start,
        )
        self.stop_srv = self.create_service(
            Trigger,
            '/marker_seek/stop',
            self._handle_stop,
        )

        timer_period = 1.0 / max(0.1, self.control_rate_hz)
        self.control_timer = self.create_timer(
            timer_period,
            self._control_timer_cb,
        )
        self._publish_cmd(0.0, 0.0)

        self.get_logger().info(
            'marker_seek_node ready. Call /marker_seek/start to begin.'
        )

    def _build_dictionary(self, dictionary_name: str):
        dictionary_map: Dict[str, int] = {
            'DICT_4X4_50': cv2.aruco.DICT_4X4_50,
            'DICT_5X5_50': cv2.aruco.DICT_5X5_50,
            'DICT_6X6_50': cv2.aruco.DICT_6X6_50,
        }
        if dictionary_name not in dictionary_map:
            self.get_logger().warn(
                f'Unknown dictionary {dictionary_name}, '
                'falling back to DICT_4X4_50.'
            )
            dictionary_name = 'DICT_4X4_50'
        dictionary_id = dictionary_map[dictionary_name]

        if hasattr(cv2.aruco, 'getPredefinedDictionary'):
            return cv2.aruco.getPredefinedDictionary(dictionary_id)
        return cv2.aruco.Dictionary_get(dictionary_id)

    def _build_detector_parameters(self):
        if hasattr(cv2.aruco, 'DetectorParameters'):
            return cv2.aruco.DetectorParameters()
        return cv2.aruco.DetectorParameters_create()

    def _build_detector(self):
        # Prefer legacy detectMarkers path because some OpenCV Python builds
        # have runtime instability in ArucoDetector.detectMarkers.
        if hasattr(cv2.aruco, 'ArucoDetector'):
            return None
        return None

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        if len(msg.k) != 9:
            self.get_logger().warn(
                'Ignoring camera info: intrinsic matrix has invalid size.'
            )
            return

        camera_matrix = np.array(msg.k, dtype=np.float64).reshape((3, 3))
        if np.allclose(camera_matrix, 0.0):
            # Fallback when driver publishes no calibration/intrinsics.
            width = max(1.0, float(msg.width))
            height = max(1.0, float(msg.height))
            fx = width
            fy = width
            cx = (width - 1.0) * 0.5
            cy = (height - 1.0) * 0.5
            camera_matrix = np.array(
                [[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]],
                dtype=np.float64,
            )
            if not self._intrinsics_fallback_warned:
                self.get_logger().warn(
                    'CameraInfo intrinsic matrix is all zeros; '
                    'using approximate fallback intrinsics.'
                )
                self._intrinsics_fallback_warned = True

        self.camera_matrix = camera_matrix
        if len(msg.d) == 0:
            # Some camera drivers publish empty distortion; treat as zero-distortion.
            self.dist_coeffs = np.zeros((5, 1), dtype=np.float64)
            if not self._empty_distortion_warned:
                self.get_logger().warn(
                    'CameraInfo distortion coefficients are empty; '
                    'assuming zero distortion.'
                )
                self._empty_distortion_warned = True
        else:
            self.dist_coeffs = np.array(msg.d, dtype=np.float64).reshape((-1, 1))

        if not self._camera_info_logged:
            self.get_logger().info('Camera calibration received.')
            self._camera_info_logged = True

    def _detect_markers(self, gray_image: np.ndarray):
        gray_image = np.ascontiguousarray(gray_image, dtype=np.uint8)
        if self._aruco_detector is not None:
            return self._aruco_detector.detectMarkers(gray_image)
        # OpenCV 4.6.0 on this platform can segfault when the `parameters=`
        # argument is supplied to detectMarkers. Use default parameters to
        # keep runtime stable.
        return cv2.aruco.detectMarkers(gray_image, self._aruco_dict)

    def _estimate_poses(self, corners):
        if self.camera_matrix is None or self.dist_coeffs is None:
            return None

        corners = [np.ascontiguousarray(corner, dtype=np.float32) for corner in corners]
        result = cv2.aruco.estimatePoseSingleMarkers(
            corners,
            self.marker_size_m,
            np.ascontiguousarray(self.camera_matrix, dtype=np.float64),
            np.ascontiguousarray(self.dist_coeffs, dtype=np.float64),
        )
        if len(result) == 3:
            rvecs, tvecs, _ = result
        else:
            rvecs, tvecs = result
        return rvecs, tvecs

    def _select_nearest_marker(
        self,
        tvecs: np.ndarray,
    ) -> Optional[Tuple[int, float, float]]:
        best_index = -1
        best_x = None
        best_z = None
        for index, tvec in enumerate(tvecs):
            vec = np.array(tvec, dtype=np.float64).reshape(-1)
            if vec.size < 3:
                continue
            x_m = float(vec[0])
            z_m = float(vec[2])
            if z_m <= 0.0:
                continue
            if best_z is None or z_m < best_z:
                best_index = index
                best_x = x_m
                best_z = z_m

        if best_index < 0 or best_x is None or best_z is None:
            return None
        return best_index, best_x, best_z

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
        yaw = normalized_offset * half_fov
        return float(np.clip(yaw, -half_fov, half_fov))

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
            img = raw.reshape((height, step))[:, :line_width]
            img = img.reshape((height, width, channels))
            if encoding == 'rgb8':
                gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
            else:
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            return np.ascontiguousarray(gray, dtype=np.uint8)

        return None

    def _image_callback(self, msg: Image) -> None:
        if self.core.state not in {SeekState.SEARCHING, SeekState.APPROACHING}:
            return
        if self.camera_matrix is None or self.dist_coeffs is None:
            return

        frame = self._image_to_gray(msg)
        if frame is None:
            self.get_logger().warn(
                f'Unsupported or invalid image encoding: {msg.encoding}'
            )
            return

        corners, ids, _ = self._detect_markers(frame)
        if ids is None or len(corners) == 0:
            return

        pose_result = self._estimate_poses(corners)
        if pose_result is None:
            return

        _, tvecs = pose_result
        nearest = self._select_nearest_marker(tvecs)
        if nearest is None:
            return

        nearest_index, pose_x_m, z_m = nearest
        x_m = pose_x_m
        if 0 <= nearest_index < len(corners):
            yaw_from_pixel = self._compute_yaw_from_marker_center(
                corners[nearest_index],
                frame.shape[1],
            )
            if yaw_from_pixel is not None and np.isfinite(yaw_from_pixel):
                x_m = z_m * math.tan(yaw_from_pixel)

        self.core.set_marker_measurement(
            x_m=x_m,
            z_m=z_m,
            now_s=time.monotonic(),
        )

    def _handle_start(self, request, response):
        del request
        success, message = self.core.start(now_s=time.monotonic())
        response.success = success
        response.message = message
        if success:
            self.get_logger().info('Marker seek started.')
        else:
            self.get_logger().warn(message)
        return response

    def _handle_stop(self, request, response):
        del request
        success, message = self.core.stop(now_s=time.monotonic())
        response.success = success
        response.message = message
        self._publish_cmd(0.0, 0.0)
        self.get_logger().info('Marker seek stopped.')
        return response

    def _publish_cmd(self, linear_x: float, angular_z: float) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''
        msg.twist.linear.x = float(linear_x)
        msg.twist.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(msg)

    def _control_timer_cb(self) -> None:
        now_s = time.monotonic()
        cmd = self.core.step(now_s=now_s)
        self._publish_cmd(cmd.linear_x, cmd.angular_z)

        if self.core.state != self._last_state:
            old_state = self._last_state.value
            new_state = self.core.state.value
            self.get_logger().info(
                f'State transition: {old_state} -> {new_state}'
            )
            self._last_state = self.core.state


def main(args=None) -> None:
    """Entrypoint for the marker seek node."""
    rclpy.init(args=args)
    node = MarkerSeekNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._publish_cmd(0.0, 0.0)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
