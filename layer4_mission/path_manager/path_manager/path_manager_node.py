"""ROS 2 node for odometry path recording and execution."""

from __future__ import annotations

import math
import threading
import time
from pathlib import Path
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer
from rclpy.action import CancelResponse
from rclpy.action import GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from bin_collection_msgs.action import ExecutePath
from bin_collection_msgs.msg import PathPoint
from bin_collection_msgs.msg import PathStatus
from bin_collection_msgs.msg import RecordedPath
from bin_collection_msgs.srv import GetPath
from bin_collection_msgs.srv import StartRecording
from bin_collection_msgs.srv import StopRecording

from .path_core import PathFollower
from .path_core import PathRecorder
from .path_core import Pose2D
from .path_core import build_execution_points
from .path_core import pose_relative
from .path_io import StoredPath
from .path_io import load_path
from .path_io import save_path

try:
    from grove.grove_ultrasonic_ranger import GroveUltrasonicRanger
    _SONAR_AVAILABLE = True
except ImportError:
    _SONAR_AVAILABLE = False


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Convert quaternion to yaw."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def float_to_ros_time(timestamp_s: float):
    """Convert a float timestamp to builtin_interfaces/Time fields."""
    secs = int(timestamp_s)
    nanosecs = int(max(0.0, timestamp_s - secs) * 1e9)
    return secs, nanosecs


class PathManagerNode(Node):
    """Record odometry paths and execute them relative to the current pose."""

    _DEFAULT_SONAR_PINS: List[int] = [5, 16, 18, 22, 24]

    def __init__(self) -> None:
        super().__init__('path_manager_node')
        self._callback_group = ReentrantCallbackGroup()

        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('path_storage_dir', str(Path('~/.ros/bin_paths').expanduser()))
        self.declare_parameter('record_distance_threshold_m', 0.05)
        self.declare_parameter('record_yaw_threshold_deg', 5.0)
        self.declare_parameter('controller_waypoint_tolerance_m', 0.05)
        self.declare_parameter('controller_final_position_tolerance_m', 0.04)
        self.declare_parameter('controller_final_heading_tolerance_rad', 0.15)
        self.declare_parameter('controller_linear_kp', 0.8)
        self.declare_parameter('controller_angular_kp', 1.6)
        self.declare_parameter('controller_heading_mix', 0.35)
        self.declare_parameter('controller_max_angular_speed_rad_s', 0.9)
        self.declare_parameter('control_rate_hz', 15.0)
        self.declare_parameter('obstacle_avoidance_enabled', True)
        self.declare_parameter('obstacle_threshold_cm', 30.0)
        self.declare_parameter('obstacle_trigger_count', 3)
        self.declare_parameter('obstacle_clear_count', 3)
        self.declare_parameter('obstacle_fail_timeout_s', 5.0)
        self.declare_parameter('sonar_pins', self._DEFAULT_SONAR_PINS)

        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.path_storage_dir = str(self.get_parameter('path_storage_dir').value)
        record_distance_threshold = float(
            self.get_parameter('record_distance_threshold_m').value
        )
        record_yaw_threshold_deg = float(
            self.get_parameter('record_yaw_threshold_deg').value
        )
        self.control_rate_hz = max(2.0, float(self.get_parameter('control_rate_hz').value))
        self.obstacle_fail_timeout_s = max(
            1.0,
            float(self.get_parameter('obstacle_fail_timeout_s').value),
        )
        self._controller_params = {
            'waypoint_tolerance_m': float(
                self.get_parameter('controller_waypoint_tolerance_m').value
            ),
            'final_position_tolerance_m': float(
                self.get_parameter('controller_final_position_tolerance_m').value
            ),
            'final_heading_tolerance_rad': float(
                self.get_parameter('controller_final_heading_tolerance_rad').value
            ),
            'linear_kp': float(self.get_parameter('controller_linear_kp').value),
            'angular_kp': float(self.get_parameter('controller_angular_kp').value),
            'heading_mix': float(self.get_parameter('controller_heading_mix').value),
            'max_angular_speed_rad_s': float(
                self.get_parameter('controller_max_angular_speed_rad_s').value
            ),
        }
        self._latest_pose: Optional[Pose2D] = None
        self._pose_lock = threading.Lock()

        self._active_recorder: Optional[PathRecorder] = None
        self._recording_path_id: str = ''
        self._recording_description: str = ''
        self._recording_start_pose: Optional[Pose2D] = None
        self._recording_start_time_s: float = 0.0

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
        self._consecutive_obstacle_hits = 0
        self._consecutive_clears = 0
        self._obstacle_active = False
        if self._obstacle_enabled and _SONAR_AVAILABLE:
            for i, pin in enumerate(sonar_pins):
                try:
                    sensor = GroveUltrasonicRanger(pin)
                    self._sonars.append(sensor)
                    self._sonar_labels.append(f'sonar_{i + 1} (pin {pin})')
                except Exception as exc:
                    self.get_logger().warn(f'Failed to init sonar on pin {pin}: {exc}')
            if not self._sonars:
                self.get_logger().warn(
                    'No ultrasonic sensors initialized; obstacle detection disabled.'
                )
                self._obstacle_enabled = False
        elif self._obstacle_enabled:
            self.get_logger().warn(
                'grove library not found; obstacle detection disabled in path manager.'
            )
            self._obstacle_enabled = False

        self._record_distance_threshold = record_distance_threshold
        self._record_yaw_threshold_rad = math.radians(record_yaw_threshold_deg)

        self.cmd_vel_pub = self.create_publisher(TwistStamped, self.cmd_vel_topic, 10)
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self._odom_callback,
            20,
            callback_group=self._callback_group,
        )
        self.start_recording_srv = self.create_service(
            StartRecording,
            '/path_manager/start_recording',
            self._handle_start_recording,
            callback_group=self._callback_group,
        )
        self.stop_recording_srv = self.create_service(
            StopRecording,
            '/path_manager/stop_recording',
            self._handle_stop_recording,
            callback_group=self._callback_group,
        )
        self.get_path_srv = self.create_service(
            GetPath,
            '/path_manager/get_path',
            self._handle_get_path,
            callback_group=self._callback_group,
        )
        self.execute_path_action = ActionServer(
            self,
            ExecutePath,
            '/path_manager/execute_path',
            execute_callback=self._execute_path_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )
        self._publish_cmd(0.0, 0.0)
        self.get_logger().info('Path manager ready.')

    def _goal_callback(self, goal_request: ExecutePath.Goal):
        if not goal_request.path_id.strip():
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        del goal_handle
        return CancelResponse.ACCEPT

    def _publish_cmd(self, linear_x: float, angular_z: float) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = float(linear_x)
        msg.twist.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(msg)

    def _read_pose(self) -> Optional[Pose2D]:
        with self._pose_lock:
            return self._latest_pose

    def _odom_callback(self, msg: Odometry) -> None:
        orientation = msg.pose.pose.orientation
        current_pose = Pose2D(
            x=float(msg.pose.pose.position.x),
            y=float(msg.pose.pose.position.y),
            theta=quaternion_to_yaw(
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            ),
        )
        with self._pose_lock:
            self._latest_pose = current_pose

        if self._active_recorder is None or self._recording_start_pose is None:
            return

        relative_pose = pose_relative(self._recording_start_pose, current_pose)
        self._active_recorder.add_pose(relative_pose, time.monotonic())

    def _read_sonars(self) -> Tuple[bool, float, str]:
        min_dist = float('inf')
        triggered_label = ''
        for sensor, label in zip(self._sonars, self._sonar_labels):
            try:
                distance_cm = sensor.get_distance()
            except Exception:
                continue
            if distance_cm < min_dist:
                min_dist = distance_cm
                triggered_label = label
        obstacle = min_dist < self._obstacle_threshold_cm
        return obstacle, min_dist, triggered_label

    def _obstacle_step(self) -> Tuple[bool, str]:
        if not self._obstacle_enabled or not self._sonars:
            return False, ''

        obstacle_raw, distance_cm, label = self._read_sonars()
        if obstacle_raw:
            self._consecutive_obstacle_hits += 1
            self._consecutive_clears = 0
        else:
            self._consecutive_clears += 1
            self._consecutive_obstacle_hits = 0

        if not self._obstacle_active and self._consecutive_obstacle_hits >= self._obstacle_trigger_count:
            self._obstacle_active = True
            return True, f'Obstacle detected at {distance_cm:.1f} cm ({label})'
        if self._obstacle_active and self._consecutive_clears >= self._obstacle_clear_count:
            self._obstacle_active = False
            return False, 'Obstacle cleared'
        return self._obstacle_active, ''

    def _handle_start_recording(self, request, response):
        if self._active_recorder is not None:
            response.success = False
            response.message = 'A path recording is already active.'
            return response

        current_pose = self._read_pose()
        if current_pose is None:
            response.success = False
            response.message = 'Odometry is unavailable.'
            return response

        path_id = request.path_id.strip()
        if not path_id:
            response.success = False
            response.message = 'path_id is required.'
            return response

        recorder = PathRecorder(
            distance_threshold_m=self._record_distance_threshold,
            yaw_threshold_rad=self._record_yaw_threshold_rad,
        )
        now_s = time.monotonic()
        recorder.start(now_s)
        self._active_recorder = recorder
        self._recording_path_id = path_id
        self._recording_description = request.description
        self._recording_start_pose = current_pose
        self._recording_start_time_s = now_s

        response.success = True
        response.message = f'Recording path {path_id}.'
        self.get_logger().info(response.message)
        return response

    def _finalize_recording(self) -> Tuple[Optional[StoredPath], str]:
        if self._active_recorder is None or self._recording_start_pose is None:
            return None, 'No active recording.'

        current_pose = self._read_pose()
        if current_pose is None:
            return None, 'Odometry is unavailable.'

        now_s = time.monotonic()
        relative_pose = pose_relative(self._recording_start_pose, current_pose)
        self._active_recorder.finalize(relative_pose, now_s)

        stored_path = StoredPath(
            path_id=self._recording_path_id,
            description=self._recording_description,
            total_distance_m=self._active_recorder.total_distance_m,
            recorded_at_s=now_s,
            points=self._active_recorder.points,
        )
        self._active_recorder = None
        self._recording_path_id = ''
        self._recording_description = ''
        self._recording_start_pose = None
        self._recording_start_time_s = 0.0
        return stored_path, ''

    def _handle_stop_recording(self, request, response):
        del request
        stored_path, error_message = self._finalize_recording()
        if stored_path is None:
            response.success = False
            response.message = error_message
            return response

        save_path(self.path_storage_dir, stored_path)
        response.success = True
        response.path_id = stored_path.path_id
        response.num_points = len(stored_path.points)
        response.total_distance = float(stored_path.total_distance_m)
        response.message = (
            f'Saved path {stored_path.path_id} with {len(stored_path.points)} points.'
        )
        self.get_logger().info(response.message)
        return response

    def _stored_path_to_msg(self, stored_path: StoredPath) -> RecordedPath:
        path_msg = RecordedPath()
        path_msg.path_id = stored_path.path_id
        path_msg.description = stored_path.description
        path_msg.total_distance = float(stored_path.total_distance_m)
        secs, nanosecs = float_to_ros_time(stored_path.recorded_at_s)
        path_msg.recorded_at.sec = secs
        path_msg.recorded_at.nanosec = nanosecs
        path_msg.points = []
        for point in stored_path.points:
            msg = PathPoint()
            msg.x = point.x
            msg.y = point.y
            msg.theta = point.theta
            msg.timestamp = point.timestamp
            path_msg.points.append(msg)
        return path_msg

    def _handle_get_path(self, request, response):
        try:
            stored_path = load_path(self.path_storage_dir, request.path_id.strip())
        except FileNotFoundError:
            response.found = False
            return response

        response.found = True
        response.path = self._stored_path_to_msg(stored_path)
        return response

    def _execute_path_callback(self, goal_handle):
        start_time_s = time.monotonic()
        path_id = goal_handle.request.path_id.strip()
        try:
            stored_path = load_path(self.path_storage_dir, path_id)
        except FileNotFoundError:
            goal_handle.abort()
            result = ExecutePath.Result()
            result.success = False
            result.message = f'Path {path_id} was not found.'
            result.duration_secs = 0.0
            return result

        current_pose_world = self._read_pose()
        if current_pose_world is None:
            goal_handle.abort()
            result = ExecutePath.Result()
            result.success = False
            result.message = 'Odometry is unavailable.'
            result.duration_secs = 0.0
            return result

        execution_waypoints = build_execution_points(
            stored_path.points,
            reverse=bool(goal_handle.request.reverse),
        )
        follower = PathFollower(
            waypoints=execution_waypoints,
            max_speed_m_s=max(0.05, float(goal_handle.request.max_speed or 0.12)),
            **self._controller_params,
        )
        reference_pose = current_pose_world
        obstacle_started_s: Optional[float] = None
        sleep_s = 1.0 / self.control_rate_hz

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self._publish_cmd(0.0, 0.0)
                result = ExecutePath.Result()
                result.success = False
                result.message = f'Path {path_id} was canceled.'
                result.duration_secs = float(time.monotonic() - start_time_s)
                return result

            current_pose_world = self._read_pose()
            if current_pose_world is None:
                time.sleep(sleep_s)
                continue

            current_pose = pose_relative(reference_pose, current_pose_world)
            obstacle_active, obstacle_message = self._obstacle_step()
            if obstacle_active:
                if obstacle_started_s is None:
                    obstacle_started_s = time.monotonic()
                    if obstacle_message:
                        self.get_logger().warn(obstacle_message)
                self._publish_cmd(0.0, 0.0)
                feedback = ExecutePath.Feedback()
                feedback.status = PathStatus()
                feedback.status.path_id = path_id
                feedback.status.current_waypoint = follower.current_index
                feedback.status.total_waypoints = len(execution_waypoints)
                feedback.status.progress_percent = 0.0
                feedback.status.distance_remaining = 0.0
                feedback.status.state = 'OBSTACLE_WAIT'
                goal_handle.publish_feedback(feedback)
                if time.monotonic() - obstacle_started_s > self.obstacle_fail_timeout_s:
                    goal_handle.abort()
                    self._publish_cmd(0.0, 0.0)
                    result = ExecutePath.Result()
                    result.success = False
                    result.message = 'Obstacle persisted during path execution.'
                    result.duration_secs = float(time.monotonic() - start_time_s)
                    return result
                time.sleep(sleep_s)
                continue

            if obstacle_started_s is not None and obstacle_message:
                self.get_logger().info(obstacle_message)
            obstacle_started_s = None

            step_result = follower.step(current_pose)
            self._publish_cmd(
                step_result.command.linear_x,
                step_result.command.angular_z,
            )
            feedback = ExecutePath.Feedback()
            feedback.status = PathStatus()
            feedback.status.path_id = path_id
            feedback.status.current_waypoint = step_result.current_waypoint
            feedback.status.total_waypoints = step_result.total_waypoints
            feedback.status.progress_percent = float(step_result.progress_percent)
            feedback.status.distance_remaining = float(step_result.distance_remaining)
            feedback.status.state = step_result.state
            goal_handle.publish_feedback(feedback)

            if step_result.succeeded:
                goal_handle.succeed()
                self._publish_cmd(0.0, 0.0)
                result = ExecutePath.Result()
                result.success = True
                result.message = f'Executed path {path_id}.'
                result.duration_secs = float(time.monotonic() - start_time_s)
                return result

            time.sleep(sleep_s)

        self._publish_cmd(0.0, 0.0)
        goal_handle.abort()
        result = ExecutePath.Result()
        result.success = False
        result.message = 'ROS shutdown interrupted path execution.'
        result.duration_secs = float(time.monotonic() - start_time_s)
        return result

    def shutdown(self) -> None:
        self._publish_cmd(0.0, 0.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PathManagerNode()
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
