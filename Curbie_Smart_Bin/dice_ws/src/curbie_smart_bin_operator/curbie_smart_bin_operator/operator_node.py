"""Main operator node for manual drive, recording, replay, and seek actions."""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32MultiArray as SonarMsg

from bin_collection_msgs.action import AlignToMarker

from .path_core import CommandRecord
from .path_core import CommandRecorder
from .path_core import PathFollower
from .path_core import PathPointRecord
from .path_core import Pose2D
from .path_core import path_point_record_to_pose
from .path_core import pose_relative
from .path_storage import EventStep
from .path_storage import MotionStep
from .path_storage import RecordedStep
from .path_storage import StoredOperatorPath
from .path_storage import delete_path
from .path_storage import list_paths
from .path_storage import load_path
from .path_storage import rename_path
from .path_storage import sanitize_path_id
from .path_storage import save_path


HOOK_UNLOCKED_POS = (0.0, 0.0)
HOOK_LOCKED_POS = (0.5, -0.5)
DEFAULT_PICKUP_MARKERS = (10, 11)
DEFAULT_COLLECTION_MARKER = 40


@dataclass(frozen=True)
class PathSummary:
    """UI-friendly path summary."""

    path_id: str
    display_name: str
    description: str
    updated_at_s: float
    step_count: int
    motion_step_count: int
    event_step_count: int
    motion_point_count: int


class OperatorNode(Node):
    """User-facing operator node with local path storage and replay."""

    def __init__(self) -> None:
        super().__init__('curbie_smart_bin_operator')

        self.declare_parameter(
            'path_storage_dir',
            str(Path('~/.ros/curbie_smart_bin/paths').expanduser()),
        )
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('servo_topic', 'servo_cmd')
        self.declare_parameter('sonar_topic', '/curbie/sonar_distances')
        self.declare_parameter('align_action_name', '/curbie/align_to_marker')
        self.declare_parameter('record_distance_threshold_m', 0.04)
        self.declare_parameter('record_yaw_threshold_deg', 4.0)
        self.declare_parameter('replay_control_rate_hz', 15.0)
        self.declare_parameter('replay_max_speed_m_s', 0.14)
        self.declare_parameter('replay_waypoint_tolerance_m', 0.05)
        self.declare_parameter('replay_final_position_tolerance_m', 0.05)
        self.declare_parameter('replay_final_heading_tolerance_rad', 0.18)
        self.declare_parameter('replay_max_angular_speed_rad_s', 0.9)
        self.declare_parameter('manual_linear_speed_m_s', 0.18)
        self.declare_parameter('manual_angular_speed_rad_s', 0.50)
        self.declare_parameter('cmd_linear_accel_limit_m_s2', 0.30)
        self.declare_parameter('cmd_linear_decel_limit_m_s2', 0.80)
        self.declare_parameter('cmd_angular_accel_limit_rad_s2', 1.50)
        self.declare_parameter('cmd_angular_decel_limit_rad_s2', 3.20)
        self.declare_parameter('pickup_stand_off_m', 0.15)
        self.declare_parameter('pickup_hook_extra_approach_m', 0.02)
        self.declare_parameter('dropoff_stand_off_m', 0.32)
        self.declare_parameter('dropoff_lateral_spacing_m', 0.50)
        self.declare_parameter('align_timeout_s', 30.0)
        self.declare_parameter('sonar_stale_timeout_s', 1.0)

        qos = QoSProfile(depth=10)
        self.path_storage_dir = str(self.get_parameter('path_storage_dir').value)
        self.replay_control_rate_hz = max(
            4.0,
            float(self.get_parameter('replay_control_rate_hz').value),
        )
        self.replay_max_speed_m_s = float(
            self.get_parameter('replay_max_speed_m_s').value
        )
        self.replay_waypoint_tolerance_m = float(
            self.get_parameter('replay_waypoint_tolerance_m').value
        )
        self.replay_final_position_tolerance_m = float(
            self.get_parameter('replay_final_position_tolerance_m').value
        )
        self.replay_final_heading_tolerance_rad = float(
            self.get_parameter('replay_final_heading_tolerance_rad').value
        )
        self.replay_max_angular_speed_rad_s = float(
            self.get_parameter('replay_max_angular_speed_rad_s').value
        )
        self.manual_linear_speed_m_s = float(
            self.get_parameter('manual_linear_speed_m_s').value
        )
        self.manual_angular_speed_rad_s = float(
            self.get_parameter('manual_angular_speed_rad_s').value
        )
        self.cmd_linear_accel_limit_m_s2 = float(
            self.get_parameter('cmd_linear_accel_limit_m_s2').value
        )
        self.cmd_linear_decel_limit_m_s2 = float(
            self.get_parameter('cmd_linear_decel_limit_m_s2').value
        )
        self.cmd_angular_accel_limit_rad_s2 = float(
            self.get_parameter('cmd_angular_accel_limit_rad_s2').value
        )
        self.cmd_angular_decel_limit_rad_s2 = float(
            self.get_parameter('cmd_angular_decel_limit_rad_s2').value
        )
        self.pickup_stand_off_m = float(self.get_parameter('pickup_stand_off_m').value)
        self.pickup_hook_extra_approach_m = max(
            0.0,
            float(self.get_parameter('pickup_hook_extra_approach_m').value),
        )
        self.dropoff_stand_off_m = float(self.get_parameter('dropoff_stand_off_m').value)
        self.dropoff_lateral_spacing_m = float(
            self.get_parameter('dropoff_lateral_spacing_m').value
        )
        self.align_timeout_s = float(self.get_parameter('align_timeout_s').value)
        self.sonar_stale_timeout_s = float(
            self.get_parameter('sonar_stale_timeout_s').value
        )

        self.cmd_vel_pub = self.create_publisher(
            TwistStamped,
            str(self.get_parameter('cmd_vel_topic').value),
            qos,
        )
        self.servo_pub = self.create_publisher(
            Float32MultiArray,
            str(self.get_parameter('servo_topic').value),
            qos,
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            str(self.get_parameter('odom_topic').value),
            self._odom_callback,
            20,
        )
        self.sonar_sub = self.create_subscription(
            SonarMsg,
            str(self.get_parameter('sonar_topic').value),
            self._sonar_callback,
            10,
        )
        self.align_client = ActionClient(
            self,
            AlignToMarker,
            str(self.get_parameter('align_action_name').value),
        )

        self.linear_x = 0.0
        self.angular_z = 0.0
        self.applied_linear_x = 0.0
        self.applied_angular_z = 0.0
        self._last_cmd_publish_s = time.monotonic()
        self.servo_left = 0.0
        self.servo_right = 0.0
        self.hook_attached = False
        self.hook_state = 'DISENGAGED'
        self._servo_lock = threading.Lock()
        self._servo_dirty = True
        self._last_servo_publish_s = 0.0
        # The TurtleBot servo driver centers after 0.5 s without updates,
        # so keepalive publishes must stay comfortably faster than that.
        self._servo_keepalive_s = 0.2

        self.current_pose: Optional[Pose2D] = None

        self.sonar_distances_m = [0.0] * 5
        self.sonar_status_text = 'Waiting for sonar topic'
        self.sonar_last_update_s = 0.0

        self.recording_active = False
        self.replay_active = False
        self.seek_active = False
        self.recording_status = 'Idle'
        self.replay_status = 'Idle'
        self.seek_status = 'Idle'
        self.last_error = ''
        self.active_path_id = ''
        self.collection_drop_count = 0
        self.selected_path_id = ''
        self.last_completed_path_id = ''
        self.last_avoidance_note = 'monitor-only (no obstacle avoidance)'
        self.align_feedback_phase = 'IDLE'
        self.align_surface_detected = False
        self.align_distance_to_surface_m = 0.0
        self.align_x_error_m = 0.0
        self.align_yaw_error_rad = 0.0
        self.align_target_marker_id: Optional[int] = None
        self.align_mode = ''
        self.current_seek_mode = ''
        self.last_seek_mode = ''
        self.last_seek_success = False

        self._state_lock = threading.Lock()
        self._replay_cancel = threading.Event()
        self._active_align_goal_handle = None
        self._active_align_lock = threading.Lock()
        self._worker_thread: Optional[threading.Thread] = None

        self._active_recorder: Optional[CommandRecorder] = None
        self._recorded_steps: list[RecordedStep] = []
        self._recording_path_id = ''
        self._recording_display_name = ''
        self._recording_description = ''

        self.cmd_timer = self.create_timer(0.1, self.publish_cmd_vel)
        self.servo_timer = self.create_timer(0.1, self.publish_servo)
        self.get_logger().info('curbie_smart_bin_operator ready.')

    def _quaternion_to_yaw(self, x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * ((w * z) + (x * y))
        cosy_cosp = 1.0 - (2.0 * ((y * y) + (z * z)))
        return math.atan2(siny_cosp, cosy_cosp)

    def _odom_callback(self, msg: Odometry) -> None:
        orientation = msg.pose.pose.orientation
        pose = Pose2D(
            x=float(msg.pose.pose.position.x),
            y=float(msg.pose.pose.position.y),
            theta=self._quaternion_to_yaw(
                x=orientation.x,
                y=orientation.y,
                z=orientation.z,
                w=orientation.w,
            ),
        )
        self.current_pose = pose

    def _sonar_callback(self, msg: SonarMsg) -> None:
        data = [float(value) for value in msg.data[:5]]
        if len(data) < 5:
            data.extend([0.0] * (5 - len(data)))
        self.sonar_distances_m = data
        self.sonar_last_update_s = time.monotonic()
        labels = ('R', 'FL', 'L', 'FR', 'F')
        self.sonar_status_text = ' | '.join(
            f'{label}:{distance:.2f}m' for label, distance in zip(labels, data)
        )

    def publish_cmd_vel(self) -> None:
        if not rclpy.ok():
            return
        now_s = time.monotonic()
        dt = max(1e-3, min(0.5, now_s - self._last_cmd_publish_s))
        self._last_cmd_publish_s = now_s
        self.applied_linear_x = self._slew_value(
            current=self.applied_linear_x,
            target=self.linear_x,
            accel_limit=self.cmd_linear_accel_limit_m_s2,
            decel_limit=self.cmd_linear_decel_limit_m_s2,
            dt=dt,
        )
        self.applied_angular_z = self._slew_value(
            current=self.applied_angular_z,
            target=self.angular_z,
            accel_limit=self.cmd_angular_accel_limit_rad_s2,
            decel_limit=self.cmd_angular_decel_limit_rad_s2,
            dt=dt,
        )
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = float(self.applied_linear_x)
        msg.twist.angular.z = float(self.applied_angular_z)
        try:
            self.cmd_vel_pub.publish(msg)
        except Exception:
            return

    def publish_servo(self, *, force: bool = False) -> None:
        if not rclpy.ok():
            return
        now_s = time.monotonic()
        with self._servo_lock:
            if (
                not force
                and not self._servo_dirty
                and (now_s - self._last_servo_publish_s) < self._servo_keepalive_s
            ):
                return
            servo_left = float(self.servo_left)
            servo_right = float(self.servo_right)
            self._servo_dirty = False
            self._last_servo_publish_s = now_s
        msg = Float32MultiArray()
        msg.data = [servo_left, servo_right]
        try:
            self.servo_pub.publish(msg)
        except Exception:
            return

    def set_motion(self, linear_x: float, angular_z: float) -> None:
        linear_x = float(linear_x)
        angular_z = float(angular_z)
        if (
            self.recording_active
            and not self.seek_active
            and not self.replay_active
            and self._active_recorder is not None
        ):
            self._active_recorder.set_command(
                linear_x,
                angular_z,
                time.monotonic(),
            )
        self.linear_x = linear_x
        self.angular_z = angular_z

    def _slew_value(
        self,
        *,
        current: float,
        target: float,
        accel_limit: float,
        decel_limit: float,
        dt: float,
    ) -> float:
        if dt <= 0.0:
            return float(current)
        if abs(target - current) < 1e-9:
            return float(target)
        if current * target < 0.0 or abs(target) <= abs(current):
            limit = max(1e-3, float(decel_limit))
        else:
            limit = max(1e-3, float(accel_limit))
        max_delta = limit * dt
        delta = max(-max_delta, min(max_delta, target - current))
        return float(current + delta)

    def stop_motion(self, *, immediate: bool = False) -> None:
        self.set_motion(0.0, 0.0)
        if immediate:
            self.applied_linear_x = 0.0
            self.applied_angular_z = 0.0
            self.publish_cmd_vel()

    def engage_hook(self) -> tuple[bool, str]:
        with self._servo_lock:
            self.servo_left, self.servo_right = HOOK_LOCKED_POS
            self.hook_attached = True
            self.hook_state = 'ENGAGED'
            self._servo_dirty = True
        self.publish_servo(force=True)
        return True, 'Hook engaged.'

    def disengage_hook(self) -> tuple[bool, str]:
        with self._servo_lock:
            self.servo_left, self.servo_right = HOOK_UNLOCKED_POS
            self.hook_attached = False
            self.hook_state = 'DISENGAGED'
            self._servo_dirty = True
        self.publish_servo(force=True)
        return True, 'Hook disengaged.'

    def _advance_before_pickup_hook(self) -> tuple[bool, str]:
        distance_m = float(self.pickup_hook_extra_approach_m)
        if distance_m <= 1e-3:
            return True, 'Skipped extra pickup approach.'
        if self.current_pose is None:
            return False, 'Odometry is unavailable for the final pickup approach.'

        start_pose = self.current_pose
        sleep_s = 1.0 / self.replay_control_rate_hz
        speed_cap = min(0.08, max(0.04, self.manual_linear_speed_m_s))
        completion_tolerance_m = min(0.005, max(0.003, distance_m * 0.12))
        timeout_s = max(4.0, distance_m / max(0.01, speed_cap) * 4.0)
        started_s = time.monotonic()

        while rclpy.ok():
            if self._replay_cancel.is_set():
                self.stop_motion(immediate=True)
                return False, 'Final pickup approach canceled.'

            if self.current_pose is None:
                time.sleep(sleep_s)
                continue

            relative = pose_relative(start_pose, self.current_pose)
            distance_traveled_m = math.hypot(relative.x, relative.y)
            remaining_m = distance_m - distance_traveled_m
            if remaining_m <= completion_tolerance_m:
                self.stop_motion(immediate=True)
                return True, (
                    f'Moved about {min(distance_traveled_m, distance_m):.2f} m '
                    'closer to the pickup marker.'
                )

            if time.monotonic() - started_s > timeout_s:
                self.stop_motion(immediate=True)
                return False, (
                    f'Final pickup approach timed out after moving '
                    f'{distance_traveled_m:.2f} m.'
                )

            speed_m_s = min(speed_cap, max(0.03, remaining_m * 1.4))
            self.set_motion(speed_m_s, 0.0)
            time.sleep(sleep_s)

        self.stop_motion(immediate=True)
        return False, 'Final pickup approach interrupted by ROS shutdown.'

    def _new_recorder(self) -> CommandRecorder:
        return CommandRecorder()

    def _finalize_active_motion_segment(self) -> None:
        if self._active_recorder is None:
            return

        now_s = time.monotonic()
        self._active_recorder.finalize(now_s)
        commands = self._active_recorder.commands
        if commands:
            self._recorded_steps.append(
                RecordedStep(
                    step_type='motion',
                    motion=MotionStep(
                        commands=commands,
                        total_duration_s=self._active_recorder.total_duration_s,
                    ),
                )
            )
        self._active_recorder = None

    def _resume_recording_after_event(self) -> None:
        if not self.recording_active:
            return
        self._active_recorder = self._new_recorder()
        self._active_recorder.start(
            time.monotonic(),
            linear_x=self.linear_x,
            angular_z=self.angular_z,
        )
        self.recording_status = 'Recording resumed after seek event.'

    def start_recording(self, display_name: str, description: str) -> tuple[bool, str]:
        if self.recording_active:
            return False, 'Recording is already active.'
        if self.replay_active or self.seek_active:
            return False, 'Stop autonomous activity before recording.'
        if self.current_pose is None:
            return False, 'Odometry is unavailable.'

        path_id = sanitize_path_id(display_name)
        path_file = Path(self.path_storage_dir).expanduser() / f'{path_id}.json'
        if path_file.exists():
            return False, f'{path_id} already exists.'

        self._recorded_steps = []
        self._active_recorder = self._new_recorder()
        self._active_recorder.start(
            time.monotonic(),
            linear_x=self.linear_x,
            angular_z=self.angular_z,
        )
        self._recording_path_id = path_id
        self._recording_display_name = display_name.strip() or path_id
        self._recording_description = description.strip()
        self.recording_active = True
        self.recording_status = f'Recording {path_id}.'
        return True, self.recording_status

    def stop_recording(self) -> tuple[bool, str]:
        if not self.recording_active:
            return False, 'No recording is active.'
        if self.seek_active:
            return False, 'Wait for the active seek to finish or cancel it first.'

        self._finalize_active_motion_segment()
        now_s = time.monotonic()
        stored_path = StoredOperatorPath(
            path_id=self._recording_path_id,
            display_name=self._recording_display_name,
            description=self._recording_description,
            created_at_s=now_s,
            updated_at_s=now_s,
            steps=list(self._recorded_steps),
        )
        save_path(self.path_storage_dir, stored_path)
        self.recording_active = False
        self.recording_status = f'Saved {stored_path.path_id}.'
        self.last_completed_path_id = stored_path.path_id
        self.active_path_id = stored_path.path_id
        self._recorded_steps = []
        self._recording_path_id = ''
        self._recording_display_name = ''
        self._recording_description = ''
        self._active_recorder = None
        return True, self.recording_status

    def _append_event_step(self, event_type: str, marker_id: int, label: str) -> None:
        self._finalize_active_motion_segment()
        self._recorded_steps.append(
            RecordedStep(
                step_type='event',
                event=EventStep(
                    event_type=event_type,
                    marker_id=int(marker_id),
                    label=label,
                ),
            )
        )

    def list_saved_paths(self) -> list[PathSummary]:
        summaries = []
        for item in list_paths(self.path_storage_dir):
            motion_steps = sum(1 for step in item.steps if step.motion is not None)
            event_steps = sum(1 for step in item.steps if step.event is not None)
            summaries.append(
                PathSummary(
                    path_id=item.path_id,
                    display_name=item.display_name,
                    description=item.description,
                    updated_at_s=item.updated_at_s,
                    step_count=len(item.steps),
                    motion_step_count=motion_steps,
                    event_step_count=event_steps,
                    motion_point_count=item.motion_point_count,
                )
            )
        return summaries

    def rename_saved_path(self, old_path_id: str, new_name: str) -> tuple[bool, str]:
        if not old_path_id.strip():
            return False, 'Select a path first.'
        if not new_name.strip():
            return False, 'Enter a new path name.'
        try:
            updated = rename_path(self.path_storage_dir, old_path_id, new_name)
        except FileExistsError as exc:
            return False, str(exc)
        except FileNotFoundError:
            return False, f'{old_path_id} was not found.'
        self.selected_path_id = updated.path_id
        return True, f'Renamed path to {updated.path_id}.'

    def delete_saved_path(self, path_id: str) -> tuple[bool, str]:
        if not path_id.strip():
            return False, 'Select a path first.'
        delete_path(self.path_storage_dir, path_id)
        if self.selected_path_id == path_id:
            self.selected_path_id = ''
        return True, f'Deleted {path_id}.'

    def set_selected_path(self, path_id: str) -> None:
        self.selected_path_id = path_id.strip()

    def handle_key(self, key: str) -> tuple[bool, str]:
        normalized = key.lower()
        if normalized == 'q':
            self.cancel_active_operation()
            return False, 'Closing UI.'

        if normalized in {'w', 'a', 's', 'd'}:
            if self.replay_active or self.seek_active:
                self.cancel_active_operation()
            if normalized == 'w':
                self.set_motion(self.manual_linear_speed_m_s, 0.0)
            elif normalized == 's':
                self.set_motion(-self.manual_linear_speed_m_s, 0.0)
            elif normalized == 'a':
                self.set_motion(0.0, self.manual_angular_speed_rad_s)
            elif normalized == 'd':
                self.set_motion(0.0, -self.manual_angular_speed_rad_s)
            return True, f'Manual motion: {normalized.upper()}'

        if normalized == 'x':
            self.cancel_active_operation()
            return True, 'Stopped motion and canceled active automation.'

        if normalized == 'c':
            return True, self.disengage_hook()[1]

        if normalized == 'v':
            return True, self.engage_hook()[1]

        if normalized == '1':
            ok, msg = self.start_pickup_seek(10)
            return True, msg

        if normalized == '2':
            ok, msg = self.start_pickup_seek(11)
            return True, msg

        if normalized == 'g':
            ok, msg = self.start_dropoff_seek()
            return True, msg

        if normalized == 'o':
            generated = time.strftime('path-%Y%m%d-%H%M%S')
            ok, msg = self.start_recording(generated, 'Quick keyboard recording')
            return True, msg

        if normalized == 'p':
            ok, msg = self.stop_recording()
            return True, msg

        if key == ' ':
            ok, msg = self.replay_path(self.selected_path_id)
            return True, msg

        return True, ''

    def reset_collection_offset(self) -> tuple[bool, str]:
        self.collection_drop_count = 0
        return True, 'Collection lateral offset reset.'

    def cancel_active_operation(self) -> None:
        self._replay_cancel.set()
        self.stop_motion(immediate=True)
        with self._active_align_lock:
            goal_handle = self._active_align_goal_handle
        if goal_handle is not None:
            try:
                goal_handle.cancel_goal_async()
            except Exception:
                pass
        self.seek_active = False
        self.replay_active = False
        self.replay_status = 'Canceled.'
        self.seek_status = 'Canceled.'
        self.align_feedback_phase = 'CANCELED'
        self.align_surface_detected = False
        self.align_target_marker_id = None
        self.align_mode = ''
        self.current_seek_mode = ''
        self.last_seek_success = False

    def start_pickup_seek(self, marker_id: int) -> tuple[bool, str]:
        if self.replay_active:
            return False, 'Replay is active.'
        if self.seek_active:
            return False, 'A seek action is already active.'
        if int(marker_id) not in DEFAULT_PICKUP_MARKERS:
            return False, 'Pickup markers are limited to 10 and 11.'

        if self.recording_active:
            self._append_event_step('pickup_seek', int(marker_id), f'Pickup marker {marker_id}')

        self._replay_cancel.clear()
        self.seek_active = True
        self.current_seek_mode = 'pickup'
        self.last_seek_mode = 'pickup'
        self.last_seek_success = False
        self.align_feedback_phase = 'SEARCHING'
        self.seek_status = (
            f'Rear camera starting 720-degree search sweeps for pickup marker {marker_id}; '
            'each new sweep reverses direction until alignment completes.'
        )
        thread = threading.Thread(
            target=self._manual_pickup_worker,
            args=(int(marker_id),),
            daemon=True,
        )
        thread.start()
        return True, self.seek_status

    def start_dropoff_seek(self) -> tuple[bool, str]:
        if self.replay_active:
            return False, 'Replay is active.'
        if self.seek_active:
            return False, 'A seek action is already active.'

        if self.recording_active:
            self._append_event_step(
                'dropoff_seek',
                DEFAULT_COLLECTION_MARKER,
                'Dropoff marker 40',
            )

        offset_m = self.collection_drop_count * self.dropoff_lateral_spacing_m
        self._replay_cancel.clear()
        self.seek_active = True
        self.current_seek_mode = 'dropoff'
        self.last_seek_mode = 'dropoff'
        self.last_seek_success = False
        self.align_feedback_phase = 'SEARCHING'
        self.seek_status = (
            f'Seeking collection marker 40 with front camera '
            f'(left offset {offset_m:.2f} m)...'
        )
        thread = threading.Thread(
            target=self._manual_dropoff_worker,
            daemon=True,
        )
        thread.start()
        return True, self.seek_status

    def _manual_pickup_worker(self, marker_id: int) -> None:
        success, message = self._run_align_action(
            marker_id=marker_id,
            mode='pickup',
            stand_off_m=self.pickup_stand_off_m,
            lateral_offset_m=0.0,
        )
        self.last_seek_mode = 'pickup'
        self.last_seek_success = False
        if success:
            self.align_feedback_phase = 'PICKUP_ADVANCING'
            self.seek_status = (
                f'Pickup marker {marker_id} aligned; moving '
                f'{self.pickup_hook_extra_approach_m:.2f} m closer before hook engagement...'
            )
            advance_ok, advance_message = self._advance_before_pickup_hook()
            message = f'{message} {advance_message}'.strip()
            if not advance_ok:
                self.align_feedback_phase = 'PICKUP_ADVANCE_FAILED'
                self.seek_active = False
                self.current_seek_mode = ''
                self.seek_status = message
                if self.recording_active:
                    self._resume_recording_after_event()
                return
            self.align_target_marker_id = int(marker_id)
            self.align_feedback_phase = 'HOOK_ENGAGING'
            self.seek_status = (
                f'Close to pickup marker {marker_id}; engaging hook now...'
            )
            hook_ok, hook_message = self.engage_hook()
            if hook_ok:
                self.align_feedback_phase = 'PICKUP_COMPLETE'
                self.last_seek_success = True
                message = (
                    f'{message} {hook_message} '
                    f'Pickup marker {marker_id} seek successful.'
                )
            else:
                self.align_feedback_phase = 'HOOK_ENGAGE_FAILED'
                message = (
                    f'{message} Failed to engage hook after pickup alignment.'
                )
        self.seek_active = False
        self.current_seek_mode = ''
        self.seek_status = message
        if self.recording_active:
            self._resume_recording_after_event()

    def _manual_dropoff_worker(self) -> None:
        offset_m = self.collection_drop_count * self.dropoff_lateral_spacing_m
        success, message = self._run_align_action(
            marker_id=DEFAULT_COLLECTION_MARKER,
            mode='dropoff',
            stand_off_m=self.dropoff_stand_off_m,
            lateral_offset_m=offset_m,
        )
        self.last_seek_mode = 'dropoff'
        self.last_seek_success = False
        if success:
            self.align_feedback_phase = 'HOOK_RELEASING'
            self.seek_status = 'Collection marker aligned; releasing hook now...'
            hook_ok, hook_message = self.disengage_hook()
            if hook_ok:
                self.collection_drop_count += 1
                self.align_feedback_phase = 'DROPOFF_COMPLETE'
                self.last_seek_success = True
                message = f'{message} {hook_message}'
            else:
                self.align_feedback_phase = 'HOOK_RELEASE_FAILED'
                message = (
                    f'{message} Failed to disengage hook after dropoff alignment.'
                )
        self.seek_active = False
        self.current_seek_mode = ''
        self.seek_status = message
        if self.recording_active:
            self._resume_recording_after_event()

    def replay_path(self, path_id: str) -> tuple[bool, str]:
        path_id = sanitize_path_id(path_id)
        if not path_id:
            return False, 'Select a saved path first.'
        if self.replay_active or self.seek_active:
            return False, 'Another autonomous activity is already active.'
        try:
            load_path(self.path_storage_dir, path_id)
        except FileNotFoundError:
            return False, f'{path_id} was not found.'
        if self.current_pose is None:
            return False, 'Odometry is unavailable.'

        self._replay_cancel.clear()
        self.replay_active = True
        self.replay_status = f'Replaying {path_id}...'
        self.active_path_id = path_id
        self._worker_thread = threading.Thread(
            target=self._replay_worker,
            args=(path_id,),
            daemon=True,
        )
        self._worker_thread.start()
        return True, self.replay_status

    def _replay_worker(self, path_id: str) -> None:
        try:
            stored_path = load_path(self.path_storage_dir, path_id)
        except FileNotFoundError:
            self.replay_active = False
            self.replay_status = f'{path_id} was not found.'
            return

        try:
            for index, step in enumerate(stored_path.steps, start=1):
                if self._replay_cancel.is_set():
                    self.replay_status = f'Replay of {path_id} canceled.'
                    return

                if step.motion is not None:
                    self.replay_status = (
                        f'Replay {path_id}: motion segment {index}/{len(stored_path.steps)}'
                    )
                    success, message = self._execute_motion_step(step.motion)
                    self.replay_status = message
                    if not success:
                        return
                    continue

                if step.event is None:
                    continue

                event = step.event
                if event.event_type == 'pickup_seek':
                    self.replay_status = (
                        f'Replay {path_id}: pickup seek marker {event.marker_id}'
                    )
                    success, message = self._run_align_action(
                        marker_id=event.marker_id,
                        mode='pickup',
                        stand_off_m=self.pickup_stand_off_m,
                        lateral_offset_m=0.0,
                    )
                    if success:
                        advance_ok, advance_message = self._advance_before_pickup_hook()
                        message = f'{message} {advance_message}'.strip()
                        if not advance_ok:
                            self.replay_status = message
                            return
                        hook_ok, hook_message = self.engage_hook()
                        if hook_ok:
                            message = f'{message} {hook_message}'
                        else:
                            message = (
                                f'{message} Failed to engage hook after pickup alignment.'
                            )
                    self.replay_status = message
                    if not success:
                        return
                elif event.event_type == 'dropoff_seek':
                    offset_m = self.collection_drop_count * self.dropoff_lateral_spacing_m
                    self.replay_status = (
                        f'Replay {path_id}: dropoff seek marker 40 '
                        f'(left offset {offset_m:.2f} m)'
                    )
                    success, message = self._run_align_action(
                        marker_id=event.marker_id or DEFAULT_COLLECTION_MARKER,
                        mode='dropoff',
                        stand_off_m=self.dropoff_stand_off_m,
                        lateral_offset_m=offset_m,
                    )
                    if success:
                        hook_ok, hook_message = self.disengage_hook()
                        if hook_ok:
                            self.collection_drop_count += 1
                            message = f'{message} {hook_message}'
                        else:
                            message = (
                                f'{message} Failed to disengage hook after dropoff alignment.'
                            )
                    self.replay_status = message
                    if not success:
                        return

            self.replay_status = f'Completed replay of {path_id}.'
            self.last_completed_path_id = path_id
        finally:
            self.stop_motion()
            self.replay_active = False

    def _execute_motion_step(self, motion: MotionStep) -> tuple[bool, str]:
        if motion.commands:
            return self._execute_command_step(motion.commands)
        return self._execute_waypoint_motion_step(motion.points)

    def _execute_command_step(
        self,
        commands: list[CommandRecord],
    ) -> tuple[bool, str]:
        if not commands:
            return True, 'Skipped empty motion segment.'
        sleep_s = 1.0 / self.replay_control_rate_hz
        executed = 0

        for command in commands:
            if self._replay_cancel.is_set():
                self.stop_motion()
                return False, 'Replay canceled.'
            self.last_avoidance_note = 'monitor-only (no obstacle avoidance)'
            self.set_motion(command.linear_x, command.angular_z)
            segment_started_s = time.monotonic()
            while rclpy.ok():
                if self._replay_cancel.is_set():
                    self.stop_motion()
                    return False, 'Replay canceled.'
                elapsed_s = time.monotonic() - segment_started_s
                if elapsed_s >= max(0.0, command.duration_s):
                    break
                time.sleep(min(sleep_s, max(0.0, command.duration_s - elapsed_s)))
            executed += 1

        self.stop_motion()
        return True, f'Motion segment complete ({executed} recorded commands).'

    def _execute_waypoint_motion_step(
        self,
        points: list[PathPointRecord],
    ) -> tuple[bool, str]:
        if not points:
            return True, 'Skipped empty motion segment.'
        if self.current_pose is None:
            return False, 'Odometry is unavailable for waypoint replay.'

        waypoints = [path_point_record_to_pose(point) for point in points]
        follower = PathFollower(
            waypoints=waypoints,
            max_speed_m_s=self.replay_max_speed_m_s,
            waypoint_tolerance_m=self.replay_waypoint_tolerance_m,
            final_position_tolerance_m=self.replay_final_position_tolerance_m,
            final_heading_tolerance_rad=self.replay_final_heading_tolerance_rad,
            max_angular_speed_rad_s=self.replay_max_angular_speed_rad_s,
        )
        reference_pose = self.current_pose
        sleep_s = 1.0 / self.replay_control_rate_hz

        while rclpy.ok():
            if self._replay_cancel.is_set():
                self.stop_motion()
                return False, 'Replay canceled.'

            if self.current_pose is None:
                time.sleep(sleep_s)
                continue

            relative_pose_now = pose_relative(reference_pose, self.current_pose)
            result = follower.step(relative_pose_now)
            if result.succeeded:
                self.stop_motion()
                return True, 'Motion segment complete.'

            command = result.command
            self.last_avoidance_note = 'monitor-only (no obstacle avoidance)'
            self.set_motion(command.linear_x, command.angular_z)
            time.sleep(sleep_s)

        self.stop_motion()
        return False, 'Replay interrupted by ROS shutdown.'

    def _align_feedback_cb(self, feedback_msg) -> None:
        try:
            feedback = feedback_msg.feedback
            self.align_feedback_phase = str(feedback.phase)
            alignment = feedback.alignment
            self.align_surface_detected = bool(alignment.surface_detected)
            self.align_distance_to_surface_m = float(alignment.distance_to_surface)
            self.align_x_error_m = float(alignment.x_offset)
            self.align_yaw_error_rad = float(alignment.y_offset)
            marker_text = (
                str(self.align_target_marker_id)
                if self.align_target_marker_id is not None
                else '?'
            )
            if self.align_mode == 'pickup':
                if not self.align_surface_detected:
                    if self.align_feedback_phase == 'LOST_SCAN':
                        self.seek_status = (
                            f'Pickup marker {marker_text} slipped out of frame; '
                            'rear camera is continuing 720-degree search sweeps and '
                            'reversing direction after each full turn.'
                        )
                    else:
                        self.seek_status = (
                            f'Rear camera performing a 720-degree search sweep for '
                            f'pickup marker {marker_text}...'
                        )
                elif self.align_feedback_phase in {'COARSE_ALIGN', 'FINE_ALIGN'}:
                    self.seek_status = (
                        f'Rear camera sees marker {marker_text}; moving in '
                        f'(z={self.align_distance_to_surface_m:.2f} m, '
                        f'x_err={self.align_x_error_m:+.2f} m).'
                    )
                elif self.align_feedback_phase == 'HOLDING':
                    self.seek_status = (
                        f'Close to marker {marker_text}; stabilizing for hook engage...'
                    )
                elif self.align_feedback_phase == 'SUCCEEDED':
                    self.seek_status = (
                        f'Pickup marker {marker_text} aligned; preparing hook engagement...'
                    )
            elif self.align_mode == 'dropoff':
                if not self.align_surface_detected:
                    if self.align_feedback_phase == 'LOST_SCAN':
                        self.seek_status = (
                            'Collection marker 40 slipped out of frame; '
                            'front camera is continuing 720-degree search sweeps and '
                            'reversing direction after each full turn.'
                        )
                    else:
                        self.seek_status = (
                            'Front camera performing a 720-degree search sweep for '
                            'collection marker 40...'
                        )
                elif self.align_feedback_phase in {'COARSE_ALIGN', 'FINE_ALIGN'}:
                    self.seek_status = (
                        f'Front camera sees collection marker 40; moving in '
                        f'(z={self.align_distance_to_surface_m:.2f} m).'
                    )
                elif self.align_feedback_phase == 'HOLDING':
                    self.seek_status = 'Close to collection marker 40; stabilizing...'
                elif self.align_feedback_phase == 'SUCCEEDED':
                    self.seek_status = 'Collection marker 40 aligned; preparing hook release...'
        except Exception:
            self.align_feedback_phase = 'UNKNOWN'

    def _run_align_action(
        self,
        *,
        marker_id: int,
        mode: str,
        stand_off_m: float,
        lateral_offset_m: float,
    ) -> tuple[bool, str]:
        self.align_mode = str(mode).strip().lower()
        self.align_target_marker_id = int(marker_id)
        self.align_surface_detected = False
        self.align_distance_to_surface_m = 0.0
        self.align_x_error_m = 0.0
        self.align_yaw_error_rad = 0.0
        if not self.align_client.wait_for_server(timeout_sec=5.0):
            self.align_target_marker_id = None
            return False, 'Alignment action server is unavailable.'

        goal = AlignToMarker.Goal()
        goal.target_marker_id = int(marker_id)
        goal.mode = str(mode)
        goal.stand_off_m = float(stand_off_m)
        goal.lateral_offset_m = float(lateral_offset_m)
        goal.yaw_offset_rad = 0.0
        goal.timeout_s = float(self.align_timeout_s)

        future = self.align_client.send_goal_async(goal, feedback_callback=self._align_feedback_cb)
        while rclpy.ok() and not future.done():
            if self._replay_cancel.is_set():
                self.align_target_marker_id = None
                return False, 'Action canceled before goal acceptance.'
            time.sleep(0.05)

        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.align_target_marker_id = None
            return False, 'Alignment goal was rejected.'

        with self._active_align_lock:
            self._active_align_goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        while rclpy.ok() and not result_future.done():
            if self._replay_cancel.is_set():
                goal_handle.cancel_goal_async()
                time.sleep(0.1)
            time.sleep(0.05)

        with self._active_align_lock:
            self._active_align_goal_handle = None

        result_wrap = result_future.result()
        if result_wrap is None:
            self.align_mode = ''
            self.align_target_marker_id = None
            return False, 'Alignment returned no result.'
        result = result_wrap.result
        outcome = bool(result.success), str(result.message)
        self.align_surface_detected = False
        self.align_mode = ''
        self.align_target_marker_id = None
        return outcome

    def shutdown(self) -> None:
        self.cancel_active_operation()
        if rclpy.ok():
            self.publish_cmd_vel()
            self.publish_servo(force=True)
