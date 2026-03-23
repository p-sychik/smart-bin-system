"""Mission queue controller for multi-bin collection."""

from __future__ import annotations

import json
import time
import urllib.error
import urllib.parse
import urllib.request
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import Trigger

from bin_collection_msgs.action import AlignToMarker
from bin_collection_msgs.action import ExecutePath
from bin_collection_msgs.msg import HookStatus
from bin_collection_msgs.msg import MissionStatus
from bin_collection_msgs.srv import AbortMission
from bin_collection_msgs.srv import DisengageHook
from bin_collection_msgs.srv import EngageHook
from bin_collection_msgs.srv import StartMission

from .mission_core import CollectionSlot
from .mission_core import CollectionSlotManager
from .mission_core import MissionQueue


@dataclass(frozen=True)
class BinMissionInfo:
    """Mission-relevant bin metadata loaded from the backend."""

    bin_id: str
    path_to_bin_id: str
    path_to_collection_id: str
    path_to_home_id: str
    pickup_marker_id: int


class MissionPhase:
    """Mission controller phases."""

    IDLE = 'IDLE'
    NAVIGATING_TO_BIN = 'NAVIGATING_TO_BIN'
    ALIGNING_PICKUP = 'ALIGNING_PICKUP'
    ENGAGING_HOOK = 'ENGAGING_HOOK'
    TOWING_TO_COLLECTION = 'TOWING_TO_COLLECTION'
    ALIGNING_DROPOFF = 'ALIGNING_DROPOFF'
    DISENGAGING_HOOK = 'DISENGAGING_HOOK'
    BACKING_OUT = 'BACKING_OUT'
    RETURNING_HOME = 'RETURNING_HOME'
    PAUSED = 'PAUSED'


class MissionController(Node):
    """Run queued one-bin-at-a-time collection missions."""

    def __init__(self) -> None:
        super().__init__('mission_controller')

        share_dir = Path(get_package_share_directory('mission_controller'))
        default_layout = share_dir / 'config' / 'collection_layout.json'
        self.declare_parameter('backend_base_url', 'http://127.0.0.1:8000')
        self.declare_parameter('collection_layout_file', str(default_layout))
        self.declare_parameter('initial_pose_context', 'home')
        self.declare_parameter('path_action_name', '/path_manager/execute_path')
        self.declare_parameter('align_action_name', '/align_to_marker')
        self.declare_parameter('transit_speed_m_s', 0.12)
        self.declare_parameter('towing_speed_m_s', 0.08)
        self.declare_parameter('backout_speed_m_s', 0.08)
        self.declare_parameter('backout_duration_s', 1.5)
        self.declare_parameter('hook_timeout_s', 4.0)
        self.declare_parameter('align_timeout_s', 20.0)
        self.declare_parameter('pickup_stand_off_m', 0.30)
        self.declare_parameter('dropoff_stand_off_m', 0.45)

        self.backend_base_url = str(self.get_parameter('backend_base_url').value).rstrip('/')
        self.position_context = str(self.get_parameter('initial_pose_context').value).strip().upper()
        if self.position_context not in {'HOME', 'COLLECTION'}:
            self.position_context = 'HOME'
        self.transit_speed_m_s = float(self.get_parameter('transit_speed_m_s').value)
        self.towing_speed_m_s = float(self.get_parameter('towing_speed_m_s').value)
        self.backout_speed_m_s = float(self.get_parameter('backout_speed_m_s').value)
        self.backout_duration_s = float(self.get_parameter('backout_duration_s').value)
        self.hook_timeout_s = float(self.get_parameter('hook_timeout_s').value)
        self.align_timeout_s = float(self.get_parameter('align_timeout_s').value)
        self.pickup_stand_off_m = float(self.get_parameter('pickup_stand_off_m').value)
        self.dropoff_stand_off_m = float(self.get_parameter('dropoff_stand_off_m').value)

        self.queue = MissionQueue()
        self.phase = MissionPhase.IDLE
        self.current_bin_info: Optional[BinMissionInfo] = None
        self.current_slot_index: Optional[int] = None
        self.current_progress_percent: float = 0.0
        self.current_action: str = 'IDLE'
        self.last_error: str = ''
        self.hook_attached = False
        self._backout_deadline_s: Optional[float] = None
        self._active_path_goal_handle = None
        self._active_align_goal_handle = None
        self._pending_step: Optional[str] = None

        self.collection_marker_id, self.slot_manager = self._load_collection_layout(
            str(self.get_parameter('collection_layout_file').value)
        )

        self.status_pub = self.create_publisher(MissionStatus, '/mission/status', 10)
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.hook_status_sub = self.create_subscription(
            HookStatus,
            '/hook/status',
            self._hook_status_callback,
            10,
        )

        self.execute_path_client = ActionClient(
            self,
            ExecutePath,
            str(self.get_parameter('path_action_name').value),
        )
        self.align_client = ActionClient(
            self,
            AlignToMarker,
            str(self.get_parameter('align_action_name').value),
        )
        self.engage_client = self.create_client(EngageHook, '/hook/engage')
        self.disengage_client = self.create_client(DisengageHook, '/hook/disengage')

        self.start_mission_srv = self.create_service(
            StartMission,
            '/mission/start',
            self._handle_start_mission,
        )
        self.abort_mission_srv = self.create_service(
            AbortMission,
            '/mission/abort',
            self._handle_abort_mission,
        )
        self.clear_slots_srv = self.create_service(
            Trigger,
            '/mission/clear_collection_slots',
            self._handle_clear_collection_slots,
        )
        self.timer = self.create_timer(0.1, self._timer_callback)
        self._publish_status()
        self.get_logger().info('Mission controller ready.')

    def _load_collection_layout(self, layout_path: str):
        payload = json.loads(Path(layout_path).read_text(encoding='utf-8'))
        slots = [
            CollectionSlot(
                name=str(slot['name']),
                stand_off_m=float(slot['stand_off_m']),
                lateral_offset_m=float(slot['lateral_offset_m']),
                yaw_offset_rad=float(slot['yaw_offset_rad']),
            )
            for slot in payload.get('slots', [])
        ]
        return int(payload.get('collection_marker_id', 0)), CollectionSlotManager(slots)

    def _publish_cmd(self, linear_x: float, angular_z: float) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = float(linear_x)
        msg.twist.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(msg)

    def _publish_status(self) -> None:
        msg = MissionStatus()
        msg.mission_id = self.queue.active_job.mission_id if self.queue.active_job else ''
        msg.bin_id = self.queue.active_job.bin_id if self.queue.active_job else ''
        msg.state = self.phase
        msg.current_action = self.current_action
        msg.progress_percent = float(self.current_progress_percent)
        msg.started_at = self.get_clock().now().to_msg()
        msg.error_message = self.last_error
        self.status_pub.publish(msg)

    def _hook_status_callback(self, msg: HookStatus) -> None:
        self.hook_attached = bool(msg.bin_attached)

    def _fetch_bin_info(self, bin_id: str) -> Optional[BinMissionInfo]:
        target = urllib.parse.quote(bin_id, safe='')
        url = f'{self.backend_base_url}/api/bins/{target}'
        try:
            with urllib.request.urlopen(url, timeout=2.0) as response:
                payload = json.loads(response.read().decode('utf-8'))
        except (urllib.error.URLError, TimeoutError, ValueError) as exc:
            self.get_logger().error(f'Failed to fetch bin metadata for {bin_id}: {exc}')
            return None

        try:
            return BinMissionInfo(
                bin_id=str(payload['bin_id']),
                path_to_bin_id=str(payload['path_to_bin_id']),
                path_to_collection_id=str(payload['path_to_collection_id']),
                path_to_home_id=str(payload['path_to_home_id']),
                pickup_marker_id=int(payload['pickup_marker_id']),
            )
        except (KeyError, TypeError, ValueError) as exc:
            self.get_logger().error(f'Invalid bin metadata for {bin_id}: {exc}')
            return None

    def _set_phase(self, phase: str, action: str, progress: float = 0.0) -> None:
        self.phase = phase
        self.current_action = action
        self.current_progress_percent = progress
        self._publish_status()

    def _pause_queue(self, reason: str) -> None:
        self.last_error = reason
        self.queue.pause(reason)
        if self._active_path_goal_handle is not None:
            self._active_path_goal_handle.cancel_goal_async()
            self._active_path_goal_handle = None
        if self._active_align_goal_handle is not None:
            self._active_align_goal_handle.cancel_goal_async()
            self._active_align_goal_handle = None
        self._pending_step = None
        self._set_phase(MissionPhase.PAUSED, 'PAUSED', self.current_progress_percent)
        self._publish_cmd(0.0, 0.0)
        if self.current_slot_index is not None:
            self.slot_manager.release(self.current_slot_index)
            self.current_slot_index = None
        self.get_logger().error(reason)

    def _handle_start_mission(self, request, response):
        bin_id = request.bin_id.strip()
        if not bin_id:
            response.accepted = False
            response.mission_id = ''
            response.message = 'bin_id is required.'
            return response

        job = self.queue.enqueue(bin_id)
        response.accepted = True
        response.mission_id = job.mission_id
        if self.phase == MissionPhase.PAUSED:
            response.message = (
                f'Queued {bin_id} as {job.mission_id}; controller remains paused.'
            )
        elif self.queue.active_job is None and self.phase == MissionPhase.IDLE:
            response.message = f'Starting mission {job.mission_id} for {bin_id}.'
            self._start_next_job()
        else:
            response.message = f'Queued {bin_id} as mission {job.mission_id}.'
        return response

    def _handle_abort_mission(self, request, response):
        self.get_logger().warn(f'Mission abort requested: {request.reason}')
        if self._active_path_goal_handle is not None:
            self._active_path_goal_handle.cancel_goal_async()
        if self._active_align_goal_handle is not None:
            self._active_align_goal_handle.cancel_goal_async()
        self.queue.abort_all()
        self.current_bin_info = None
        self.current_slot_index = None
        self.last_error = request.reason
        self.position_context = 'UNKNOWN'
        self._pending_step = None
        self._active_path_goal_handle = None
        self._active_align_goal_handle = None
        self._backout_deadline_s = None
        self._publish_cmd(0.0, 0.0)
        self._set_phase(MissionPhase.IDLE, 'IDLE', 0.0)
        response.success = True
        response.message = 'Mission queue cleared.'
        return response

    def _handle_clear_collection_slots(self, request, response):
        del request
        self.slot_manager.clear_all()
        response.success = True
        response.message = 'Collection slots cleared.'
        return response

    def _start_next_job(self) -> None:
        if self.phase == MissionPhase.PAUSED:
            return
        job = self.queue.start_next()
        if job is None:
            self.current_bin_info = None
            self.last_error = ''
            self._set_phase(MissionPhase.IDLE, 'IDLE', 0.0)
            return

        bin_info = self._fetch_bin_info(job.bin_id)
        if bin_info is None:
            self._pause_queue(f'Failed to load bin metadata for {job.bin_id}.')
            return

        self.current_bin_info = bin_info
        self.last_error = ''
        if self.position_context == 'COLLECTION':
            self._start_execute_path(
                path_id=bin_info.path_to_collection_id,
                reverse=True,
                max_speed=self.transit_speed_m_s,
                next_step='ALIGN_PICKUP',
                phase=MissionPhase.NAVIGATING_TO_BIN,
                action='RETURN_TO_BIN_STAGING',
            )
        else:
            self._start_execute_path(
                path_id=bin_info.path_to_bin_id,
                reverse=False,
                max_speed=self.transit_speed_m_s,
                next_step='ALIGN_PICKUP',
                phase=MissionPhase.NAVIGATING_TO_BIN,
                action='PATH_TO_BIN',
            )

    def _start_execute_path(
        self,
        path_id: str,
        reverse: bool,
        max_speed: float,
        next_step: str,
        phase: str,
        action: str,
    ) -> None:
        if not self.execute_path_client.wait_for_server(timeout_sec=2.0):
            self._pause_queue('ExecutePath action server is unavailable.')
            return

        goal = ExecutePath.Goal()
        goal.path_id = path_id
        goal.reverse = reverse
        goal.max_speed = float(max_speed)
        self._pending_step = next_step
        self._set_phase(phase, action, 0.0)
        future = self.execute_path_client.send_goal_async(
            goal,
            feedback_callback=self._on_execute_path_feedback,
        )
        future.add_done_callback(self._on_execute_path_goal_response)

    def _on_execute_path_feedback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback.status
        self.current_progress_percent = float(feedback.progress_percent)
        self.current_action = feedback.state
        self._publish_status()

    def _on_execute_path_goal_response(self, future) -> None:
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self._pause_queue('ExecutePath goal was rejected.')
            return
        self._active_path_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_execute_path_result)

    def _on_execute_path_result(self, future) -> None:
        if self.phase == MissionPhase.PAUSED:
            return
        self._active_path_goal_handle = None
        wrapped = future.result()
        result = wrapped.result
        if not result.success:
            self._pause_queue(result.message or 'ExecutePath failed.')
            return

        step = self._pending_step
        self._pending_step = None
        if step == 'ALIGN_PICKUP':
            self._start_align_to_marker(
                marker_id=self.current_bin_info.pickup_marker_id if self.current_bin_info else 0,
                mode='pickup',
                stand_off_m=self.pickup_stand_off_m,
                lateral_offset_m=0.0,
                yaw_offset_rad=0.0,
                next_step='ENGAGE_HOOK',
                phase=MissionPhase.ALIGNING_PICKUP,
                action='ALIGN_PICKUP',
            )
        elif step == 'ALIGN_DROPOFF':
            self._maybe_start_dropoff_alignment()
        elif step == 'FINISH_HOME_RETURN':
            self._finish_final_home_return()
        else:
            self._pause_queue(f'Unknown post-path step: {step}')

    def _start_align_to_marker(
        self,
        marker_id: int,
        mode: str,
        stand_off_m: float,
        lateral_offset_m: float,
        yaw_offset_rad: float,
        next_step: str,
        phase: str,
        action: str,
    ) -> None:
        if not self.align_client.wait_for_server(timeout_sec=2.0):
            self._pause_queue('AlignToMarker action server is unavailable.')
            return

        goal = AlignToMarker.Goal()
        goal.target_marker_id = int(marker_id)
        goal.mode = mode
        goal.stand_off_m = float(stand_off_m)
        goal.lateral_offset_m = float(lateral_offset_m)
        goal.yaw_offset_rad = float(yaw_offset_rad)
        goal.timeout_s = float(self.align_timeout_s)
        self._pending_step = next_step
        self._set_phase(phase, action, 0.0)
        future = self.align_client.send_goal_async(
            goal,
            feedback_callback=self._on_align_feedback,
        )
        future.add_done_callback(self._on_align_goal_response)

    def _on_align_feedback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        alignment = feedback.alignment
        self.current_progress_percent = 100.0 if alignment.aligned else 0.0
        self.current_action = feedback.phase
        self._publish_status()

    def _on_align_goal_response(self, future) -> None:
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self._pause_queue('AlignToMarker goal was rejected.')
            return
        self._active_align_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_align_result)

    def _on_align_result(self, future) -> None:
        if self.phase == MissionPhase.PAUSED:
            return
        self._active_align_goal_handle = None
        wrapped = future.result()
        result = wrapped.result
        if not result.success:
            self._pause_queue(result.message or 'AlignToMarker failed.')
            return

        step = self._pending_step
        self._pending_step = None
        if step == 'ENGAGE_HOOK':
            self._start_engage_hook()
        elif step == 'DISENGAGE_HOOK':
            self._start_disengage_hook()
        else:
            self._pause_queue(f'Unknown post-align step: {step}')

    def _start_engage_hook(self) -> None:
        if not self.engage_client.wait_for_service(timeout_sec=2.0):
            self._pause_queue('Hook engage service is unavailable.')
            return

        self._set_phase(MissionPhase.ENGAGING_HOOK, 'ENGAGE_HOOK', 0.0)
        future = self.engage_client.call_async(EngageHook.Request())
        future.add_done_callback(self._on_engage_hook_result)

    def _on_engage_hook_result(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self._pause_queue(f'Hook engage call failed: {exc}')
            return

        if not response.success or not response.status.bin_attached:
            self._pause_queue(response.message or 'Hook engagement failed.')
            return

        if self.current_bin_info is None:
            self._pause_queue('No active bin metadata after hook engagement.')
            return

        self._start_execute_path(
            path_id=self.current_bin_info.path_to_collection_id,
            reverse=False,
            max_speed=self.towing_speed_m_s,
            next_step='ALIGN_DROPOFF',
            phase=MissionPhase.TOWING_TO_COLLECTION,
            action='PATH_TO_COLLECTION',
        )

    def _reserve_dropoff_slot(self) -> Optional[CollectionSlot]:
        index = self.slot_manager.reserve_next()
        if index is None:
            self._pause_queue('No free collection slots remain.')
            return None
        self.current_slot_index = index
        return self.slot_manager.slots[index]

    def _start_disengage_hook(self) -> None:
        if self.phase != MissionPhase.DISENGAGING_HOOK:
            self._set_phase(MissionPhase.DISENGAGING_HOOK, 'DISENGAGE_HOOK', 0.0)
        if not self.disengage_client.wait_for_service(timeout_sec=2.0):
            self._pause_queue('Hook disengage service is unavailable.')
            return
        future = self.disengage_client.call_async(DisengageHook.Request())
        future.add_done_callback(self._on_disengage_hook_result)

    def _on_disengage_hook_result(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self._pause_queue(f'Hook disengage call failed: {exc}')
            return

        if not response.success:
            self._pause_queue(response.message or 'Hook disengagement failed.')
            return

        self._backout_deadline_s = time.monotonic() + self.backout_duration_s
        self._set_phase(MissionPhase.BACKING_OUT, 'BACKING_OUT', 0.0)

    def _maybe_start_dropoff_alignment(self) -> None:
        slot = self._reserve_dropoff_slot()
        if slot is None:
            return
        self._start_align_to_marker(
            marker_id=self.collection_marker_id,
            mode='dropoff',
            stand_off_m=slot.stand_off_m,
            lateral_offset_m=slot.lateral_offset_m,
            yaw_offset_rad=slot.yaw_offset_rad,
            next_step='DISENGAGE_HOOK',
            phase=MissionPhase.ALIGNING_DROPOFF,
            action='ALIGN_DROPOFF',
        )

    def _start_final_return_home(self) -> None:
        if self.current_bin_info is None:
            self._pause_queue('No active bin metadata available for final return home.')
            return
        if not self.current_bin_info.path_to_home_id.strip():
            self._pause_queue('No collection-to-home path is configured for the final return.')
            return

        self._start_execute_path(
            path_id=self.current_bin_info.path_to_home_id,
            reverse=False,
            max_speed=self.transit_speed_m_s,
            next_step='FINISH_HOME_RETURN',
            phase=MissionPhase.RETURNING_HOME,
            action='RETURN_HOME',
        )

    def _finish_final_home_return(self) -> None:
        self.queue.complete_active()
        self.current_bin_info = None
        self.position_context = 'HOME'
        self.last_error = ''
        self._set_phase(MissionPhase.IDLE, 'IDLE', 100.0)
        self._start_next_job()

    def _complete_active_job(self) -> None:
        if self.current_slot_index is not None:
            self.slot_manager.occupy(self.current_slot_index)
        self.current_slot_index = None
        self.position_context = 'COLLECTION'
        self.last_error = ''

        if self.queue.pending:
            self.queue.complete_active()
            self.current_bin_info = None
            self._set_phase(MissionPhase.IDLE, 'IDLE', 100.0)
            self._start_next_job()
            return

        self._start_final_return_home()

    def _timer_callback(self) -> None:
        if self.phase == MissionPhase.BACKING_OUT:
            if self._backout_deadline_s is None:
                self._complete_active_job()
                return
            if time.monotonic() < self._backout_deadline_s:
                self._publish_cmd(-abs(self.backout_speed_m_s), 0.0)
                return
            self._backout_deadline_s = None
            self._publish_cmd(0.0, 0.0)
            self._complete_active_job()
            return

        if self.phase == MissionPhase.IDLE and self.queue.active_job is None and self.queue.pending:
            self._start_next_job()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_cmd(0.0, 0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
