import math
import select
import sys
import termios
import time
import tty
from typing import Optional

from bin_collection_msgs.msg import HookStatus
from bin_collection_msgs.srv import DisengageHook
from bin_collection_msgs.srv import EngageHook
import rclpy
from rclpy.clock import Clock
from rclpy.node import Node
from rclpy.qos import QoSProfile

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray, String
from std_srvs.srv import Trigger
from turtlebot3_combined_teleop.recording_playback_core import Pose2D
from turtlebot3_combined_teleop.recording_playback_core import absolute_pose
from turtlebot3_combined_teleop.recording_playback_core import compute_waypoint_command
from turtlebot3_combined_teleop.recording_playback_core import relative_pose

ODOM_TOPIC = 'odom'
PLAYBACK_CONTROL_RATE_HZ = 20.0
PLAYBACK_MIN_SEGMENT_TIMEOUT_S = 1.5
PLAYBACK_TIMEOUT_SCALE = 3.0
POSE_WAIT_TIMEOUT_S = 1.0
HOOK_SETTLE_TIMEOUT_S = 1.2
HOOK_UNLOCKED_POS = (0.0, 0.0)
HOOK_LOCKED_POS = (0.5, -0.5)
PLAYBACK_POSITION_TOLERANCE_M = 0.12
PLAYBACK_YAW_TOLERANCE_RAD = 0.20

USAGE_MSG = """
Combined TurtleBot3 Teleop
--------------------------
Movement:
    W : Forward (0.2 m/s)
    S : Backward (-0.2 m/s)
    A : Turn left (0.5 rad/s)
    D : Turn right (-0.5 rad/s)
    X : Stop movement + cancel marker seek

Servos:
    C : Unhook / center servos
    V : Hook / V-position

Recording:
    O : Start recording
    P : Stop recording
    SPACE : Replay recorded run (waits for marker seek)

Marker Seek:
    F : Start marker seek / insert auto-find step while recording

    T : Run test sequence

    Q : Quit
"""

# (delta_t, linear_x, angular_z, servo_left, servo_right[, marker_action[, hook_action]])
# marker_action can be: '', 'start', 'stop'
# hook_action can be: '', 'engage', 'disengage'
TEST_SEQUENCE = [
    (0.0,  0.2,  0.0,  0.0,  0.0),   # Forward
    (2.0,  0.0,  0.5,  0.0,  0.0),   # Turn left
    (2.0,  0.2,  0.0,  1.0, -1.0),   # Forward + servos V
    (2.0,  0.0,  0.0,  0.0,  0.0),   # Stop + servos center
    (2.0, -0.2,  0.0,  0.0,  0.0),   # Backward
    (2.0,  0.0,  0.0,  0.0,  0.0),   # Stop
]

MARKER_SEEK_TERMINAL_EVENTS = {'SUCCEEDED', 'FAILED', 'STOPPED'}
MARKER_SEEK_REPLAY_TIMEOUT_S = 60.0


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class CombinedTeleopNode(Node):

    def __init__(self):
        super().__init__('combined_teleop_node')

        qos = QoSProfile(depth=10)
        self.cmd_vel_pub = self.create_publisher(
            TwistStamped, 'cmd_vel', qos)
        self.servo_pub = self.create_publisher(
            Float32MultiArray, 'servo_cmd', 10)

        self.linear_x = 0.0
        self.angular_z = 0.0
        self.servo_left = 0.0
        self.servo_right = 0.0

        self.servo_timer = self.create_timer(0.05, self.publish_servo)

        self.recording = False
        self.recorded_actions = []
        self.last_record_time = 0.0
        self.marker_seek_running = False
        self.last_marker_seek_event = 'IDLE'
        self._recording_waiting_for_marker_seek_completion = False
        self._replay_active = False
        self.marker_seek_start_client = self.create_client(
            Trigger, '/marker_seek/start')
        self.marker_seek_stop_client = self.create_client(
            Trigger, '/marker_seek/stop')
        self.marker_seek_event_sub = self.create_subscription(
            String,
            '/marker_seek/events',
            self._marker_seek_event_callback,
            10,
        )
        self.hook_status_sub = self.create_subscription(
            HookStatus,
            '/hook/status',
            self._hook_status_callback,
            10,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            ODOM_TOPIC,
            self._odom_callback,
            10,
        )
        self.hook_engage_client = self.create_client(EngageHook, '/hook/engage')
        self.hook_disengage_client = self.create_client(
            DisengageHook,
            '/hook/disengage',
        )
        self.current_pose: Optional[Pose2D] = None
        self._recording_anchor_pose: Optional[Pose2D] = None
        self._replay_anchor_pose: Optional[Pose2D] = None
        self.hook_attached: Optional[bool] = None
        self.hook_state = 'UNKNOWN'
        self._hook_status_update_count = 0

    def publish_cmd_vel(self):
        msg = TwistStamped()
        msg.header.stamp = Clock().now().to_msg()
        msg.header.frame_id = ''
        msg.twist.linear.x = self.linear_x
        msg.twist.angular.z = self.angular_z
        self.cmd_vel_pub.publish(msg)

    def publish_servo(self):
        msg = Float32MultiArray()
        msg.data = [self.servo_left, self.servo_right]
        self.servo_pub.publish(msg)

    def publish_all(self):
        self.publish_cmd_vel()
        self.publish_servo()

    def _odom_callback(self, msg):
        orientation = msg.pose.pose.orientation
        self.current_pose = Pose2D(
            x_m=float(msg.pose.pose.position.x),
            y_m=float(msg.pose.pose.position.y),
            yaw_rad=self._quaternion_to_yaw(
                x=orientation.x,
                y=orientation.y,
                z=orientation.z,
                w=orientation.w,
            ),
        )

    def _quaternion_to_yaw(self, x, y, z, w):
        siny_cosp = 2.0 * ((w * z) + (x * y))
        cosy_cosp = 1.0 - (2.0 * ((y * y) + (z * z)))
        return math.atan2(siny_cosp, cosy_cosp)

    def _wait_for_current_pose(self, timeout_s=POSE_WAIT_TIMEOUT_S):
        if self.current_pose is not None:
            return True

        deadline = time.monotonic() + max(0.0, timeout_s)
        while rclpy.ok() and self.current_pose is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)
        return self.current_pose is not None

    def _current_recording_relative_pose(self):
        if self.current_pose is None:
            return None
        if self._recording_anchor_pose is None:
            self._recording_anchor_pose = self.current_pose
        return relative_pose(self._recording_anchor_pose, self.current_pose)

    def _hook_status_callback(self, msg):
        self.hook_attached = bool(msg.bin_attached)
        self.hook_state = msg.state or 'UNKNOWN'
        self._hook_status_update_count += 1

    def _normalize_recorded_action(self, action):
        if len(action) == 5:
            dt, lin_x, ang_z, s_left, s_right = action
            marker_action = ''
            hook_action = ''
            target_pose = None
        elif len(action) == 6:
            dt, lin_x, ang_z, s_left, s_right, marker_action_raw = action
            if isinstance(marker_action_raw, bool):
                marker_action = 'start' if marker_action_raw else ''
            elif isinstance(marker_action_raw, str):
                marker_action = marker_action_raw.strip().lower()
            else:
                marker_action = ''
            hook_action = ''
            target_pose = None
        elif len(action) == 7:
            dt, lin_x, ang_z, s_left, s_right, marker_action_raw, hook_action_raw = action
            if isinstance(marker_action_raw, bool):
                marker_action = 'start' if marker_action_raw else ''
            elif isinstance(marker_action_raw, str):
                marker_action = marker_action_raw.strip().lower()
            else:
                marker_action = ''
            if isinstance(hook_action_raw, str):
                hook_action = hook_action_raw.strip().lower()
            else:
                hook_action = ''
            target_pose = None
        elif len(action) == 9:
            (
                dt,
                lin_x,
                ang_z,
                s_left,
                s_right,
                marker_action_raw,
                rel_x,
                rel_y,
                rel_yaw,
            ) = action
            if isinstance(marker_action_raw, bool):
                marker_action = 'start' if marker_action_raw else ''
            elif isinstance(marker_action_raw, str):
                marker_action = marker_action_raw.strip().lower()
            else:
                marker_action = ''
            hook_action = ''
            target_pose = Pose2D(
                x_m=float(rel_x),
                y_m=float(rel_y),
                yaw_rad=float(rel_yaw),
            )
        elif len(action) == 10:
            (
                dt,
                lin_x,
                ang_z,
                s_left,
                s_right,
                marker_action_raw,
                hook_action_raw,
                rel_x,
                rel_y,
                rel_yaw,
            ) = action
            if isinstance(marker_action_raw, bool):
                marker_action = 'start' if marker_action_raw else ''
            elif isinstance(marker_action_raw, str):
                marker_action = marker_action_raw.strip().lower()
            else:
                marker_action = ''
            if isinstance(hook_action_raw, str):
                hook_action = hook_action_raw.strip().lower()
            else:
                hook_action = ''
            target_pose = Pose2D(
                x_m=float(rel_x),
                y_m=float(rel_y),
                yaw_rad=float(rel_yaw),
            )
        else:
            raise ValueError(f'Unsupported recorded action format: {action}')
        if marker_action not in {'', 'start', 'stop'}:
            marker_action = ''
        if hook_action not in {'', 'engage', 'disengage'}:
            hook_action = ''
        return dt, lin_x, ang_z, s_left, s_right, marker_action, hook_action, target_pose

    def record_action(self, marker_action='', hook_action=''):
        if not self.recording:
            return
        now = time.monotonic()
        if len(self.recorded_actions) == 0:
            delta = 0.0
        else:
            delta = now - self.last_record_time
        target_pose = self._current_recording_relative_pose()
        if target_pose is None:
            self.recorded_actions.append((
                delta,
                self.linear_x,
                self.angular_z,
                self.servo_left,
                self.servo_right,
                marker_action if marker_action in {'start', 'stop'} else '',
                hook_action if hook_action in {'engage', 'disengage'} else '',
            ))
        else:
            self.recorded_actions.append((
                delta,
                self.linear_x,
                self.angular_z,
                self.servo_left,
                self.servo_right,
                marker_action if marker_action in {'start', 'stop'} else '',
                hook_action if hook_action in {'engage', 'disengage'} else '',
                target_pose.x_m,
                target_pose.y_m,
                target_pose.yaw_rad,
            ))
        self.last_record_time = now

    def _begin_recorded_marker_seek_segment(self):
        if not self.recording:
            return
        self._recording_waiting_for_marker_seek_completion = True

    def _finish_recorded_marker_seek_segment(self, terminal_event):
        self.marker_seek_running = False
        self.last_marker_seek_event = terminal_event
        if self._recording_waiting_for_marker_seek_completion:
            self.last_record_time = time.monotonic()
            self._recording_waiting_for_marker_seek_completion = False

    def _wait_for_marker_seek_completion(
        self,
        timeout_s=MARKER_SEEK_REPLAY_TIMEOUT_S,
    ):
        if not self.marker_seek_running:
            return self.last_marker_seek_event == 'SUCCEEDED', self.last_marker_seek_event

        deadline = time.monotonic() + max(0.0, timeout_s)
        while rclpy.ok() and self.marker_seek_running and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)

        if self.marker_seek_running:
            print(
                f'Marker seek replay wait timed out after {timeout_s:.1f} s; '
                'attempting stop.'
            )
            self.stop_marker_seek()
            self.last_marker_seek_event = 'TIMEOUT'
            return False, 'TIMEOUT'

        terminal_event = self.last_marker_seek_event or 'UNKNOWN'
        return terminal_event == 'SUCCEEDED', terminal_event

    def replay(self):
        if self._replay_active:
            print('Replay is already running.')
            return
        count = len(self.recorded_actions)
        if count == 0:
            print('No recorded actions to replay.')
            return
        if not self._wait_for_current_pose():
            print('Odometry unavailable at replay start; using timed fallback where needed.')
        self._replay_anchor_pose = self.current_pose
        self._replay_active = True
        replay_completed = False
        pending_rejoin_after_seek = False
        print(f'Replaying {count} actions...')
        try:
            for i, action in enumerate(self.recorded_actions):
                (
                    dt,
                    lin_x,
                    ang_z,
                    s_left,
                    s_right,
                    marker_action,
                    hook_action,
                    target_pose,
                ) = (
                    self._normalize_recorded_action(action)
                )

                if i > 0 and dt > 0.0:
                    end_time = time.monotonic() + dt
                    while time.monotonic() < end_time:
                        self.publish_all()
                        rclpy.spin_once(self, timeout_sec=0.01)
                        time.sleep(0.05)

                should_rejoin_after_seek = (
                    pending_rejoin_after_seek
                    and self._action_has_motion(lin_x, ang_z)
                )
                if should_rejoin_after_seek:
                    if (
                        target_pose is not None
                        and self._replay_anchor_pose is not None
                        and self.current_pose is not None
                    ):
                        rejoin_success = self._drive_to_recorded_pose(
                            target_pose=target_pose,
                            direction_hint=lin_x if abs(lin_x) > 1e-4 else self.linear_x,
                            timeout_s=max(
                                PLAYBACK_MIN_SEGMENT_TIMEOUT_S,
                                dt * PLAYBACK_TIMEOUT_SCALE,
                            ),
                        )
                        if not rejoin_success:
                            print(
                                f'Replay aborted at action {i + 1}: '
                                'could not rejoin the recorded path after marker seek.'
                            )
                            return
                    else:
                        print(
                            f'No recorded pose available for post-seek correction at '
                            f'action {i + 1}; continuing timed replay.'
                        )
                    pending_rejoin_after_seek = False

                self.linear_x = lin_x
                self.angular_z = ang_z
                self.servo_left = s_left
                self.servo_right = s_right
                self.publish_all()
                if hook_action:
                    self._perform_hook_action(hook_action, wait_for_status=True)
                if marker_action == 'start':
                    status = self.start_marker_seek()
                    if status == 'failed':
                        print(
                            f'Replay aborted at action {i + 1}: '
                            'could not start marker seek.'
                        )
                        return
                    succeeded, terminal_event = self._wait_for_marker_seek_completion()
                    if not succeeded:
                        print(
                            f'Replay aborted at action {i + 1}: '
                            f'marker seek ended with {terminal_event}.'
                        )
                        return
                    pending_rejoin_after_seek = True
                elif marker_action == 'stop':
                    self.stop_marker_seek()
                rclpy.spin_once(self, timeout_sec=0.01)
                print(f'  [{i + 1}/{count}] lin={lin_x:.1f} ang={ang_z:.1f}'
                      f' servo=({s_left:.1f}, {s_right:.1f})'
                      f' marker_action={marker_action or "none"}'
                      f' hook_action={hook_action or "none"}')
            replay_completed = True
        finally:
            self.linear_x = 0.0
            self.angular_z = 0.0
            self.publish_cmd_vel()
            self._replay_active = False
        if replay_completed:
            print('Replay complete. Robot stopped.')

    def _action_has_motion(self, linear_x, angular_z):
        return abs(linear_x) > 1e-4 or abs(angular_z) > 1e-4

    def _call_trigger_service(self, client, service_name):
        if not client.wait_for_service(timeout_sec=0.20):
            print(f'{service_name} unavailable.')
            return False, 'service unavailable'

        request = Trigger.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        if not future.done():
            print(f'{service_name} call timed out.')
            return False, 'timeout'

        response = future.result()
        if response is None:
            print(f'{service_name} call failed with no response.')
            return False, 'no response'

        return bool(response.success), response.message

    def _call_hook_service(self, client, service_name):
        if not client.wait_for_service(timeout_sec=0.20):
            print(f'{service_name} unavailable.')
            return False, 'service unavailable', None

        if service_name == '/hook/engage':
            request = EngageHook.Request()
        else:
            request = DisengageHook.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        if not future.done():
            print(f'{service_name} call timed out.')
            return False, 'timeout', None

        response = future.result()
        if response is None:
            print(f'{service_name} call failed with no response.')
            return False, 'no response', None

        return bool(response.success), response.message, response.status

    def _wait_for_hook_status_update(
        self,
        *,
        expected_attached,
        previous_update_count,
        timeout_s=HOOK_SETTLE_TIMEOUT_S,
    ):
        deadline = time.monotonic() + max(0.0, timeout_s)
        while rclpy.ok() and time.monotonic() < deadline:
            if (
                self._hook_status_update_count > previous_update_count
                and self.hook_attached == expected_attached
            ):
                return True
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)
        return False

    def _perform_hook_action(self, hook_action, wait_for_status):
        if hook_action == 'engage':
            client = self.hook_engage_client
            service_name = '/hook/engage'
            expected_attached = True
        elif hook_action == 'disengage':
            client = self.hook_disengage_client
            service_name = '/hook/disengage'
            expected_attached = False
        else:
            return False

        previous_update_count = self._hook_status_update_count
        success, message, status = self._call_hook_service(client, service_name)
        already_in_state = (
            ('Already engaged' in message and expected_attached)
            or ('Already disengaged' in message and not expected_attached)
        )
        if status is not None:
            self.hook_attached = bool(status.bin_attached)
            self.hook_state = status.state or self.hook_state

        if success or already_in_state:
            if already_in_state and self.hook_attached == expected_attached:
                return True
            if wait_for_status:
                status_synced = self._wait_for_hook_status_update(
                    expected_attached=expected_attached,
                    previous_update_count=previous_update_count,
                )
                if not status_synced:
                    time.sleep(0.20)
                    print(
                        f'Hook topic did not confirm {hook_action} within '
                        f'{HOOK_SETTLE_TIMEOUT_S:.1f} s.'
                    )
            return True

        print(f'Failed to {hook_action} hook via {service_name}: {message}')
        time.sleep(0.20)
        return False

    def _drive_to_recorded_pose(
        self,
        *,
        target_pose,
        direction_hint,
        timeout_s,
    ):
        if self._replay_anchor_pose is None:
            return False

        target_world_pose = absolute_pose(self._replay_anchor_pose, target_pose)
        deadline = time.monotonic() + max(timeout_s, PLAYBACK_MIN_SEGMENT_TIMEOUT_S)
        sleep_s = 1.0 / PLAYBACK_CONTROL_RATE_HZ

        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.01)
            if self.current_pose is None:
                time.sleep(sleep_s)
                continue

            command = compute_waypoint_command(
                current_pose=self.current_pose,
                target_pose=target_world_pose,
                direction_hint=direction_hint,
                position_tolerance_m=PLAYBACK_POSITION_TOLERANCE_M,
                yaw_tolerance_rad=PLAYBACK_YAW_TOLERANCE_RAD,
                require_yaw_alignment=False,
            )
            self.linear_x = command.linear_x
            self.angular_z = command.angular_z
            self.publish_cmd_vel()

            if command.reached:
                self.linear_x = 0.0
                self.angular_z = 0.0
                self.publish_cmd_vel()
                return True

            time.sleep(sleep_s)

        self.linear_x = 0.0
        self.angular_z = 0.0
        self.publish_cmd_vel()
        return False

    def start_marker_seek(self):
        success, message = self._call_trigger_service(
            self.marker_seek_start_client, '/marker_seek/start')
        if success:
            self.marker_seek_running = True
            self.last_marker_seek_event = 'RUNNING'
            print('Marker seek started.')
            return 'started'

        if 'Cannot start while in' in message:
            self.marker_seek_running = True
            self.last_marker_seek_event = 'RUNNING'
            print('Marker seek is already running.')
            return 'already_running'
        print(f'Failed to start marker seek: {message}')
        return 'failed'

    def stop_marker_seek(self):
        success, message = self._call_trigger_service(
            self.marker_seek_stop_client, '/marker_seek/stop')
        if success:
            self._finish_recorded_marker_seek_segment('STOPPED')
            print('Marker seek stopped.')
            return True
        print(f'Failed to stop marker seek: {message}')
        return False

    def _marker_seek_event_callback(self, msg):
        event = msg.data.strip().upper()
        if not event:
            return
        if event in MARKER_SEEK_TERMINAL_EVENTS:
            self._finish_recorded_marker_seek_segment(event)
            print(f'\nMarker seek {event}.')
            return
        self.last_marker_seek_event = event
        print(f'\nMarker seek event: {event}')

    def handle_key(self, key):
        k = key.lower()
        state_changed = False
        marker_action = ''

        if k == 'w':
            self.linear_x = 0.2
            self.angular_z = 0.0
            state_changed = True
            print('Forward')
        elif k == 's':
            self.linear_x = -0.2
            self.angular_z = 0.0
            state_changed = True
            print('Backward')
        elif k == 'a':
            self.linear_x = 0.0
            self.angular_z = 0.5
            state_changed = True
            print('Turn left')
        elif k == 'd':
            self.linear_x = 0.0
            self.angular_z = -0.5
            state_changed = True
            print('Turn right')
        elif k == 'x':
            self.linear_x = 0.0
            self.angular_z = 0.0
            if self.stop_marker_seek():
                marker_action = 'stop'
            state_changed = True
            print('Stop')
        elif k == 'c':
            self.servo_left, self.servo_right = HOOK_UNLOCKED_POS
            state_changed = True
            self._perform_hook_action('disengage', wait_for_status=True)
            print('Hook disengaged')
        elif k == 'v':
            self.servo_left, self.servo_right = HOOK_LOCKED_POS
            state_changed = True
            self._perform_hook_action('engage', wait_for_status=True)
            print('Hook engaged')
        elif k == 'o':
            if not self._wait_for_current_pose():
                print('Odometry unavailable at record start; new actions will fall back to timed replay.')
            self.recording = True
            self.recorded_actions = []
            self.last_record_time = 0.0
            self._recording_anchor_pose = self.current_pose
            self._recording_waiting_for_marker_seek_completion = False
            print('Recording started')
        elif k == 'p':
            self.recording = False
            self._recording_waiting_for_marker_seek_completion = False
            print(f'Recording stopped ({len(self.recorded_actions)} actions):')
            for i, action in enumerate(self.recorded_actions):
                (
                    dt,
                    lin_x,
                    ang_z,
                    s_left,
                    s_right,
                    recorded_marker_action,
                    recorded_hook_action,
                    target_pose,
                ) = (
                    self._normalize_recorded_action(action)
                )
                pose_suffix = ''
                if target_pose is not None:
                    pose_suffix = (
                        f', pose=({target_pose.x_m:.2f}, {target_pose.y_m:.2f}, '
                        f'{target_pose.yaw_rad:.2f})'
                    )
                print(
                    f'  ({dt:.2f}, {lin_x:.1f}, {ang_z:.1f}, '
                    f'{s_left:.1f}, {s_right:.1f}, '
                    f'{recorded_marker_action or "none"}, '
                    f'{recorded_hook_action or "none"}{pose_suffix}),'
                )

        elif k == 't':
            print('Loading test sequence...')
            self.recorded_actions = list(TEST_SEQUENCE)
            self.replay()
            return True
        elif key == ' ':
            self.replay()
            return True
        elif k == 'f':
            self.linear_x = 0.0
            self.angular_z = 0.0
            self.publish_cmd_vel()
            status = self.start_marker_seek()
            if status == 'started':
                self.record_action(marker_action='start')
                self._begin_recorded_marker_seek_segment()
            return True
        elif k == 'q' or key == '\x03':
            return False

        if state_changed:
            if k in ('w', 's', 'a', 'd', 'x'):
                self.publish_cmd_vel()
            elif k in ('c', 'v'):
                self.publish_servo()
            hook_action = ''
            if k == 'v':
                hook_action = 'engage'
            elif k == 'c':
                hook_action = 'disengage'
            self.record_action(marker_action=marker_action, hook_action=hook_action)

        return True


def main(args=None):
    rclpy.init(args=args)
    node = CombinedTeleopNode()
    settings = termios.tcgetattr(sys.stdin)

    print(USAGE_MSG)

    try:
        while True:
            key = get_key(settings)
            if key == '':
                rclpy.spin_once(node, timeout_sec=0.01)
                continue
            result = node.handle_key(key)
            if result is False:
                break
            rclpy.spin_once(node, timeout_sec=0.01)
    except Exception as e:
        print(f'Error: {e}')
    finally:
        # Send stop on exit
        node.linear_x = 0.0
        node.angular_z = 0.0
        node.publish_cmd_vel()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
