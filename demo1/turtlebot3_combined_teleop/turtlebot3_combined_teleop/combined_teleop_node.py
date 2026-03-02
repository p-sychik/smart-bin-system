import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.clock import Clock
from rclpy.node import Node
from rclpy.qos import QoSProfile

from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float32MultiArray, String
from std_srvs.srv import Trigger

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
    C : Center both servos (0.0, 0.0)
    V : V-position (1.0, -1.0)

Recording:
    O : Start recording
    P : Stop recording
    SPACE : Replay recorded actions

Marker Seek:
    F : Start marker seek

    T : Run test sequence

    Q : Quit
"""

# (delta_t, linear_x, angular_z, servo_left, servo_right[, marker_action])
# marker_action can be: '', 'start', 'stop'
TEST_SEQUENCE = [
    (0.0,  0.2,  0.0,  0.0,  0.0),   # Forward
    (2.0,  0.0,  0.5,  0.0,  0.0),   # Turn left
    (2.0,  0.2,  0.0,  1.0, -1.0),   # Forward + servos V
    (2.0,  0.0,  0.0,  0.0,  0.0),   # Stop + servos center
    (2.0, -0.2,  0.0,  0.0,  0.0),   # Backward
    (2.0,  0.0,  0.0,  0.0,  0.0),   # Stop
]


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

    def _normalize_recorded_action(self, action):
        if len(action) == 5:
            dt, lin_x, ang_z, s_left, s_right = action
            marker_action = ''
        elif len(action) == 6:
            dt, lin_x, ang_z, s_left, s_right, marker_action_raw = action
            if isinstance(marker_action_raw, bool):
                marker_action = 'start' if marker_action_raw else ''
            elif isinstance(marker_action_raw, str):
                marker_action = marker_action_raw.strip().lower()
            else:
                marker_action = ''
        else:
            raise ValueError(f'Unsupported recorded action format: {action}')
        if marker_action not in {'', 'start', 'stop'}:
            marker_action = ''
        return dt, lin_x, ang_z, s_left, s_right, marker_action

    def record_action(self, marker_action=''):
        if not self.recording:
            return
        now = time.monotonic()
        if len(self.recorded_actions) == 0:
            delta = 0.0
        else:
            delta = now - self.last_record_time
        self.recorded_actions.append((
            delta,
            self.linear_x,
            self.angular_z,
            self.servo_left,
            self.servo_right,
            marker_action if marker_action in {'start', 'stop'} else '',
        ))
        self.last_record_time = now

    def replay(self):
        count = len(self.recorded_actions)
        if count == 0:
            print('No recorded actions to replay.')
            return
        print(f'Replaying {count} actions...')
        for i, action in enumerate(self.recorded_actions):
            dt, lin_x, ang_z, s_left, s_right, marker_action = (
                self._normalize_recorded_action(action)
            )
            if dt > 0.0:
                end_time = time.monotonic() + dt
                while time.monotonic() < end_time:
                    self.publish_all()
                    rclpy.spin_once(self, timeout_sec=0.01)
                    time.sleep(0.05)
            self.linear_x = lin_x
            self.angular_z = ang_z
            self.servo_left = s_left
            self.servo_right = s_right
            self.publish_all()
            if marker_action == 'start':
                self.start_marker_seek()
            elif marker_action == 'stop':
                self.stop_marker_seek()
            rclpy.spin_once(self, timeout_sec=0.01)
            print(f'  [{i + 1}/{count}] lin={lin_x:.1f} ang={ang_z:.1f}'
                  f' servo=({s_left:.1f}, {s_right:.1f})'
                  f' marker_action={marker_action or "none"}')
        # Stop after replay
        self.linear_x = 0.0
        self.angular_z = 0.0
        self.publish_cmd_vel()
        print('Replay complete. Robot stopped.')

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

    def start_marker_seek(self):
        success, message = self._call_trigger_service(
            self.marker_seek_start_client, '/marker_seek/start')
        if success:
            self.marker_seek_running = True
            print('Marker seek started.')
            return

        if 'Cannot start while in' in message:
            self.marker_seek_running = True
            print('Marker seek is already running.')
            return
        print(f'Failed to start marker seek: {message}')

    def stop_marker_seek(self):
        success, message = self._call_trigger_service(
            self.marker_seek_stop_client, '/marker_seek/stop')
        if success:
            self.marker_seek_running = False
            print('Marker seek stopped.')
            return
        print(f'Failed to stop marker seek: {message}')

    def _marker_seek_event_callback(self, msg):
        event = msg.data.strip().upper()
        if not event:
            return
        if event == 'SUCCEEDED':
            self.marker_seek_running = False
            print('\nMarker seek SUCCEEDED.')
            return
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
            self.stop_marker_seek()
            marker_action = 'stop'
            state_changed = True
            print('Stop')
        elif k == 'c':
            self.servo_left = 0.0
            self.servo_right = 0.0
            state_changed = True
            print('Servos centered')
        elif k == 'v':
            self.servo_left = 0.7
            self.servo_right = -0.7
            state_changed = True
            print('Servos V-position')
        elif k == 'o':
            self.recording = True
            self.recorded_actions = []
            self.last_record_time = 0.0
            print('Recording started')
        elif k == 'p':
            self.recording = False
            print(f'Recording stopped ({len(self.recorded_actions)} actions):')
            for i, action in enumerate(self.recorded_actions):
                dt, lin_x, ang_z, s_left, s_right, recorded_marker_action = (
                    self._normalize_recorded_action(action)
                )
                print(
                    f'  ({dt:.2f}, {lin_x:.1f}, {ang_z:.1f}, '
                    f'{s_left:.1f}, {s_right:.1f}, '
                    f'{recorded_marker_action or "none"}),'
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
            self.start_marker_seek()
            self.record_action(marker_action='start')
            return True
        elif k == 'q' or key == '\x03':
            return False

        if state_changed:
            if k in ('w', 's', 'a', 'd', 'x'):
                self.publish_cmd_vel()
            elif k in ('c', 'v'):
                self.publish_servo()
            self.record_action(marker_action=marker_action)

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
