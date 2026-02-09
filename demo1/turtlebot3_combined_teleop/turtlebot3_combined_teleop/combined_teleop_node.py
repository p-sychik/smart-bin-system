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
from std_msgs.msg import Float32MultiArray

USAGE_MSG = """
Combined TurtleBot3 Teleop
--------------------------
Movement:
    W : Forward (0.2 m/s)
    S : Backward (-0.2 m/s)
    A : Turn left (0.5 rad/s)
    D : Turn right (-0.5 rad/s)
    X : Stop movement

Servos:
    C : Center both servos (0.0, 0.0)
    V : V-position (1.0, -1.0)

Recording:
    O : Start recording
    P : Stop recording
    SPACE : Replay recorded actions

    T : Run test sequence

    Q : Quit
"""

# (delta_t, linear_x, angular_z, servo_left, servo_right)
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

    def record_action(self):
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
        ))
        self.last_record_time = now

    def replay(self):
        count = len(self.recorded_actions)
        if count == 0:
            print('No recorded actions to replay.')
            return
        print(f'Replaying {count} actions...')
        for i, (dt, lin_x, ang_z, s_left, s_right) in enumerate(
                self.recorded_actions):
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
            rclpy.spin_once(self, timeout_sec=0.01)
            print(f'  [{i + 1}/{count}] lin={lin_x:.1f} ang={ang_z:.1f}'
                  f' servo=({s_left:.1f}, {s_right:.1f})')
        # Stop after replay
        self.linear_x = 0.0
        self.angular_z = 0.0
        self.publish_cmd_vel()
        print('Replay complete. Robot stopped.')

    def handle_key(self, key):
        k = key.lower()
        state_changed = False

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
            state_changed = True
            print('Stop')
        elif k == 'c':
            self.servo_left = 0.0
            self.servo_right = 0.0
            state_changed = True
            print('Servos centered')
        elif k == 'v':
            self.servo_left = 0.5
            self.servo_right = -0.5
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
            for i, (dt, lin_x, ang_z, s_left, s_right) in enumerate(
                    self.recorded_actions):
                print(f'  ({dt:.2f}, {lin_x:.1f}, {ang_z:.1f}, {s_left:.1f}, {s_right:.1f}),')

        elif k == 't':
            print('Loading test sequence...')
            self.recorded_actions = list(TEST_SEQUENCE)
            self.replay()
            return True
        elif key == ' ':
            self.replay()
            return True
        elif k == 'q' or key == '\x03':
            return False

        if state_changed:
            if k in ('w', 's', 'a', 'd', 'x'):
                self.publish_cmd_vel()
            elif k in ('c', 'v'):
                self.publish_servo()
            self.record_action()

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
