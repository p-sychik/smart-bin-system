#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import sys
import termios
import tty
import time


def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key


class KeyboardTeleop(Node):

    def __init__(self):
        super().__init__('keyboard_teleop_twiststamped')

        self.publisher_ = self.create_publisher(
            TwistStamped,
            '/cmd_vel',
            10
        )

        self.linear_speed = 0.2
        self.angular_speed = 0.5

        # Recording state
        self.recording = False
        self.recorded_path = []   # list of (dt, lin_x, ang_z)
        self.last_record_time = None

        self.get_logger().info(
            '\nKeyboard Teleop with Path Record & Replay\n'
            '-----------------------------------------\n'
            'W: forward | S: backward\n'
            'A: left    | D: right\n'
            'X: stop\n'
            'O: start recording\n'
            'P: stop recording\n'
            'SPACE: replay path\n'
            'Q: quit\n'
        )

    def publish_cmd(self, lin_x, ang_z, record=True):
        msg = TwistStamped()
        now = self.get_clock().now()

        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = lin_x
        msg.twist.angular.z = ang_z

        self.publisher_.publish(msg)

        # Record command with timing
        if self.recording and record:
            t = time.time()
            if self.last_record_time is None:
                dt = 0.0
            else:
                dt = t - self.last_record_time

            self.recorded_path.append((dt, lin_x, ang_z))
            self.last_record_time = t

    def replay_path(self):
        if not self.recorded_path:
            self.get_logger().warn('No path recorded!')
            return

        self.get_logger().info('Replaying recorded path...')
        self.publish_cmd(0.0, 0.0, record=False)
        time.sleep(0.5)

        for dt, lin_x, ang_z in self.recorded_path:
            time.sleep(dt)
            self.publish_cmd(lin_x, ang_z, record=False)

        self.publish_cmd(0.0, 0.0, record=False)
        self.get_logger().info('Replay finished')

    def run(self):
        while rclpy.ok():
            key = get_key()

            if key == 'w':
                self.publish_cmd(self.linear_speed, 0.0)
            elif key == 's':
                self.publish_cmd(-self.linear_speed, 0.0)
            elif key == 'a':
                self.publish_cmd(0.0, self.angular_speed)
            elif key == 'd':
                self.publish_cmd(0.0, -self.angular_speed)
            elif key == 'x':
                self.publish_cmd(0.0, 0.0)

            elif key == 'o':
                self.recorded_path.clear()
                self.recording = True
                self.last_record_time = None
                self.get_logger().info('Recording started')

            elif key == 'p':
                self.recording = False
                self.last_record_time = None
                self.get_logger().info(
                    f'Recording stopped ({len(self.recorded_path)} commands)'
                )

            elif key == ' ':
                self.recording = False
                self.last_record_time = None
                self.replay_path()

            elif key == 'q':
                self.get_logger().info('Exiting')
                break

            else:
                self.publish_cmd(0.0, 0.0)

        self.publish_cmd(0.0, 0.0)


def main():
    rclpy.init()
    node = KeyboardTeleop()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
