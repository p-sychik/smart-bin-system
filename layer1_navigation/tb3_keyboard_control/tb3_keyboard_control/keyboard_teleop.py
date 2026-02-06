#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import sys
import termios
import tty


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

        # Publisher using TwistStamped
        self.publisher_ = self.create_publisher(
            TwistStamped,
            '/cmd_vel',
            10
        )

        self.linear_speed = 0.2
        self.angular_speed = 0.5

        self.get_logger().info(
            '\nKeyboard Teleop (TwistStamped)\n'
            '-----------------------------\n'
            'W: forward\n'
            'S: backward\n'
            'A: turn left\n'
            'D: turn right\n'
            'X: stop\n'
            'Q: quit\n'
        )

    def publish_cmd(self, lin_x, ang_z):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        msg.twist.linear.x = lin_x
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0

        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = ang_z

        self.publisher_.publish(msg)

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
            elif key == 'q':
                self.get_logger().info('Exiting keyboard teleop')
                break
            else:
                # Stop on unknown key
                self.publish_cmd(0.0, 0.0)

        # Stop robot on exit
        self.publish_cmd(0.0, 0.0)


def main():
    rclpy.init()
    node = KeyboardTeleop()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

