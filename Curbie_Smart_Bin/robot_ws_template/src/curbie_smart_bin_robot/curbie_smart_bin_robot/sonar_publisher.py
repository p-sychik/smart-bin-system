"""Publish the five Grove ultrasonic sensor readings for the DICE UI and replay logic."""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

try:
    from grove.grove_ultrasonic_ranger import GroveUltrasonicRanger
    SONAR_AVAILABLE = True
except ImportError:
    GroveUltrasonicRanger = None
    SONAR_AVAILABLE = False


class SonarPublisher(Node):
    """Publish the five sonar distances as meters."""

    DEFAULT_PINS = [5, 16, 18, 22, 24]

    def __init__(self) -> None:
        super().__init__('curbie_smart_bin_sonar_publisher')
        self.declare_parameter('pins', self.DEFAULT_PINS)
        self.declare_parameter('publish_rate_hz', 5.0)
        self.declare_parameter('topic_name', '/curbie/sonar_distances')

        pins = [int(pin) for pin in self.get_parameter('pins').value]
        self._sensors = []
        self._labels = []
        if SONAR_AVAILABLE:
            for index, pin in enumerate(pins, start=1):
                try:
                    self._sensors.append(GroveUltrasonicRanger(pin))
                    self._labels.append(f'sonar_{index}(pin={pin})')
                except Exception as exc:
                    self.get_logger().warn(f'Failed to initialize sonar on pin {pin}: {exc}')
        else:
            self.get_logger().warn('grove.grove_ultrasonic_ranger not available; publishing zeros.')

        self.publisher = self.create_publisher(
            Float32MultiArray,
            str(self.get_parameter('topic_name').value),
            10,
        )
        hz = max(1.0, float(self.get_parameter('publish_rate_hz').value))
        self.timer = self.create_timer(1.0 / hz, self._publish_once)
        self.get_logger().info('Sonar publisher ready.')

    def _publish_once(self) -> None:
        distances_m = []
        for sensor in self._sensors:
            try:
                distances_m.append(max(0.0, float(sensor.get_distance())) / 100.0)
            except Exception:
                distances_m.append(0.0)

        if not distances_m:
            distances_m = [0.0] * 5

        msg = Float32MultiArray()
        msg.data = distances_m
        self.publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SonarPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
