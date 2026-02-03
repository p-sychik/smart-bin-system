import rclpy
from rclpy.node import Node
from bin_collection_msgs.msg import UltrasonicArray


class UltrasonicSimulator(Node):
    """
    Fake ultrasonic readings for testing.
    Change distances to test alignment logic.
    Purely for testing purposes.
    """

    def __init__(self):
        super().__init__('ultrasonic_simulator')

        # Fake distances - equal means aligned
        self.left_distance = 0.20
        self.right_distance = 0.20

        self.publisher = self.create_publisher(UltrasonicArray, '/ultrasonic/raw', 10)
        self.timer = self.create_timer(0.1, self.publish_readings)

        self.get_logger().info(f'Ultrasonic Simulator: L={self.left_distance}, R={self.right_distance}')

    def publish_readings(self):
        msg = UltrasonicArray()
        msg.distances = [self.left_distance, self.right_distance, 0.0, 0.0]
        msg.valid = [True, True, False, False]
        msg.timestamp = self.get_clock().now().to_msg()
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
