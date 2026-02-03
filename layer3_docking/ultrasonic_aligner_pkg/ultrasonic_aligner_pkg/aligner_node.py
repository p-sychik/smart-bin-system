import rclpy
from rclpy.node import Node
from bin_collection_msgs.msg import UltrasonicArray, AlignmentStatus


class UltrasonicAligner(Node):
    """
    Processes ultrasonic sensor data to determine alignment.
    """

    def __init__(self):
        super().__init__('ultrasonic_aligner')

        self.alignment_tolerance = 0.02
        self.max_range = 0.5
        self.min_range = 0.05

        self.left_distance = None
        self.right_distance = None

        self.ultrasonic_sub = self.create_subscription(
            UltrasonicArray,
            '/ultrasonic/raw',
            self.ultrasonic_callback,
            10
        )

        self.alignment_pub = self.create_publisher(AlignmentStatus, '/alignment/status', 10)
        self.timer = self.create_timer(0.1, self.publish_alignment)

        self.get_logger().info('Ultrasonic Aligner ready')

    def ultrasonic_callback(self, msg):
        if len(msg.distances) >= 2:
            self.left_distance = msg.distances[0]
            self.right_distance = msg.distances[1]

    def publish_alignment(self):
        msg = AlignmentStatus()

        # Set all fields explicitly with correct types
        msg.surface_detected = False
        msg.hole_detected = False
        msg.aligned = False
        msg.x_offset = 0.0
        msg.y_offset = 0.0
        msg.distance_to_surface = 0.0
        msg.state = 'NO_DATA'

        if self.left_distance is None or self.right_distance is None:
            self.alignment_pub.publish(msg)
            return

        # Check valid range
        left_valid = self.min_range < self.left_distance < self.max_range
        right_valid = self.min_range < self.right_distance < self.max_range

        # Must cast to bool explicitly
        msg.surface_detected = bool(left_valid and right_valid)
        msg.x_offset = float(self.left_distance - self.right_distance)
        msg.distance_to_surface = float((self.left_distance + self.right_distance) / 2)

        if msg.surface_detected and abs(msg.x_offset) < self.alignment_tolerance:
            msg.aligned = True
            msg.state = 'ALIGNED'
        else:
            msg.aligned = False
            msg.state = 'NOT_ALIGNED'

        msg.hole_detected = False
        msg.y_offset = 0.0

        self.alignment_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicAligner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
