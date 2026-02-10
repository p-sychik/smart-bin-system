import rclpy
from rclpy.node import Node
from bin_collection_msgs.msg import HookStatus
from bin_collection_msgs.srv import EngageHook, DisengageHook
from std_msgs.msg import Float32MultiArray


class HookController(Node):
    """
    Handles hooking of the L-shaped rods.
    Publishes to /servo_cmd topic (same as teleop).
    """

    def __init__(self):
        super().__init__('hook_controller')

        # State
        self.state = 'DISENGAGED'
        self.bin_attached = False

        # Servo positions: Adjust these based on hardware testing.
        # Format is [left_servo, right_servo]
        self.UNLOCKED_POS = [0.0, 0.0]      # Centered
        self.LOCKED_POS = [0.5, -0.5]       # V-position (locks the hooks)

        # Publisher to servo_cmd topic
        self.servo_pub = self.create_publisher(Float32MultiArray, 'servo_cmd', 10)

        # Publisher
        self.status_pub = self.create_publisher(HookStatus, '/hook/status', 10)

        # Services
        self.engage_srv = self.create_service(EngageHook, '/hook/engage', self.engage_callback)
        self.disengage_srv = self.create_service(DisengageHook, '/hook/disengage', self.disengage_callback)

        # Publish status
        self.timer = self.create_timer(0.5, self.publish_status)

        self.get_logger().info('Hook Controller ready')

    def send_servo_command(self, positions):
        """Send servo positions to /servo_cmd topic."""
        msg = Float32MultiArray()
        msg.data = positions
        self.servo_pub.publish(msg)
        self.get_logger().info(f'Sent servo command: {positions}')
        return True

    def publish_status(self):
        msg = HookStatus()
        msg.state = self.state
        msg.bin_attached = self.bin_attached
        msg.error_message = ''
        self.status_pub.publish(msg)

    def engage_callback(self, request, response):
        self.get_logger().info('Engage requested')

        if self.state == 'ENGAGED':
            response.success = False
            response.message = 'Already engaged'
            response.status = self.get_status_msg()
            return response

        self.send_servo_command(self.LOCKED_POS)
        self.state = 'ENGAGED'
        self.bin_attached = True
        response.success = True
        response.message = 'Hook engaged'
        self.get_logger().info('Hook engaged')

        response.status = self.get_status_msg()
        return response

    def disengage_callback(self, request, response):
        self.get_logger().info('Disengage requested')

        if self.state == 'DISENGAGED':
            response.success = False
            response.message = 'Already disengaged'
            response.status = self.get_status_msg()
            return response

        self.send_servo_command(self.UNLOCKED_POS)
        self.state = 'DISENGAGED'
        self.bin_attached = False
        response.success = True
        response.message = 'Hook disengaged'
        self.get_logger().info('Hook disengaged')

        response.status = self.get_status_msg()
        return response

    def get_status_msg(self):
        msg = HookStatus()
        msg.state = self.state
        msg.bin_attached = self.bin_attached
        msg.error_message = ''
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = HookController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
