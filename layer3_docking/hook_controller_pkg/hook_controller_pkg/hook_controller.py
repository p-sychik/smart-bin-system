import rclpy
from rclpy.node import Node
from bin_collection_msgs.msg import HookStatus, AlignmentStatus
from bin_collection_msgs.srv import EngageHook, DisengageHook


class HookController(Node):
    """
    Hook controller that checks alignment before engaging.
    """

    def __init__(self):
        super().__init__('hook_controller')

        self.state = 'DISENGAGED'
        self.bin_attached = False
        self.hook_position = 'DOWN'

        # Alignment state from ultrasonic aligner.
        self.is_aligned = False

        self.alignment_sub = self.create_subscription(
            AlignmentStatus,
            '/alignment/status',
            self.alignment_callback,
            10
        )

        # Publisher
        self.status_pub = self.create_publisher(HookStatus, '/hook/status', 10)

        # Services
        self.engage_srv = self.create_service(EngageHook, '/hook/engage', self.engage_callback)
        self.disengage_srv = self.create_service(DisengageHook, '/hook/disengage', self.disengage_callback)

        self.timer = self.create_timer(0.5, self.publish_status)

        self.get_logger().info('Hook Controller ready - waiting for alignment data')

    def alignment_callback(self, msg):
        """Receive alignment status from ultrasonic aligner."""
        self.is_aligned = msg.aligned

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

        if not self.is_aligned:
            response.success = False
            response.message = 'Cannot engage - not aligned'
            response.status = self.get_status_msg()
            self.get_logger().warn('Engage rejected - not aligned')
            return response

        self.hook_position = 'UP'
        self.state = 'ENGAGED'
        self.bin_attached = True

        response.success = True
        response.message = 'Hook engaged'
        response.status = self.get_status_msg()

        self.get_logger().info('Hook engaged')
        return response

    def disengage_callback(self, request, response):
        self.get_logger().info('Disengage requested')

        if self.state == 'DISENGAGED':
            response.success = False
            response.message = 'Already disengaged'
            response.status = self.get_status_msg()
            return response

        self.hook_position = 'DOWN'
        self.state = 'DISENGAGED'
        self.bin_attached = False

        response.success = True
        response.message = 'Hook disengaged'
        response.status = self.get_status_msg()

        self.get_logger().info('Hook disengaged')
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
