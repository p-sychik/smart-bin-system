import rclpy
from rclpy.node import Node
from bin_collection_msgs.msg import HookStatus
from bin_collection_msgs.srv import EngageHook, DisengageHook
import serial


class HookController(Node):
    """
    Handles hooking of the L-shaped rods.
    """

    def __init__(self):
        super().__init__('hook_controller')

        # State
        self.state = 'DISENGAGED'
        self.bin_attached = False

        # Serial connection to Pico: You can change the /dev/ttyX if port is
	# different. Try ls /dev/tty* before and after plugging in Pico
        try:
            self.pico = serial.Serial('/dev/ttyUSB1', 9600, timeout=1)
            self.get_logger().info('Connected to Pico on /dev/ttyUSB1')
        except Exception as e:
            self.pico = None
            self.get_logger().error(f'Could not connect to Pico: {e}')

        # Publisher
        self.status_pub = self.create_publisher(HookStatus, '/hook/status', 10)

        # Services
        self.engage_srv = self.create_service(EngageHook, '/hook/engage', self.engage_callback)
        self.disengage_srv = self.create_service(DisengageHook, '/hook/disengage', self.disengage_callback)

        # Publish status
        self.timer = self.create_timer(0.5, self.publish_status)

        self.get_logger().info('Hook Controller ready')

    def send_command(self, cmd):
        """Send command to Pico."""
        if self.pico is None:
            self.get_logger().error('Pico not connected')
            return False

        try:
            self.pico.write(f'{cmd}\n'.encode())
            response = self.pico.readline().decode().strip()
            self.get_logger().info(f'Pico response: {response}')
            return True
        except Exception as e:
            self.get_logger().error(f'Serial error: {e}')
            return False

    def publish_status(self):
        msg = HookStatus()
        msg.state = self.state
        msg.bin_attached = self.bin_attached
        msg.error_message = '' if self.pico else 'Pico not connected'
        self.status_pub.publish(msg)

    def engage_callback(self, request, response):
        self.get_logger().info('Engage requested')

        if self.pico is None:
            response.success = False
            response.message = 'Pico not connected'
            response.status = self.get_status_msg()
            return response

        if self.state == 'ENGAGED':
            response.success = False
            response.message = 'Already engaged'
            response.status = self.get_status_msg()
            return response

        if self.send_command('LOCK'):
            self.state = 'ENGAGED'
            self.bin_attached = True
            response.success = True
            response.message = 'Hook engaged'
            self.get_logger().info('Hook engaged')
        else:
            response.success = False
            response.message = 'Failed to lock'

        response.status = self.get_status_msg()
        return response

    def disengage_callback(self, request, response):
        self.get_logger().info('Disengage requested')

        if self.pico is None:
            response.success = False
            response.message = 'Pico not connected'
            response.status = self.get_status_msg()
            return response

        if self.state == 'DISENGAGED':
            response.success = False
            response.message = 'Already disengaged'
            response.status = self.get_status_msg()
            return response

        if self.send_command('UNLOCK'):
            self.state = 'DISENGAGED'
            self.bin_attached = False
            response.success = True
            response.message = 'Hook disengaged'
            self.get_logger().info('Hook disengaged')
        else:
            response.success = False
            response.message = 'Failed to unlock'

        response.status = self.get_status_msg()
        return response

    def get_status_msg(self):
        msg = HookStatus()
        msg.state = self.state
        msg.bin_attached = self.bin_attached
        msg.error_message = '' if self.pico else 'Pico not connected'
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = HookController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
