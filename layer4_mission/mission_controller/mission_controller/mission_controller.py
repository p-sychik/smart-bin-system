import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import TwistStamped
from std_srvs.srv import Trigger
from shared_msgs/bin_collection_msgs.srv import EngageHook, DisengageHook
from shared_msgs/bin_collection_msgs.msg import HookStatus
import time

class MissionState:
    """State definitions"""
    IDLE = 'IDLE'
    NAVIGATING_TO_BIN = 'NAVIGATING_TO_BIN'
    SEARCHING_FOR_BIN = 'SEARCHING_FOR_BIN'
    APPROACHING_BIN = 'APPROACHING_BIN'
    DOCKING = 'DOCKING'
    TRANSPORTING = 'TRANSPORTING'
    UNDOCKING = 'UNDOCKING'
    RETURNING_TO_DOCK = 'RETURNING_TO_DOCK'
    SEARCHING_FOR_DOCK = 'SEARCHING_FOR_DOCK'
    ERROR_RECOVERY = 'ERROR_RECOVERY'

class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')

        # State variables
        self.state = MissionState.IDLE
        self.previous_state = None
        self.state_start_time = time.time()
        self.hook_engaged = False
        self.retry_count = 0
        self.max_retries = 3

        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            TwistStamped,
            '/cmd_vel',
            10
        )

        # Subscribers
        self.hook_status_sub = self.create_subscription(
            HookStatus,
            '/hook/status',
            self.hook_status_callback,
            10
        )

        # Service clients
        self.engage_client = self.create_client(
            EngageHook,
            '/hook/engage'
        )
        
        self.disengage_client = self.create_client(
            DisengageHook,
            '/hook/disengage'
        )

        # Service servers
        self.start_mission_srv = self.create_service(
            Trigger,
            '/mission/start',
            self.start_mission_callback
        )
        
        self.abort_mission_srv = self.create_service(
            Trigger,
            '/mission/abort',
            self.abort_mission_callback
        )

    # Update hook engagement status
    def hook_status_callback(self, msg):
        self.hook_engaged = (msg.state == 'ENGAGED')

    # Handle mission start service call
    def start_mission_callback(self, request, response):
        if self.state != MissionState.IDLE:
            response.success = False
            response.message = f'Cannot start mission, currently in {self.state}'
            return response
        
        self.get_logger().info('Mission start requested')
        self.transition_to(MissionState.NAVIGATING_TO_BIN)
        response.success = True
        response.message = 'Mission started'
        return response
    
    # Handle mission abort service call
    def abort_mission_callback(self, request, response):
        self.get_logger().warn('Mission abort requested!')
        self.transition_to(MissionState.ERROR_RECOVERY)
        response.success = True
        response.message = 'Mission aborted, entering error recovery'
        return response
    
    # Handle state transitions
    def transition_to(self, new_state):
        if new_state == self.state:
            return
        
        self.get_logger().info(f'State transition: {self.state} → {new_state}')

        self.exit_state(self.state)

        self.previous_state = self.state
        self.state = new_state
        self.state_start_time = time.time()

        self.enter_state(new_state)

    # Actions to perform when entering a state
    def enter_state(self, state):
        if state == MissionState.DOCKING:
            self.get_logger().info('Entering DOCKING state')
            self.retry_count = 0
        
        elif state == MissionState.UNDOCKING:
            self.get_logger().info('Entering UNDOCKING state')
            self.retry_count = 0
        
        elif state == MissionState.ERROR_RECOVERY:
            self.get_logger().warn('Entering ERROR_RECOVERY state')
            self.stop_robot()

    # Actions to perform when exiting a state
    def exit_state(self, state):
        if state in [MissionState.NAVIGATING_TO_BIN, MissionState.TRANSPORTING]:
            self.stop_robot()

    # Main state machine logic
    def state_machine_update(self):
        time_in_state = time.time() - self.state_start_time

        if self.state == MissionState.IDLE:
            pass

        elif self.state == MissionState.NAVIGATING_TO_BIN:
            # TODO

        elif self.state == MissionState.SEARCHING_FOR_BIN:
            # TODO

        elif self.state == MissionState.APPROACHING_BIN:
            # TODO

        elif self.state == MissionState.DOCKING:
            self.handle_docking_state()

        elif self.state == MissionState.TRANSPORTING:
            # TODO

        elif self.state == MissionState.UNDOCKING:
            self.handle_undocking_state()

        elif self.state == MissionState.RETURNING_TO_DOCK:
            # TODO

        elif self.state == MissionState.SEARCHING_FOR_DOCK:
            # TODO

        elif self.state == MissionState.ERROR_RECOVERY:
            if time_in_state > 2.0:
                self.get_logger().info('Bin not found, return to dock')
                self.transition_to(MissionState.RETURNING_TO_DOCK)

        
    

    # ===========================================
    # Docking and Undocking states implementation
    # ===========================================

    def handle_docking_state(self):
        time_in_state = time.time() - self.state_start_time

        if time_in_state < 0.5:
            return
        
        if time_in_state < 1.0:
            self.get_logger().info('Attempting to engage hook...')
            success = self.call_engage_service()

            if success:
                self.get_logger().info('Hook engaged successfully!')
                self.transition_to(MissionState.TRANSPORTING)

            else:
                self.retry_count += 1
                self.get_logger().warn(f'Hook engagement failed (attempt {self.retry_count}/{self.max_retries})')

                if self.retry_count >= self.max_retries:
                    self.get_logger().error('Max retries reached for hook engagement')
                    # TODO

                else:
                    # TODO

    def call_engage_service(self):
        if not self.engage_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Hook engage service not available')
            return False
        
        request = EngageHook.Request()

        try:
            future = self.engage_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

            if future.done():
                response = future.result()
                if response.success:
                    self.get_logger().info(f'{response.message}')
                    return True
            
                else:
                    self.get_logger().error(f'{response.message}')
                    return False

            else:
                self.get_logger().error('Hook engage service call timed out')
                return False
            
        except Exception as e:
            self.get_logger().error(f'Exception calling engage service: {e}')
            return False
        
    def handle_undocking_state(self):
        time_in_state = time.time() - self.state_start_time

        if time_in_state < 0.5:
            return
        
        if time_in_state < 1.0:
            self.get_logger().info('Attempting to engage hook...')
            success = self.call_disengage_service()

            if success:
                self.get_logger().info('Hook disengaged successfully!')
                self.transition_to(MissionState.RETURNING_TO_DOCK)

            else:
                self.retry_count += 1
                self.get_logger().warn(f'Hook disengagement failed (attempt {self.retry_count}/{self.max_retries})')

                if self.retry_count >= self.max_retries:
                    self.get_logger().error('Max retries reached for hook disengagement')
                    # TODO

                else:
                    # TODO

    def call_disengage_service(self):
        if not self.disengage_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Hook disengage service not available')
            return False
        
        request = DisengageHook.Request()

        try:
            future = self.disengage_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.done():
                response = future.result()
                if response.success:
                    self.get_logger().info(f'{response.message}')
                    return True
                
                else:
                    self.get_logger().error(f'{response.message}')
                    return False
                
            else:
                self.get_logger().error('Hook disengage service call timed out')
                return False
            
        except Exception as e:
            self.get_logger().error(f'Exception calling disengage service: {e}')
            return False
        




def main(args=None):
    rclpy.init(args=args)
    node = MissionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()




        
