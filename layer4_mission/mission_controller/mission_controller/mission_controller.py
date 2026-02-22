import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import TwistStamped
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

    # Update hook engagement status
    def hook_status_callback(self, msg):
        self.hook_engaged = (msg.state == 'ENGAGED')