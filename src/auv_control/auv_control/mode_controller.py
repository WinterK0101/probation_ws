import rclpy
from rclpy.node import Node
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import State
from std_msgs.msg import String

class ModeController(Node):
    def __init__(self):
        super().__init__('mode_controller')
        
        self.declare_parameter('initial_mode', 'GUIDED')

        # Service client for mode changes
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        
        # State subscriber
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_callback, 10)
            
        self.current_state = None
        self.mode_set = False
        
        # Timer to attempt mode change
        self.timer = self.create_timer(1.0, self.check_and_set_mode)
        
        self.get_logger().info('Mode Controller Node started')

    def state_callback(self, msg):
        self.current_state = msg
        
        # Only log mode changes when not in GUIDED mode
        if not self.mode_set:
            self.get_logger().info(f'Current mode: {msg.mode}')

    def check_and_set_mode(self):
        if self.current_state and not self.mode_set:
            if self.current_state.mode != 'GUIDED':
                self.set_guided_mode()
            else:
                self.get_logger().info('Already in GUIDED mode')
                self.mode_set = True

    def set_guided_mode(self):
        if not self.mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Mode service not available, retrying...')
            return
            
        request = SetMode.Request()
        request.custom_mode = 'GUIDED'
        
        future = self.mode_client.call_async(request)
        future.add_done_callback(self.mode_change_callback)

    def mode_change_callback(self, future):
        try:
            response = future.result()
            if response.mode_sent:
                self.get_logger().info('Successfully requested GUIDED mode')
                self.mode_set = True
            else:
                self.get_logger().error('Failed to set GUIDED mode')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ModeController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()