import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from mavros_msgs.msg import State
from std_msgs.msg import String

class DepthController(Node):
    def __init__(self):
        super().__init__('depth_controller')
        
        # Parameters
        self.declare_parameter('descent_speed', 0.8)  
        self.declare_parameter('descent_time', 5.0)  
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        self.status_pub = self.create_publisher(String, '/depth_controller/status', 10)
            
        # Subscribers
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_callback, 10)
            
        self.descent_started = False
        self.descent_complete = False

    def state_callback(self, msg):
        if msg.mode == 'GUIDED' and not self.descent_started:
            self.start_descent()

    def start_descent(self):
        if self.descent_started:
            return
            
        self.descent_started = True
        self.descent_timer = self.create_timer(0.1, self.descend)
        self.stop_timer = self.create_timer(self.get_parameter('descent_time').value, self.stop_descent)

    def descend(self):
        if self.descent_complete:
            return
            
        cmd = Twist()
        cmd.linear.z = -self.get_parameter('descent_speed').value  # Negative for down
        self.cmd_vel_pub.publish(cmd)

    def stop_descent(self):
        if self.descent_complete:
            return
            
        self.descent_complete = True
        
        # Stop movement
        self.cmd_vel_pub.publish(Twist())  # All zeros
        
        # Clean up timers
        self.descent_timer.destroy()
        self.stop_timer.destroy()
        
        # Start status publishing
        self.create_timer(1.0, self.publish_completion_status)
        self.get_logger().info('Descent complete')

    def publish_completion_status(self):
        status_msg = String()
        status_msg.data = "DEPTH_COMPLETE"
        self.status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DepthController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()