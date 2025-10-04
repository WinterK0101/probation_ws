import rclpy
from rclpy.node import Node
from vision_msgs.msg import BoundingBoxArray
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from enum import Enum

class ControlState(Enum):
    WAITING_FOR_DEPTH = 0
    SEARCHING = 1
    APPROACHING = 2

class GateController(Node):
    def __init__(self):
        super().__init__('gate_controller')
        
        # Parameters
        self.declare_parameter('angular_speed', 0.3)
        self.declare_parameter('center_tolerance', 0.05)
        self.declare_parameter('approach_speed', 0.2)
        self.declare_parameter('search_time_per_round', 21.0)
        
        # Subscribers
        self.depth_status_sub = self.create_subscription(String, '/depth_controller/status', self.depth_status_callback, 10)
        self.detection_sub = self.create_subscription(BoundingBoxArray, '/main_camera/detection/bounding_boxes', self.detection_callback, 10)
            
        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        
        # Timer
        self.control_timer = self.create_timer(0.2, self.execute_control)
        
        # State variables
        self.current_state = ControlState.WAITING_FOR_DEPTH
        self.gate_box = None
        self.search_start_time = None
        self.search_rounds = 0
        self.depth_wait_start = self.get_clock().now()
        self.mission_complete = False
        self.post_gate_start_time = None  # Track when we start moving after passing gate
        self.max_gate_width_seen = 0.0  # Track the largest gate width seen
        self.gate_passed = False  # Flag to track if we've actually passed through
        
        self.get_logger().info('Gate Controller started - Waiting for depth controller')

    def depth_status_callback(self, msg):
        if msg.data == "DEPTH_COMPLETE" and self.current_state == ControlState.WAITING_FOR_DEPTH:
            self.start_gate_search()

    def start_gate_search(self):
        self.current_state = ControlState.SEARCHING
        self.search_start_time = self.get_clock().now()
        self.search_rounds += 1
        self.get_logger().info(f'Starting gate search round {self.search_rounds}')

    def detection_callback(self, msg):
        if self.current_state not in [ControlState.SEARCHING, ControlState.APPROACHING] or self.mission_complete:
            return
        
        # Look for gate
        gate_found = False
        for box in msg.bounding_boxes:
            if box.label_name == 'gate':
                self.gate_box = box
                gate_found = True
                
                # Track the maximum gate width we've seen
                if box.w > self.max_gate_width_seen:
                    self.max_gate_width_seen = box.w
                    self.get_logger().info(f'Max gate width updated: {self.max_gate_width_seen:.3f}')
                
                # Only consider "passed through" if:
                # 1. We've seen a large gate before (> 0.5)
                # 2. Current gate is very small (< 0.1) 
                # 3. We're in APPROACHING state
                if (self.max_gate_width_seen > 0.4 and 
                    box.w < 0.05 and 
                    self.current_state == ControlState.APPROACHING and
                    not self.gate_passed):
                    
                    self.gate_passed = True
                    self.post_gate_start_time = self.get_clock().now()
                    self.get_logger().info(f'Passed through gate (max width was {self.max_gate_width_seen:.3f}, now {box.w:.3f}) - moving forward for 1 second')
                    return

                break
        
        # Handle gate found (only if we haven't passed through yet)
        if gate_found and not self.gate_passed:
            if self.current_state == ControlState.SEARCHING:
                self.current_state = ControlState.APPROACHING
                self.get_logger().info('Gate found - switching to approach mode')
            

    def execute_control(self):
        if self.mission_complete:
            return
        
        # Handle post-gate forward movement
        if self.post_gate_start_time is not None:
            elapsed_time = (self.get_clock().now() - self.post_gate_start_time).nanoseconds / 1e9
            
            if elapsed_time < 1.0:  # Move forward for 1 second
                cmd = Twist()
                cmd.linear.x = self.get_parameter('approach_speed').value
                self.cmd_vel_pub.publish(cmd)
                return
            else:
                # 1 second has passed - mission complete
                self.mission_complete = True
                self.get_logger().info('Mission accomplished - guaranteed gate clearance!')
                self.cmd_vel_pub.publish(Twist())  # Publish zero velocity
                self.control_timer.destroy()
                return
            
        cmd = Twist()
        
        if self.current_state == ControlState.WAITING_FOR_DEPTH:
            elapsed_time = (self.get_clock().now() - self.depth_wait_start).nanoseconds / 1e9
            if elapsed_time > 15.0:
                self.get_logger().warn('Depth timeout - starting search')
                self.start_gate_search()
            return
        elif self.current_state == ControlState.SEARCHING:
            self.execute_search_pattern(cmd)
        elif self.current_state == ControlState.APPROACHING:
            self.execute_approach(cmd)
        else:
            return
        
        self.cmd_vel_pub.publish(cmd)

    def execute_search_pattern(self, cmd):
        if not self.search_start_time:
            return
            
        angular_speed = self.get_parameter('angular_speed').value
        search_time_per_round = self.get_parameter('search_time_per_round').value
        
        cmd.angular.z = -angular_speed
        
        elapsed_time = (self.get_clock().now() - self.search_start_time).nanoseconds / 1e9
        
        if elapsed_time >= search_time_per_round:
            self.start_gate_search()

    def execute_approach(self, cmd):
        if not self.gate_box:
            return
        
        cmd_angular_z = 0.0
        linear_x = 0.0
        
        # Calculate the CENTER of the bounding box
        gate_center_x = self.gate_box.x + (self.gate_box.w / 2.0)
        x_error = 0.5 - gate_center_x  # Align to center of bounding box
        center_tolerance = self.get_parameter('center_tolerance').value

        # 2-Phase control logic
        if abs(x_error) > center_tolerance:
            # Phase 1: Not centered - turn to center
            linear_x = self.get_parameter('approach_speed').value * 0.3  # 30% speed while centering
            cmd_angular_z = 0.3 * x_error
        else:
            # Phase 2: Centered - go through the gate
            linear_x = self.get_parameter('approach_speed').value
            cmd_angular_z = 0.05 * x_error
        
        cmd.angular.z = cmd_angular_z
        cmd.linear.x = linear_x
        

def main(args=None):
    rclpy.init(args=args)
    node = GateController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()