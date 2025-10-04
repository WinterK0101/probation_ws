# AUV Gate Navigation Controller

## Link to video (Google Drive)
Click this [link](https://drive.google.com/drive/folders/1XyPJLvkpLEys5zHfpoF10yYmM_OsLU12?usp=sharing)

## 📁 Package Structure

```
auv_control/
├── auv_control/
│   ├── __init__.py
│   ├── mode_controller.py      # System initialization and mode management
│   ├── depth_controller.py     # Depth control and stabilization
│   └── gate_controller.py      # Main navigation logic
├── launch/
│   └── gate_navigation.launch.py
├── package.xml
├── setup.py
└── README.md
```

## 🧠 Solution Design & Chain of Thoughts

### Problem Decomposition

The gate navigation challenge was broken down into three sequential phases:

1. **System Preparation**: Ensure AUV is in correct mode and ready for operation
2. **Positioning**: Move to correct depth for consistent navigation and detection
3. **Navigation**: Find and traverse through gates autonomously

## 📋 Controller Details

### 1. Mode Controller (`mode_controller.py`)

**Purpose**: Ensure vehicle is set to GUIDED mode

**Chain of Thought**:
- Vehicle must be in GUIDED mode for autonomous control via topics
- System should retry mode setting if initial attempt fails

**Key Implementation**:
```python
class ModeController(Node):
    def __init__(self):
        # Service client for mode changes
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        
        # Timer-based retry mechanism
        self.timer = self.create_timer(1.0, self.check_and_set_mode)
```

### 2. Depth Controller (`depth_controller.py`)

**Purpose**: Ensure consistent depth positioning for navigation and detection

**Chain of Thought**:
- Gate detection only work when the vehicle is underwater
- Time-based descent is simpler and reliable
- Signal depth reached so that vehicle can start to search for gate

**Key Implementation**:
```python
def start_descent(self):
    self.descent_timer = self.create_timer(0.1, self.descend)
    self.stop_timer = self.create_timer(descent_time, self.stop_descent)

def descend(self):
    cmd = Twist()
    cmd.linear.z = -descent_speed  # Negative for downward
    self.cmd_vel_pub.publish(cmd)
```

### 3. Gate Controller (`gate_controller.py`)

**Purpose**: Core navigation logic for gate detection and traversal

**Chain of Thought**:
- User need to manually press "Tab" for the main camera to be able to detect bounding boxes
- Only start searching for gate after vehicle has reached the target depth
- Once the gate has been found, center to it before navigating to it
- Track the width of the gate so that we can use it to determine when the vehicle has passed the gate 

**Key Design Insights**:

1. **Vision-Based Centering**:
```python
# Calculate gate center from bounding box
gate_center_x = self.gate_box.x + (self.gate_box.w / 2.0)
x_error = 0.5 - gate_center_x  # Screen center is at 0.5
```

**Rationale**: Tried raw bounding box coordinates, but the vehicle kept aligning to gate edges rather than center opening. 

2. **Two-Phase Approach Control**:
```python
if abs(x_error) > center_tolerance:
    # Phase 1: Centering with slow approach
    cmd.angular.z = 0.3 * x_error
    cmd.linear.x = approach_speed * 0.3
else:
    # Phase 2: Centered enough, full speed through
    cmd.angular.z = 0.05 * x_error
    cmd.linear.x = approach_speed
```

**Rationale**: Ensure it is center enough before navigating to it at full speed.

3. **Gate Passed Detection**:
```python
# Track maximum gate width to prevent premature completion
if (self.max_gate_width_seen > 0.4 and 
    box.w < 0.05 and 
    self.current_state == ControlState.APPROACHING):
    self.gate_passed = True
```

**Rationale**: Small gate detections early in approach were causing false "mission complete" signals. 
So only consider "passed through" if:
- We've seen a large gate before (> 0.5)
- Current detected gate is very small (< 0.1) 
- We're in APPROACHING state

4. **Guaranteed Gate Clearance**:
```python
# Continue forward for 1 second after detecting passage
if elapsed_time < 1.0:
    cmd.linear.x = self.get_parameter('approach_speed').value
```

**Rationale**: Let vehicle move 1 more second to ensure it fully passes through the gate before stopping.

## 🚀 Usage

### Build and Launch
```bash
cd ~/probation_ws
colcon build --packages-select auv_control
source install/setup.bash
ros2 launch auv_control gate_navigation.launch.py
```