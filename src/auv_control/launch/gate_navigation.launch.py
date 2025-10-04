from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # ROS-TCP-Endpoint for Unity communication
        Node(
            package='ros_tcp_endpoint',
            executable='default_server_endpoint',
            name='unity_endpoint',
            output='screen',
            emulate_tty=True,
        ),
        
        # AUV Control Nodes
        Node(
            package='auv_control',
            executable='mode_controller',
            name='mode_controller',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='auv_control',
            executable='depth_controller',
            name='depth_controller',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='auv_control',
            executable='gate_controller',
            name='gate_controller',
            output='screen',
            emulate_tty=True,
        )
    ])