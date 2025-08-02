from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_ros_bridge',
            output='screen',
            parameters=[{
                'config_file': '/home/aero360/ros2_ws/src/slam/config/bridge.yaml'
            }]
        ),
        
    ])