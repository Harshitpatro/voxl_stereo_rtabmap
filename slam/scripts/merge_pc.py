#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


class QVIOFrameFixer(Node):
    def __init__(self):
        super().__init__('qvio_frame_fixer')
        
        # Declare parameters for frame IDs
        self.declare_parameter('input_topic', '/qvio')
        self.declare_parameter('output_topic', '/qvio_fixed')
        self.declare_parameter('target_frame_id', 'world')
        self.declare_parameter('child_frame_id', 'base_link')
        
        # Get parameters
        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.target_frame_id = self.get_parameter('target_frame_id').get_parameter_value().string_value
        self.child_frame_id = self.get_parameter('child_frame_id').get_parameter_value().string_value
        
        # Configure QoS to match common QVIO publisher settings
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Try to determine message type and create appropriate subscriber/publisher
        self.subscription = None
        self.publisher = None
        self.message_type = None
        
        # Try to create subscription and publisher
        if not self.try_setup_odom_relay():
            self.try_setup_pose_relay()
        
        self.get_logger().info(f'QVIO Frame Fixer started:')
        self.get_logger().info(f'  Input: {self.input_topic}')
        self.get_logger().info(f'  Output: {self.output_topic}')
        self.get_logger().info(f'  Frame ID: {self.target_frame_id}')
        self.get_logger().info(f'  Child Frame ID: {self.child_frame_id}')
        self.get_logger().info(f'  Message Type: {self.message_type}')

    def try_setup_odom_relay(self):
        """Try to set up Odometry relay"""
        try:
            # Create subscriber for input
            self.subscription = self.create_subscription(
                Odometry,
                self.input_topic,
                self.odom_callback,
                self.qos_profile)
            
            # Create publisher for output
            self.publisher = self.create_publisher(
                Odometry,
                self.output_topic,
                self.qos_profile)
            
            self.message_type = 'Odometry'
            self.get_logger().info('Successfully set up Odometry relay')
            return True
        except Exception as e:
            self.get_logger().warn(f'Failed to set up Odometry relay: {str(e)}')
            return False

    def try_setup_pose_relay(self):
        """Try to set up PoseStamped relay"""
        try:
            # Create subscriber for input
            self.subscription = self.create_subscription(
                PoseStamped,
                self.input_topic,
                self.pose_callback,
                self.qos_profile)
            
            # Create publisher for output
            self.publisher = self.create_publisher(
                PoseStamped,
                self.output_topic,
                self.qos_profile)
            
            self.message_type = 'PoseStamped'
            self.get_logger().info('Successfully set up PoseStamped relay')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to set up PoseStamped relay: {str(e)}')
            return False

    def odom_callback(self, msg):
        """Handle Odometry messages and fix frame IDs"""
        try:
            # Create new message with corrected frame IDs
            fixed_msg = Odometry()
            
            # Copy all data
            fixed_msg.pose = msg.pose
            fixed_msg.twist = msg.twist
            
            # Fix header
            fixed_msg.header.stamp = msg.header.stamp
            fixed_msg.header.frame_id = self.target_frame_id  # Set proper frame_id
            fixed_msg.child_frame_id = self.child_frame_id    # Set proper child_frame_id
            
            # Publish fixed message
            self.publisher.publish(fixed_msg)
            
            # Log occasionally
            if self.get_clock().now().nanoseconds % 1000000000 < 50000000:  # ~Every second
                self.get_logger().info(
                    f'Fixed Odometry: {msg.header.frame_id or "empty"} -> {self.target_frame_id}, '
                    f'child: {msg.child_frame_id or "empty"} -> {self.child_frame_id}'
                )
                
        except Exception as e:
            self.get_logger().error(f'Error fixing Odometry message: {str(e)}')

    def pose_callback(self, msg):
        """Handle PoseStamped messages and fix frame ID"""
        try:
            # Create new message with corrected frame ID
            fixed_msg = PoseStamped()
            
            # Copy pose data
            fixed_msg.pose = msg.pose
            
            # Fix header
            fixed_msg.header.stamp = msg.header.stamp
            fixed_msg.header.frame_id = self.target_frame_id  # Set proper frame_id
            
            # Publish fixed message
            self.publisher.publish(fixed_msg)
            
            # Log occasionally
            if self.get_clock().now().nanoseconds % 1000000000 < 50000000:  # ~Every second
                self.get_logger().info(
                    f'Fixed PoseStamped: {msg.header.frame_id or "empty"} -> {self.target_frame_id}'
                )
                
        except Exception as e:
            self.get_logger().error(f'Error fixing PoseStamped message: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    qvio_frame_fixer = QVIOFrameFixer()
    
    try:
        rclpy.spin(qvio_frame_fixer)
    except KeyboardInterrupt:
        pass
    finally:
        qvio_frame_fixer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()