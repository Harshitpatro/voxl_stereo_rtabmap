#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

class StereoTFPublisher(Node):
    def __init__(self):
        super().__init__('stereo_tf_publisher')
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.publish_tf()

    def publish_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'stereo_front_left'
        t.child_frame_id = 'stereo_front_right'
        t.transform.translation.x = -0.080654192673918060
        t.transform.translation.y = -0.00048394511427368833
        t.transform.translation.z = 0.0062772610676518252
        # Convert rotation matrix to quaternion (approximate values)
        t.transform.rotation.x = 0.017930933849970264
        t.transform.rotation.y = 0.00010031539633260698
        t.transform.rotation.z = -0.046666656153742811
        t.transform.rotation.w = 0.99874957062053160
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = StereoTFPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()