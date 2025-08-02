#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class StereoCameraInfoPublisher(Node):
    def __init__(self):
        super().__init__('voxl_camera_info_publisher')
        self.get_logger().info('Initializing StereoCameraInfoPublisher')

        # Publishers for left and right camera info
        self.left_info_pub = self.create_publisher(CameraInfo, '/stereo_front/left/camera_info', 10)
        self.right_info_pub = self.create_publisher(CameraInfo, '/stereo_front/right/camera_info', 10)

        # TF broadcaster for left->right static transform
        self.tf_broadcaster = TransformBroadcaster(self)

        # === Left Camera Info ===
        self.left_camera_info = CameraInfo()
        self.left_camera_info.header.frame_id = 'stereo_front_left'
        self.left_camera_info.width = 640
        self.left_camera_info.height = 480
        self.left_camera_info.distortion_model = 'plumb_bob'
        self.left_camera_info.k = [
            484.08342805766921, 0.0, 341.15511576727556,
            0.0, 484.75970973915867, 256.28063347009578,
            0.0, 0.0, 1.0
        ]
        self.left_camera_info.d = [
            -0.17891434683926608, 0.14253921745148040,
            0.0018905601741798163, -0.0012315866702996034,
            -0.086417275487469039
        ]
        self.left_camera_info.r = [1.0, 0.0, 0.0,
                                   0.0, 1.0, 0.0,
                                   0.0, 0.0, 1.0]
        self.left_camera_info.p = [
            484.08342805766921, 0.0, 341.15511576727556, 0.0,
            0.0, 484.75970973915867, 256.28063347009578, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]
        self.left_camera_info.binning_x = 1
        self.left_camera_info.binning_y = 1
        self.left_camera_info.roi.do_rectify = False

        self.get_logger().info('Left camera intrinsics loaded (reprojection error: 0.21678, date: 2025-06-24)')

        # === Right Camera Info ===
        self.right_camera_info = CameraInfo()
        self.right_camera_info.header.frame_id = 'stereo_front_right'
        self.right_camera_info.width = 640
        self.right_camera_info.height = 480
        self.right_camera_info.distortion_model = 'plumb_bob'
        self.right_camera_info.k = [
            502.37466742598525, 0.0, 295.44695999117351,
            0.0, 502.44836915828580, 247.14374496341932,
            0.0, 0.0, 1.0
        ]
        self.right_camera_info.d = [
            -0.20398266922164715, 0.22579291428071341,
            0.00050480413413915787, -0.0015391522146159773,
            -0.18717055379417169
        ]
        self.right_camera_info.r = [1.0, 0.0, 0.0,
                                    0.0, 1.0, 0.0,
                                    0.0, 0.0, 1.0]

        # Extrinsics baseline (from T[0])
        baseline = 0.08065419267391806  # meters
        fx = 484.08342805766921  # left camera fx
        tx = -fx * baseline  # ≈ -39.048...

        self.right_camera_info.p = [
            fx, 0.0, 341.15511576727556, tx,
            0.0, 484.75970973915867, 256.28063347009578, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]
        self.right_camera_info.binning_x = 1
        self.right_camera_info.binning_y = 1
        self.right_camera_info.roi.do_rectify = False

        self.get_logger().info('Right camera intrinsics loaded (reprojection error: 0.18795, date: 2025-06-24)')
        self.get_logger().warn(
            'Extrinsics loaded (baseline: -0.08065419267391806 m, reprojection error: 0.43705, date: 2023-03-02). '
            'Warning: Extrinsics date differs from intrinsics (2025-06-24). Re-calibration recommended.'
        )

        # === Timers ===
        self.timer = self.create_timer(1.0 / 30.0, self.publish_camera_info)
        self.tf_timer = self.create_timer(1.0, self.publish_static_tf)  # publish TF at 1 Hz
        self.get_logger().info('Publishing camera info at 30 Hz and TF at 1 Hz')

    def publish_camera_info(self):
        now = self.get_clock().now().to_msg()
        self.left_camera_info.header.stamp = now
        self.right_camera_info.header.stamp = now
        self.left_info_pub.publish(self.left_camera_info)
        self.right_info_pub.publish(self.right_camera_info)
        self.get_logger().debug('Published camera info messages')

    def publish_static_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'stereo_front_left'
        t.child_frame_id = 'stereo_front_right'
        t.transform.translation.x = -0.08065419267391806  # right is left of left
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = StereoCameraInfoPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down due to keyboard interrupt')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
