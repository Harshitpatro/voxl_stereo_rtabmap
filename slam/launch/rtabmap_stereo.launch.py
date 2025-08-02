from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rtabmap_ros',
            executable='stereo_mapping',
            name='rtabmap',
            parameters=[{
                'frame_id': 'base_link',
                'approx_sync': True,
                'subscribe_depth': False,
                'subscribe_rgb': False,
                'subscribe_stereo': True,
                'queue_size': 10,
                'Rtabmap/DetectionRate': '1.0',
                'Stereo/OpticalFlow': 'false',  # Use stereo disparity
            }],
            remappings=[
                ('left/image_rect', '/stereo_front_left'),
                ('right/image_rect', '/stereo_front_right'),
                ('left/camera_info', '/stereo_front_left_camera_info'),
                ('right/camera_info', '/stereo_front_right_camera_info'),
                ('odom', '/odom'),  # Optional, if odometry is available
            ],
            output='screen',
        ),
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            parameters=[{
                'frame_id': 'base_link',
                'subscribe_stereo': True,
            }],
            remappings=[
                ('left/image_rect', '/stereo_front_left'),
                ('right/image_rect', '/stereo_front_right'),
                ('left/camera_info', '/stereo_front_left_camera_info'),
                ('right/camera_info', '/stereo_front_right_camera_info'),
            ],
            output='screen',
        ),
    ])