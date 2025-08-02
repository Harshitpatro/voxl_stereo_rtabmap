from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # QVIO Frame Fixer - fixes empty frame_id in /qvio messages
        Node(
            package='slam',  # Replace with your package name
            executable='merge_pc.py',  # Save the frame fixer code as this file
            name='qvio_frame_fixer',
            output='screen',
            parameters=[{
                'input_topic': '/qvio',
                'output_topic': '/qvio_fixed',
                'target_frame_id': 'world',      # Set frame_id to 'world'
                'child_frame_id': 'base_link',   # Set child_frame_id to 'base_link'
            }],
        ),
        
        # TF Tree setup: map -> world -> base_link -> stereo_front
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_world_static',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'world'],
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_base_link_static',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link'],
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_stereo_front_static',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'stereo_front'],
        ),
        
        # Camera info publisher
        Node(
            package='slam',
            executable='voxl_camera_info_publisher.py',
            name='camera_info_publisher',
            output='screen',
        ),
        
        # RTABMap SLAM node - now uses /qvio_fixed with proper frame IDs
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'frame_id': 'base_link',
                'map_frame_id': 'map',
                'odom_frame_id': 'world',        # world frame (from fixed /qvio)
                'subscribe_scan': False,
                'subscribe_depth': False,
                'subscribe_rgb': False,
                'subscribe_scan_cloud': True,
                'subscribe_odom': True,
                'approx_sync': True,
                'sync_queue_size': 30,
                'topic_queue_size': 30,
                'Rtabmap/DbSqlite3/InMemory': 'true',
                'wait_for_transform': 3.0,
                'tf_delay': 0.5,
                'tf_tolerance': 0.5,
                'database_path': '/home/aero360/.ros/rtabmap_slam.db',  # Custom database name
                'Rtabmap/DbSqlite3/InMemory': 'false',
                'Rtabmap/DetectionRate': '1.0',
                'Rtabmap/TimeThr': '0.0',
                'Rtabmap/MemoryThr': '0',
                'Icp/VoxelSize': '0.05',
                'Icp/MaxCorrespondenceDistance': '0.1',
                'Grid/FromDepth': 'false',
                'Grid/RangeMax': '10.0',
                'Reg/Strategy': '1',
            }],
            remappings=[
                ('scan_cloud', '/stereo_front_pc'),
                ('odom', '/qvio_fixed'),          # Use the fixed topic
                ('camera_info', '/stereo_front_camera_info'),
                ('left/image_rect', '/stereo_front_reliable'),
                
            ],
        ),
        
        # RTAB-Map visualization node
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            output='screen',
            parameters=[{
                'frame_id': 'base_link',
                'map_frame_id': 'map',
                'odom_frame_id': 'world',
            }],
            remappings=[
                ('left/image_rect', '/stereo_front_reliable'),
                ('right/image_rect', '/stereo_front_reliable'),
                ('left/camera_info', '/stereo_front_camera_info'),
                ('right/camera_info', '/stereo_front_camera_info'),
                ('scan_cloud', '/stereo_front_pc'),
                ('odom', '/qvio_fixed'),          # Use the fixed topic
            ],
        ),
    ])