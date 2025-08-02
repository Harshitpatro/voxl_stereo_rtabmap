from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Static transform from map to odom (RTABMap will override this)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_static',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        ),
        
        # Static transform from odom to world (since your point cloud and pose are in world frame)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_world_static',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'world'],
        ),
        
        # RTABMap SLAM node - simplified approach
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'frame_id': 'world',  # Use world frame since that's where your point cloud is
                'map_frame_id': 'map',
                'odom_frame_id': 'odom',
                
                # Subscription settings
                'subscribe_scan': False,
                'subscribe_depth': False,
                'subscribe_rgb': False,
                'subscribe_scan_cloud': True,
                'subscribe_odom': True,  # Enable odom subscription
                
                # Sync settings
                'approx_sync': True,  # Use approximate sync
                'sync_queue_size': 20,
                'topic_queue_size': 20,
                
                # Database settings
                'Rtabmap/DbSqlite3/InMemory': 'true',  # String value
                
                # TF settings
                'wait_for_transform': 1.0,
                'tf_delay': 0.1,
                'tf_tolerance': 0.2,
                
                # RTAB-Map specific parameters (all as strings)
                'Rtabmap/DetectionRate': '1.0',
                'Rtabmap/TimeThr': '0.0',
                'Rtabmap/MemoryThr': '0',
                
                # ICP parameters for point cloud processing (as strings)
                'Icp/VoxelSize': '0.05',
                'Icp/MaxCorrespondenceDistance': '0.1',
            }],
            remappings=[
                ('scan_cloud', '/voa_pc_out'),
                ('odom', '/qvio_odom'),  # Use the clean odom topic if you're using topic relay
            ],
        ),
        
    ])