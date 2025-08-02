#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition
from launch_ros.actions import Node
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    rectify_images = LaunchConfiguration('rectify_images', default='false')

    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'rectify_images',
            default_value='true',
            description='Enable stereo_image_proc for rectification if true'
        ),

        # QVIO Frame Fixer - fixes empty frame_id in /qvio messages
        Node(
            package='slam',  # Replace with your package name
            executable='merge_pc.py',
            name='qvio_frame_fixer',
            output='screen',
            parameters=[{
                'input_topic': '/qvio',
                'output_topic': '/qvio_fixed',
                'target_frame_id': 'world',
                'child_frame_id': 'base_link',
            }],
        ),

        # TF Tree setup: map -> world -> base_link -> stereo_front_left -> stereo_front_right
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_world_static',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'world'],
            output='screen'
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_base_link_static',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link'],
            output='screen'
        ),

        # Stereo camera transforms
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_stereo_front_left_static',
            arguments=['0.0', '0.0', '0.0', '0', '0', '0', 'base_link', 'stereo_front_left'],  # Adjust as needed
            output='screen'
        ),

        

        # Stereo camera info publisher
        Node(
            package='slam',  # Replace with your package name
            executable='voxl_camera_info_publisher.py',  # Updated script from previous response
            name='stereo_camera_info_publisher',
            output='screen',
            parameters=[{
                       'use_sim_time': use_sim_time
    }]
        ),

        # RTAB-Map SLAM node
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'frame_id': 'stereo_front_left',  # Camera frame for odometry
                'map_frame_id': 'map',
                'odom_frame_id': 'world',
                'subscribe_depth': False,
                'subscribe_rgb': False,
                'subscribe_stereo': True,
                'subscribe_rgbd': False,
                
                'subscribe_odom': True,
                'subscribe_user_data': False,
                'subscribe_scan': False,
                'subscribe_scan_cloud': False,
                'subscribe_scan_descriptor': False,
                'approx_sync': True,  # Enable approximate sync
                'queue_size': 30,
                'sync_queue_size': 30,
                'qos_image': 0,
                'qos_camera_info': 0,
                'qos_scan': 0,
                'qos_odom': 0,
                'qos_user_data': 0,
                'wait_for_transform': 3.0,
                'tf_delay': 0.5,
                'tf_tolerance': 0.5,
                'database_path': '/home/aero360/.ros/rtabmap_stereo_new1slam.db',
                'Rtabmap/DbSqlite3/InMemory': 'true',
                'Rtabmap/DetectionRate': '1.0',
                'Rtabmap/TimeThr': '0.0',
                'Rtabmap/MemoryThr': '0',
                'Mem/IncrementalMemory': 'true',  # SLAM mode
                # Stereo-specific parameters
                'Stereo/MaxDisparity': '192.0',
                'Stereo/MinDisparity': '0.0',
                'StereoBM/BlockSize': '15',
                'StereoBM/MinDisparity': '0',
                'StereoBM/NumDisparities': '192',
                'StereoBM/PreFilterCap': '31',
                'StereoBM/PreFilterSize': '9',
                'StereoBM/SpeckleRange': '4',
                'StereoBM/SpeckleWindowSize': '100',
                'StereoBM/TextureThreshold': '20',
                'StereoBM/UniquenessRatio': '10',
                # Visual odometry parameters
                'Vis/EstimationType': '1',  # 0=3D->3D
                'Vis/MinInliers': '1000',     # Match log warning
                'Vis/InlierDistance': '0.1',
                'Vis/MaxDepth': '5.0',
                'Vis/DepthAsMask': 'true',
                'Vis/MaxFeatures': '700',
                'Vis/FeatureType': '1',     # 6=GFTT/BRIEF (due to xfeatures2d unavailability)
                # Map building
                'Grid/FromDepth': 'true',
                'Grid/DepthDecimation': '2',
                'Grid/RangeMax': '10.0',
                'Grid/RangeMin': '0.1',
                'Grid/CellSize': '0.05',
                'Kp/MaxFeatures': '1000',
                'Kp/MaxDepth': '5.0',
                'Kp/DetectorStrategy': '6',
                    # 0=FAST
                # Registration
                'Reg/Strategy': '0',  # Visual only
                'Reg/Force3DoF': 'false',
            }],
            remappings=[
                ('odom', '/qvio_fixed'),
                ('left/image_rect', '/stereo_front_left_reliable'),
                ('right/image_rect', '/stereo_front_right_reliable'),
                ('left/camera_info', '/stereo_front/left/camera_info'),
                ('right/camera_info', '/stereo_front/right/camera_info'),
                ('imu','/imu_apps'),
            ],
        ),

        # RTAB-Map visualization node
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'frame_id': 'stereo_front_left',  # Use camera frame for consistency
                'map_frame_id': 'map',
                'odom_frame_id': 'world',
                'subscribe_depth': False,
                'subscribe_rgb': False,
                'subscribe_stereo': True,
                'subscribe_rgbd': False,
                'subscribe_odom': True,
                'subscribe_user_data': False,
                'subscribe_scan': False,
                'subscribe_scan_cloud': False,
                'subscribe_scan_descriptor': False,
                'approx_sync': True,  # Enable approximate sync
                'queue_size': 10,
                'qos_image': 0,
                'qos_camera_info': 0,
                'qos_scan': 0,
                'qos_odom': 0,
                'qos_user_data': 0,
            }],
            remappings=[
                ('left/image_rect', '/stereo_front_left_reliable'),
                ('right/image_rect', '/stereo_front_right_reliable'),
                ('left/camera_info', '/stereo_front/left/camera_info'),
                ('right/camera_info', '/stereo_front/right/camera_info'),
                ('odom','/qvio_fixed'),
                ('imu','/imu_apps'),  # Match log subscription
            ],
        ),

        # Optional: Stereo image processor (for unrectified images)
        Node(
            package='stereo_image_proc',
            executable='disparity_node',
            name='stereo_image_proc',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'approximate_sync': True,
            }],
            remappings=[
                ('left/image_raw', '/stereo_front_left_reliable'),
                ('right/image_raw', '/stereo_front_right_reliable'),
                ('left/camera_info', '/stereo_front/left/camera_info'),
                ('right/camera_info', '/stereo_front/right/camera_info'),
                ('left/image_rect', '/stereo/left/image_rect'),
                ('right/image_rect', '/stereo/right/image_rect'),
            ],
            condition=UnlessCondition(rectify_images),  # Enable if rectify_images=true
        ),
        Node(
             package='rviz2',
             executable='rviz2',
             name='rviz2',
             output='screen',
             arguments=['-d', PathJoinSubstitution([
             FindPackageShare('slam'),  # Replace 'slam' with your actual package name if different
                           'config',
                            'stereo_rtabmap_vio.rviz'
    ])],
),
    ])
