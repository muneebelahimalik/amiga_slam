from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    cloud_topic = LaunchConfiguration('cloud_topic')

    # ICP Odometry from point cloud
    icp_odom = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        name='icp_odometry',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'publish_tf': True,
            'approx_sync': True,
            'wait_for_transform': 0.2,
        }],
        # IMPORTANT: rtabmap nodes subscribe to "scan_cloud" by default
        remappings=[
            ('scan_cloud', cloud_topic),
        ]
    )

    # RTAB-Map SLAM
    rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'map_frame_id': 'map',
            'publish_tf': True,
            'approx_sync': True,
            'wait_for_transform': 0.2,
            # LiDAR-only mode: disable camera sync, enable point cloud input
            'subscribe_scan_cloud': True,
            'subscribe_rgb': False,
            'subscribe_depth': False,
        }],
        remappings=[
            ('scan_cloud', cloud_topic),
        ]
    )

    # Visualization
    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'map_frame_id': 'map',
            'approx_sync': True,
            'subscribe_scan_cloud': True,
            'subscribe_rgb': False,
            'subscribe_depth': False,
            'subscribe_odom': True,
        }],
        remappings=[
            ('scan_cloud', cloud_topic),
            ('odom', '/odom'),
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('cloud_topic', default_value='/velodyne_points'),
        icp_odom,
        rtabmap,
        rtabmap_viz,
    ])
