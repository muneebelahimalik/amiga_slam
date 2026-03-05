from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Estimated mount position — update once physically measured.
    # Matches the joint origin in urdf/amiga_min.urdf.
    # Values: x=0 (centered), y=0 (centered), z=0.80 m above base_link
    # (≈ 0.76 m robot frame height + 0.04 m bracket).
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_base_to_velodyne',
        output='screen',
        arguments=[
            '--frame-id', 'base_link',
            '--child-frame-id', 'velodyne',
            '--x', '0', '--y', '0', '--z', '0.80',
            '--roll', '0', '--pitch', '0', '--yaw', '0'
        ]
    )
    return LaunchDescription([static_tf])
