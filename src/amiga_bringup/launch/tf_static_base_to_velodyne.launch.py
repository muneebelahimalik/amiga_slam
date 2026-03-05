from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Temporary identity until you measure the mount.
    # Replace x y z roll pitch yaw later.
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_base_to_velodyne',
        output='screen',
        arguments=[
            '--frame-id', 'base_link',
            '--child-frame-id', 'velodyne',
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0'
        ]
    )
    return LaunchDescription([static_tf])
