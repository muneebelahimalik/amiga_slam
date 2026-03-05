from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Update these once you physically mount the VLP-16 (meters, radians)
    x = 0.0
    y = 0.0
    z = 0.0
    roll = 0.0
    pitch = 0.0
    yaw = 0.0

    base_to_velodyne = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_velodyne_tf",
        output="screen",
        arguments=[
            "--x", str(x),
            "--y", str(y),
            "--z", str(z),
            "--roll", str(roll),
            "--pitch", str(pitch),
            "--yaw", str(yaw),
            "--frame-id", "base_link",
            "--child-frame-id", "velodyne",
        ],
    )

    return LaunchDescription([base_to_velodyne])
