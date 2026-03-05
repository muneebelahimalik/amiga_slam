"""
slam_full.launch.py — complete live stack: VLP-16 + Amiga odometry + SLAM.

This is the all-in-one launch file for running on the physical Amiga robot.
It brings up all components in the correct order:

  1. Velodyne VLP-16 driver  →  /velodyne_packets, /velodyne_points
  2. robot_state_publisher   →  base_link → velodyne TF (from URDF)
  3. amiga_odometry node     →  /amiga/vel → /wheel_odom
  4. amiga_velocity_bridge   →  /cmd_vel → /amiga/cmd_vel
  5. icp_odometry            →  /velodyne_points → /odom + odom→base_link TF
  6. rtabmap                 →  builds map, publishes map→odom TF
  7. rtabmap_viz             →  3-D visualisation
  8. RViz2 (optional)

Prerequisites
-------------
• The amiga_ros_bridge must be running (on robot or bridged via ros1_bridge)
  so that /amiga/vel and /amiga/cmd_vel are live in ROS 2.
• VLP-16 reachable at 192.168.1.201:2368.

Usage
-----
  ros2 launch amiga_bringup slam_full.launch.py
  ros2 launch amiga_bringup slam_full.launch.py rviz:=true
  ros2 launch amiga_bringup slam_full.launch.py database_path:=~/maps/field.db
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory('amiga_bringup')
    rviz = LaunchConfiguration('rviz')
    database_path = LaunchConfiguration('database_path')

    # ---- VLP-16 sensor stack ----
    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'velodyne_vlp16.launch.py')
        )
    )

    # ---- Amiga odometry + velocity bridge ----
    amiga_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'amiga_bringup_nodes.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'use_ekf': 'false',   # ICP odometry owns odom→base_link
        }.items(),
    )

    # ---- SLAM stack (ICP odom + rtabmap + viz + robot_state_publisher) ----
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'slam_rtabmap_lidar3d.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'cloud_topic': '/velodyne_points',
            'database_path': database_path,
            'rviz': rviz,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Launch RViz2'),
        DeclareLaunchArgument(
            'database_path', default_value='~/.ros/rtabmap.db',
            description='Path to RTAB-Map database (map saved here on exit)'),
        sensors,
        amiga_nodes,
        slam,
    ])
