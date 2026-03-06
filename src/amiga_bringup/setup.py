from setuptools import setup
import os
from glob import glob

package_name = 'amiga_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='muneebelahimalik',
    maintainer_email='59524535+muneebelahimalik@users.noreply.github.com',
    description='Bringup for Farm-ng Amiga SLAM stack',
    license='MIT',
    entry_points={
        'console_scripts': [
            'amiga_odometry = amiga_bringup.amiga_odometry:main',
            'amiga_velocity_bridge = amiga_bringup.amiga_velocity_bridge:main',
            'amiga_ros2_bridge = amiga_bringup.amiga_ros2_bridge:main',
            'field_coverage_planner = amiga_bringup.field_coverage_planner:main',
        ],
    },
)
