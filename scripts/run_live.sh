#!/usr/bin/env bash
set -e
cd ~/amiga_slam
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch amiga_bringup sensors_live.launch.py
