#!/usr/bin/env bash
set -e
cd ~/amiga_slam
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
echo "Built and sourced workspace."
