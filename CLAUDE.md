# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS2 Humble workspace for 3D SLAM on the Amiga robot using a Velodyne VLP-16 LiDAR. The single ROS2 package (`amiga_bringup`) handles sensor bringup, static TF publishing, and RTAB-Map-based SLAM.

## Build & Run Commands

All commands assume the workspace root is `~/amiga_slam`.

**Build:**
```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```
Or use the helper script: `bash scripts/build.sh`

**Source environment (without rebuilding):**
```bash
source scripts/env.sh
```

**Run live sensors (VLP-16 driver + static TF):**
```bash
bash scripts/run_live.sh
# equivalent to:
ros2 launch amiga_bringup sensors_live.launch.py
```

**Run SLAM (requires sensors running first or bag playback):**
```bash
ros2 launch amiga_bringup slam_rtabmap_lidar3d.launch.py
# With bag replay:
ros2 launch amiga_bringup slam_rtabmap_lidar3d.launch.py use_sim_time:=true cloud_topic:=/velodyne_points
```

**Play back a recorded bag:**
```bash
ros2 bag play data/bags/vlp16_test/ --clock
```

**Run tests:**
```bash
colcon test --packages-select amiga_bringup
colcon test-result --verbose
```

## Architecture

### TF Tree
```
map
 └── odom          (published by rtabmap_slam via ICP odometry)
      └── base_link (published by icp_odometry)
           └── velodyne  (static TF, identity until LiDAR is physically measured)
```

### Node Pipeline
1. **`velodyne_driver_node`** — reads UDP packets from VLP-16 at `192.168.1.201:2368`, publishes `/velodyne_packets`
2. **`velodyne_transform_node`** — converts packets to `sensor_msgs/PointCloud2` on `/velodyne_points`; configured via `config/velodyne_transform.yaml`
3. **`icp_odometry`** (rtabmap_odom) — subscribes to `/velodyne_points` (remapped from `scan_cloud`), publishes `odom -> base_link` TF
4. **`rtabmap`** (rtabmap_slam) — builds the map, publishes `map -> odom` TF
5. **`rtabmap_viz`** — 3D visualization

### Key Files
- `src/amiga_bringup/launch/sensors_live.launch.py` — top-level live sensor launch (includes `velodyne_vlp16.launch.py` + `tf_static_base_to_velodyne.launch.py`)
- `src/amiga_bringup/launch/slam_rtabmap_lidar3d.launch.py` — SLAM launch with args `use_sim_time` and `cloud_topic`
- `src/amiga_bringup/config/velodyne_transform.yaml` — VLP-16 pointcloud conversion params (range, organization, FOV)
- `src/amiga_bringup/urdf/amiga_min.urdf` — minimal URDF with only `base_link` and `velodyne` links
- `data/bags/vlp16_test/` — test bag recording (`/velodyne_packets` + `/velodyne_points`, ~90s)

### Pending Physical Calibration
The `base_link -> velodyne` static TF is currently identity (`0 0 0 0 0 0`). Once the LiDAR is physically mounted, update the values in `src/amiga_bringup/launch/tf_static_base_to_velodyne.launch.py` and `src/amiga_bringup/urdf/amiga_min.urdf`.

### Redundant Launch Files
Several launch files overlap in functionality (legacy/exploratory versions):
- `static_tf.launch.py`, `tf_static.launch.py`, `tf_static_base_to_velodyne.launch.py` — all publish the same `base_link -> velodyne` static TF
- `sensors_vlp16.launch.py`, `bringup_sensors.launch.py`, `sensors_live.launch.py` — all bring up the Velodyne sensor stack

The **active** files used by the run scripts are `sensors_live.launch.py` and `tf_static_base_to_velodyne.launch.py`.
