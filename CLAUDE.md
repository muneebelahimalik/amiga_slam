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

**Run live sensors + SLAM together (all-in-one):**
```bash
ros2 launch amiga_bringup slam_live.launch.py
```

**Run SLAM only (sensors already running separately):**
```bash
ros2 launch amiga_bringup slam_rtabmap_lidar3d.launch.py
# With RViz and a custom DB path:
ros2 launch amiga_bringup slam_rtabmap_lidar3d.launch.py rviz:=true database_path:=~/maps/field.db
```

**Run bag replay + SLAM together (all-in-one):**
```bash
ros2 launch amiga_bringup slam_bag_replay.launch.py
# Custom bag, slower replay, custom database:
ros2 launch amiga_bringup slam_bag_replay.launch.py bag_path:=/path/to/bag playback_rate:=0.5 database_path:=~/maps/field.db
```

**Save the built map (while SLAM is running):**
```bash
bash scripts/save_map.sh                     # saves to ~/maps/<timestamp>/
bash scripts/save_map.sh /data/field_map     # saves to specified directory
```

**Localise against a saved map (live robot, no new mapping):**
```bash
ros2 launch amiga_bringup slam_localization.launch.py database_path:=~/maps/field.db
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
- `src/amiga_bringup/launch/slam_rtabmap_lidar3d.launch.py` — SLAM stack (ICP odom + rtabmap + viz + robot_state_publisher); args: `use_sim_time`, `cloud_topic`, `database_path`, `rviz`
- `src/amiga_bringup/launch/slam_live.launch.py` — live all-in-one: VLP-16 driver + static TF + SLAM (`use_sim_time=false`)
- `src/amiga_bringup/launch/slam_bag_replay.launch.py` — bag replay + static TF + SLAM in one launch; `use_sim_time` is hardcoded true; bag starts after a 3s delay so SLAM nodes are ready; arg: `database_path`
- `src/amiga_bringup/launch/slam_localization.launch.py` — localization-only mode; loads saved `database_path`, sets `Mem/IncrementalMemory=false`
- `src/amiga_bringup/config/velodyne_transform.yaml` — VLP-16 pointcloud conversion params (range, organization, FOV); calibration must be a full absolute path — see note below
- `src/amiga_bringup/urdf/amiga_min.urdf` — URDF with `base_link` (ground-level center) and `velodyne` (estimated z=0.80 m above base_link); includes visual/collision/inertia
- `src/amiga_bringup/rviz/slam_lidar.rviz` — RViz2 config showing TF, live cloud, map cloud, occupancy grid, odometry, trajectory, robot model
- `scripts/save_map.sh` — copies `~/.ros/rtabmap.db` to a timestamped directory; generates a ready-to-use `localise.sh`
- `data/bags/vlp16_test/` — test bag recording (`/velodyne_packets` + `/velodyne_points`, ~90s)

### Physical Calibration (completed 2025-03-05)
The `base_link -> velodyne` static TF has been set from physical measurements:

| Parameter | Value | Notes |
|-----------|-------|-------|
| x | **1.130 m** | 44.5 in forward from wheelbase midpoint to LiDAR spin axis |
| y | **0.000 m** | Centered on robot centreline |
| z | **0.800 m** | LiDAR spin axis height above ground |
| roll/pitch/yaw | **0 rad** | Mount is level; LiDAR front aligns with robot +x |

These values are set identically in **both**:
1. `src/amiga_bringup/launch/tf_static_base_to_velodyne.launch.py`
2. `src/amiga_bringup/urdf/amiga_min.urdf` (`base_to_velodyne` joint origin)

Amiga geometry:
- Robot footprint: 1.10 m (L) × 0.93 m (W), height 0.76 m
- Wheel diameter: ~0.38 m → axle at z=0.19 m above ground
- `base_link`: center of footprint at ground level (z=0)
- LiDAR is front-mounted on the cross bar, 1.130 m ahead of the wheelbase midpoint

### RTAB-Map LiDAR-Only Mode
By default, `rtabmap` and `rtabmap_viz` wait for camera topics (`/rgb/image`, `/depth/image`). For LiDAR-only operation, these params are required on both nodes:
```python
'subscribe_scan_cloud': True,
'subscribe_rgb': False,
'subscribe_depth': False,
```

### rtabmap_viz and Bag Replay
`rtabmap_viz` doing TF lookups at raw scan timestamps causes "extrapolation into the future" warnings during bag replay (the scan header timestamp arrives slightly ahead of the TF buffer). Fix: set `subscribe_scan_cloud: False` in rtabmap_viz so it only visualises rtabmap's processed output. Setting `subscribe_odom: True` does **not** suppress these warnings — the node ignores it.

Expected SLAM output during bag replay: ICP odometry at ~10 Hz, rtabmap at ~1 Hz. `WM=1-3` in rtabmap logs is normal (old nodes transferred to Long-Term Memory).

### Velodyne Calibration File
`velodyne_transform_node` requires a calibration file. When using a custom params file (not the upstream one), the `calibration` field must be an **absolute path** — a bare filename like `VLP16db.yaml` will fail with "Failed to open calibration file":

```yaml
# config/velodyne_transform.yaml
calibration: /opt/ros/humble/share/velodyne_pointcloud/params/VLP16db.yaml
```

An empty string (`calibration: ""`) also crashes the node.

### Redundant Launch Files
Several launch files overlap in functionality (legacy/exploratory versions):
- `static_tf.launch.py`, `tf_static.launch.py`, `tf_static_base_to_velodyne.launch.py` — all publish the same `base_link -> velodyne` static TF
- `sensors_vlp16.launch.py`, `bringup_sensors.launch.py`, `sensors_live.launch.py` — all bring up the Velodyne sensor stack

The **active** files used by the run scripts are `sensors_live.launch.py` and `tf_static_base_to_velodyne.launch.py`.
