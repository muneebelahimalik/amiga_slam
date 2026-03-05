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

**Run live sensors + SLAM together (all-in-one, no Amiga odometry):**
```bash
ros2 launch amiga_bringup slam_live.launch.py
```

**Run full stack (VLP-16 + Amiga wheel odometry bridge + SLAM) — RECOMMENDED:**
```bash
ros2 launch amiga_bringup slam_full.launch.py
ros2 launch amiga_bringup slam_full.launch.py rviz:=true database_path:=~/maps/field.db
```

**Run only the Amiga bridge nodes (odometry + cmd_vel relay):**
```bash
ros2 launch amiga_bringup amiga_bringup_nodes.launch.py
# With EKF fusion of ICP + wheel odometry:
ros2 launch amiga_bringup amiga_bringup_nodes.launch.py use_ekf:=true
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

### Node Pipeline (slam_full.launch.py)

```
Amiga gRPC canbus service (on robot hardware)
    │
    │  amiga_ros_bridge (ROS 1 Noetic)  ← OR ─── ros1_bridge ─── OR native ROS 2 gRPC client
    │        /amiga/vel  (TwistStamped)
    │        /amiga/cmd_vel (Twist)
    │
    ▼ ROS 2 Humble
┌─────────────────────────────────────────────────────────────────────┐
│  amiga_odometry       /amiga/vel → /wheel_odom (dead-reckoning)     │
│  amiga_velocity_bridge /cmd_vel  → /amiga/cmd_vel (Nav2 → Amiga)    │
└─────────────────────────────────────────────────────────────────────┘
    │                       │
    │ /wheel_odom           │ /cmd_vel (from Nav2 / teleop)
    ▼                       ▼
┌─────────────────────────────────────────────────────────────────────┐
│  velodyne_driver_node → /velodyne_packets                           │
│  velodyne_transform_node → /velodyne_points                         │
│  icp_odometry  /velodyne_points → /odom  +  odom→base_link TF      │
│  rtabmap       /velodyne_points + /odom → map, map→odom TF         │
│  rtabmap_viz   (rtabmap output only, no raw scan subscription)      │
│  robot_state_publisher  URDF → base_link→velodyne TF               │
└─────────────────────────────────────────────────────────────────────┘

Optional EKF fusion (amiga_bringup_nodes.launch.py use_ekf:=true):
    /odom (ICP) + /wheel_odom (wheels) → ekf_filter_node → /odometry/filtered
    (when enabled: icp_odometry publish_tf must be false; EKF owns odom→base_link)
```

1. **`velodyne_driver_node`** — reads UDP packets from VLP-16 at `192.168.1.201:2368`, publishes `/velodyne_packets`
2. **`velodyne_transform_node`** — converts packets → `sensor_msgs/PointCloud2` on `/velodyne_points`
3. **`amiga_odometry`** — `/amiga/vel` (TwistStamped) → `/wheel_odom` (Odometry); dead-reckoning integration
4. **`amiga_velocity_bridge`** — `/cmd_vel` (Twist) → `/amiga/cmd_vel`; safety watchdog stops robot if commands stop
5. **`icp_odometry`** (rtabmap_odom) — subscribes to `/velodyne_points`, publishes `odom → base_link` TF
6. **`rtabmap`** (rtabmap_slam) — builds the map, publishes `map → odom` TF, saves database
7. **`rtabmap_viz`** — 3D visualization (subscribes to rtabmap output only, not raw scans)

### Key Files
- `src/amiga_bringup/amiga_bringup/amiga_ros2_bridge.py` — **native gRPC bridge**; connects to Amiga over Tailscale; publishes `/amiga/vel` + `/amiga/pose`; forwards `/cmd_vel` to Amiga
- `src/amiga_bringup/amiga_bringup/amiga_odometry.py` — wheel odometry node; `/amiga/vel` → `/wheel_odom`
- `src/amiga_bringup/amiga_bringup/amiga_velocity_bridge.py` — fallback cmd_vel relay (only needed without gRPC bridge)
- `src/amiga_bringup/config/ekf.yaml` — robot_localization EKF config fusing ICP + wheel odometry
- `src/amiga_bringup/launch/slam_full.launch.py` — **RECOMMENDED** all-in-one live launch: gRPC bridge + VLP-16 + SLAM
- `src/amiga_bringup/launch/amiga_grpc_bridge.launch.py` — launch just the native gRPC bridge node
- `src/amiga_bringup/launch/amiga_bringup_nodes.launch.py` — Amiga odometry + velocity bridge (with optional EKF)
- `config/service_configs/canbus_config.json` — farm-ng canbus service config (port 6001)
- `config/service_configs/filter_config.json` — farm-ng filter service config (port 20001)
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

### Amiga ROS Bridge Integration (Stage 2)

The `amiga_ros_bridge` is written for **ROS 1 Noetic** (catkin, roslaunch).  Our SLAM stack
runs in **ROS 2 Humble**.  Three options to bridge the gap:

**Implemented: native ROS 2 gRPC bridge (no ROS 1 required)**

The `amiga_ros2_bridge` node connects directly to the Amiga's gRPC services
over Tailscale using the farm-ng Python SDK — the same SDK used by filter_client.py.

**IMPORTANT: Install farm-ng in the system Python BEFORE building (run once):**
```bash
bash ~/amiga_slam/scripts/install_farmng.sh
# Installs farm-ng-amiga + farm-ng-core into the Python ROS 2 uses.
# Do NOT use a venv — ROS 2 nodes cannot see venv-installed packages.
```

```bash
# Start the bridge (Tailscale must be active)
ros2 launch amiga_bringup amiga_grpc_bridge.launch.py

# Override host if needed:
ros2 launch amiga_bringup amiga_grpc_bridge.launch.py \
    host:=camphor-clone.tail0be07.ts.net canbus_port:=6001 filter_port:=20001
```

Service configs (for standalone SDK testing) are in `config/service_configs/`:
- `canbus_config.json` — canbus service (wheel velocity + commands)
- `filter_config.json` — filter service (GPS + IMU pose)

Verify connectivity before running ROS 2:
```bash
# Test filter service (already confirmed working):
cd ~/farm-ng-amiga/py/examples/filter_client
python3 main.py --service-config ~/amiga_slam/config/service_configs/filter_config.json

# Test canbus service (new):
# (use a similar EventClient subscriber script, or check AmigaTpdo1 messages)
```

### Amiga OS 2.0 (Barley) SDK API notes

The bridge uses OS 2.0 patterns (farm-ng-core/amiga >= 2.0.0):
- `AmigaTpdo1` and `Twist2d` live in `farm_ng.canbus.packet` (not `canbus_pb2`)
- Canbus subscription: `decode=False` → `payload_to_protobuf(event, payload)`
  → `AmigaTpdo1.from_proto(message.amiga_tpdo1)`
- Velocity commands: `await client.request_reply("/twist", Twist2d(...))` within
  the subscribe loop — no raw gRPC stub needed (matches `pose_generator` pattern)
- Filter subscription: unchanged `decode=True` (same as filter_client.py)

### Amiga Topics (ROS 2)

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/amiga/vel` | `geometry_msgs/TwistStamped` | **subscribe** | Measured wheel velocity (linear.x, angular.z) |
| `/amiga/cmd_vel` | `geometry_msgs/Twist` | **publish** | Velocity commands to drive the robot |
| `/wheel_odom` | `nav_msgs/Odometry` | **publish** | Dead-reckoning odometry from `amiga_odometry` |
| `/cmd_vel` | `geometry_msgs/Twist` | **subscribe** | From Nav2 / teleop; forwarded to `/amiga/cmd_vel` |
| `/odometry/filtered` | `nav_msgs/Odometry` | **publish** | EKF-fused odometry (when use_ekf:=true) |

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
