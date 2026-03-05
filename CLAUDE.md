# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS 2 Humble workspace for 3D SLAM + autonomous navigation on the farm-ng Amiga robot using a Velodyne VLP-16 LiDAR.  The single ROS 2 package (`amiga_bringup`) handles sensor bringup, static TF publishing, RTAB-Map-based SLAM, native farm-ng gRPC connectivity, and Nav2 autonomous navigation.

**Ultimate goal**: fully autonomous laser-based weed control in onion fields.

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

---

### Mapping (first run in a new field)

**Build a map while driving autonomously — RECOMMENDED for new fields:**
```bash
ros2 launch amiga_bringup slam_nav.launch.py rviz:=true
ros2 launch amiga_bringup slam_nav.launch.py database_path:=~/maps/field.db rviz:=true
```
Drive the robot around the field (teleop or nav goals) until the map is complete, then:
```bash
bash scripts/save_map.sh ~/maps/field
```

**Build a map with manual teleoperation only (no Nav2):**
```bash
ros2 launch amiga_bringup slam_full.launch.py rviz:=true database_path:=~/maps/field.db
```

---

### Autonomous operation (known field)

**Full production run — localization + Nav2 — RECOMMENDED:**
```bash
ros2 launch amiga_bringup localize_nav.launch.py database_path:=~/maps/field.db
ros2 launch amiga_bringup localize_nav.launch.py database_path:=~/maps/field.db rviz:=true
```

**Send autonomous goals:**
```bash
# Single pose goal:
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: map}, pose: {position: {x: 5.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}}}"

# Ordered waypoints (field row traversal):
ros2 action send_goal /follow_waypoints nav2_msgs/action/FollowWaypoints \
    "{poses: [
       {header: {frame_id: map}, pose: {position: {x: 5.0, y: 0.0}, orientation: {w: 1.0}}},
       {header: {frame_id: map}, pose: {position: {x: 10.0, y: 0.0}, orientation: {w: 1.0}}}
    ]}"
```

---

### Individual sub-stacks

**Run live sensors (VLP-16 driver + static TF):**
```bash
ros2 launch amiga_bringup sensors_live.launch.py
```

**Run full SLAM stack (VLP-16 + Amiga gRPC bridge + SLAM, no Nav2):**
```bash
ros2 launch amiga_bringup slam_full.launch.py rviz:=true database_path:=~/maps/field.db
```

**Run only Nav2 (sensors + SLAM already running separately):**
```bash
ros2 launch amiga_bringup nav2.launch.py
```

**Run only the Amiga bridge nodes (odometry + cmd_vel relay):**
```bash
ros2 launch amiga_bringup amiga_bringup_nodes.launch.py
ros2 launch amiga_bringup amiga_bringup_nodes.launch.py use_ekf:=true
```

**Run SLAM only (sensors already running separately):**
```bash
ros2 launch amiga_bringup slam_rtabmap_lidar3d.launch.py rviz:=true database_path:=~/maps/field.db
```

**Localise against a saved map (no Nav2):**
```bash
ros2 launch amiga_bringup slam_localization.launch.py database_path:=~/maps/field.db
```

**Run bag replay + SLAM:**
```bash
ros2 launch amiga_bringup slam_bag_replay.launch.py
ros2 launch amiga_bringup slam_bag_replay.launch.py bag_path:=/path/to/bag playback_rate:=0.5 database_path:=~/maps/field.db
```

**Save the built map (while SLAM is running):**
```bash
bash scripts/save_map.sh                     # saves to ~/maps/<timestamp>/
bash scripts/save_map.sh /data/field_map     # saves to specified directory
```

**Run tests:**
```bash
colcon test --packages-select amiga_bringup
colcon test-result --verbose
```

---

## Architecture

### TF Tree
```
map
 └── odom          (rtabmap_slam via ICP odometry or EKF)
      └── base_link (icp_odometry or ekf_filter_node)
           └── velodyne  (static: x=1.130 m, z=0.800 m, from robot_state_publisher)
```

### Full Autonomous Stack (localize_nav.launch.py)

```
Amiga brain (Tailscale: camphor-clone.tail0be07.ts.net)
    │  canbus service :6001  →  AmigaTpdo1 (wheel velocity, state)
    │  filter service :20001 →  FilterState (GPS+IMU pose, heading)
    │
    ▼  amiga_ros2_bridge  (native ROS 2 gRPC, farm-ng OS 2.0 SDK)
         /amiga/vel  (TwistStamped)   ← measured speed
         /amiga/pose (Odometry/world) ← GPS pose (when converged)
         /cmd_vel    (Twist)          → forwarded as Twist2d via request_reply("/twist")
    │
    ├── amiga_odometry      /amiga/vel → /wheel_odom (dead-reckoning)
    │
    ▼  Velodyne VLP-16  (192.168.1.201:2368)
         /velodyne_packets → /velodyne_points (PointCloud2, 10 Hz)
    │
    ├── icp_odometry        /velodyne_points → /odom  +  odom→base_link TF
    ├── rtabmap             localization-only (database loaded, map→odom TF)
    │                       publishes /map (OccupancyGrid) for Nav2 costmaps
    │
    └── Nav2
         controller_server   — local costmap (10 m rolling) + DWB local planner
         planner_server      — global costmap (full map) + NavFn/A* global planner
         behavior_server     — spin / back-up / wait recoveries
         bt_navigator        — Navigate-to-Pose / Navigate-Through-Poses actions
         waypoint_follower   — /follow_waypoints action (field row traversal)
         lifecycle_manager   — activates all Nav2 nodes
```

### Node descriptions
1. **`amiga_ros2_bridge`** — native gRPC bridge; OS 2.0 SDK; subscribe canbus via `config.subscriptions[0]` + 20s first-message timeout; filter via `decode=True`
2. **`amiga_odometry`** — `/amiga/vel` → `/wheel_odom`; midpoint RK2 integration; soil covariances
3. **`amiga_velocity_bridge`** — fallback `/cmd_vel` relay; only needed without gRPC bridge
4. **`velodyne_driver_node`** — UDP packets from VLP-16 at `192.168.1.201:2368`
5. **`velodyne_transform_node`** — packets → `PointCloud2` on `/velodyne_points`
6. **`icp_odometry`** — ICP on pointcloud; publishes `odom → base_link` TF at ~10 Hz
7. **`rtabmap`** — 3D map, `map → odom` TF; mapping or localization-only mode
8. **`controller_server`** — DWB local planner; max 1.0 m/s, 0.8 rad/s; 30 cm goal tolerance
9. **`planner_server`** — NavFn/A* global planner; 10 cm costmap resolution
10. **`behavior_server`** — spin / back-up / wait recoveries
11. **`bt_navigator`** — NavigateToPose and FollowWaypoints BT actions
12. **`waypoint_follower`** — executes ordered waypoint lists (field row traversal)

### Key Files

**Nodes (Python):**
- `src/amiga_bringup/amiga_bringup/amiga_ros2_bridge.py` — gRPC bridge (OS 2.0); publishes `/amiga/vel`, `/amiga/pose`; forwards `/cmd_vel`
- `src/amiga_bringup/amiga_bringup/amiga_odometry.py` — wheel odometry
- `src/amiga_bringup/amiga_bringup/amiga_velocity_bridge.py` — fallback cmd_vel relay

**Launch files (active):**
- `src/amiga_bringup/launch/localize_nav.launch.py` — **PRODUCTION**: Amiga + VLP-16 + localization + Nav2
- `src/amiga_bringup/launch/slam_nav.launch.py` — **NEW FIELD MAPPING**: SLAM + Nav2 combined
- `src/amiga_bringup/launch/slam_full.launch.py` — Amiga + VLP-16 + SLAM (no Nav2, for manual/teleoperation)
- `src/amiga_bringup/launch/nav2.launch.py` — Nav2 only (attach to running SLAM)
- `src/amiga_bringup/launch/slam_localization.launch.py` — RTAB-Map localization only
- `src/amiga_bringup/launch/amiga_grpc_bridge.launch.py` — gRPC bridge only
- `src/amiga_bringup/launch/amiga_bringup_nodes.launch.py` — odometry + velocity bridge (+ optional EKF)
- `src/amiga_bringup/launch/slam_rtabmap_lidar3d.launch.py` — SLAM stack only
- `src/amiga_bringup/launch/slam_bag_replay.launch.py` — bag replay + SLAM
- `src/amiga_bringup/launch/sensors_live.launch.py` — VLP-16 driver + static TF
- `src/amiga_bringup/launch/velodyne_vlp16.launch.py` — VLP-16 driver only

**Config:**
- `src/amiga_bringup/config/nav2_params.yaml` — Nav2: DWB controller, NavFn planner, costmaps, behaviors; tuned for 1.10×0.93 m outdoor robot
- `src/amiga_bringup/config/ekf.yaml` — robot_localization EKF (ICP + wheel odom fusion)
- `src/amiga_bringup/config/velodyne_transform.yaml` — VLP-16 pointcloud params
- `config/service_configs/canbus_config.json` — canbus gRPC service config (port 6001)
- `config/service_configs/filter_config.json` — filter gRPC service config (port 20001)

**URDF / RViz:**
- `src/amiga_bringup/urdf/amiga_min.urdf` — Amiga URDF; `base_link` at ground-level centre, `velodyne` at x=1.130 m, z=0.800 m
- `src/amiga_bringup/rviz/slam_lidar.rviz` — RViz2 config: TF, cloud, map, occupancy grid, odometry, trajectory, robot model

**Scripts:**
- `scripts/install_farmng.sh` — install farm-ng SDK into system Python (run once before build)
- `scripts/test_canbus.py` — standalone diagnostic for canbus gRPC service; `--scan` probes ports 6001/6002/50051
- `scripts/save_map.sh` — copies `~/.ros/rtabmap.db` to timestamped dir; generates `localise.sh`
- `scripts/build.sh`, `scripts/env.sh`, `scripts/run_live.sh`

---

## Physical Calibration (completed 2025-03-05)

`base_link → velodyne` static TF from physical measurements:

| Parameter | Value | Notes |
|-----------|-------|-------|
| x | **1.130 m** | 44.5 in forward from wheelbase midpoint to LiDAR spin axis |
| y | **0.000 m** | Centred on robot centreline |
| z | **0.800 m** | LiDAR spin axis height above ground |
| roll/pitch/yaw | **0 rad** | Mount is level; LiDAR front aligns with robot +x |

Set identically in both:
1. `src/amiga_bringup/launch/tf_static_base_to_velodyne.launch.py`
2. `src/amiga_bringup/urdf/amiga_min.urdf` (`base_to_velodyne` joint origin)

Amiga geometry:
- Robot footprint: 1.10 m (L) × 0.93 m (W), height 0.76 m
- `base_link`: centre of footprint at ground level (z=0)
- Nav2 footprint polygon (nav2_params.yaml): `[[0.55,0.465],[-0.55,0.465],[-0.55,-0.465],[0.55,-0.465]]`

---

## Amiga gRPC Bridge

**Install farm-ng SDK once (system Python, not venv):**
```bash
bash ~/amiga_slam/scripts/install_farmng.sh
```

**OS 2.0 API notes:**
- `AmigaTpdo1` and `Twist2d` live in `farm_ng.canbus.packet`
- Subscribe canbus via `client.subscribe(config.subscriptions[0], decode=False)` — same pattern as filter_client.py
- Decode: `payload_to_protobuf(event, payload)` → `AmigaTpdo1.from_proto(message.amiga_tpdo1)`
- Commands: `await client.request_reply("/twist", Twist2d(...))` in the subscribe loop
- Filter: `decode=True`, same as filter_client.py

**Diagnose canbus connectivity:**
```bash
python3 scripts/test_canbus.py                   # test port 6001
python3 scripts/test_canbus.py --scan            # probe all common ports
python3 scripts/test_canbus.py --port 6001 --decode  # show AmigaTpdo1 fields
```

---

## Amiga Topics (ROS 2)

| Topic | Type | Notes |
|-------|------|-------|
| `/amiga/vel` | `TwistStamped` | Measured wheel velocity → consumed by `amiga_odometry` |
| `/amiga/pose` | `Odometry` (frame: world) | GPS+IMU pose (when filter converged) |
| `/wheel_odom` | `Odometry` | Dead-reckoning from `amiga_odometry` |
| `/cmd_vel` | `Twist` | Nav2 / teleop → forwarded to Amiga as Twist2d |
| `/odom` | `Odometry` | ICP odometry from `icp_odometry` → Nav2 uses this |
| `/odometry/filtered` | `Odometry` | EKF-fused (optional, `use_ekf:=true`) |
| `/map` | `OccupancyGrid` | Published by RTAB-Map → Nav2 global costmap static layer |

---

## Nav2 Configuration Notes

**Costmap obstacle detection:**
- Both local and global costmaps use `/velodyne_points` (PointCloud2)
- `min_obstacle_height: 0.10` — ignores ground plane returns (VLP-16 at z=0.80 m)
- `max_obstacle_height: 2.0` — ignores returns above 2 m
- `obstacle_range: 5.0 m`, `raytrace_range: 8.0 m`
- Resolution: 10 cm cells; local window: 10 m × 10 m rolling

**Controller (DWB):**
- Max 1.0 m/s linear, 0.8 rad/s angular (bridge hard-clamps at 1.5 m/s, 1.0 rad/s)
- Goal tolerance: 30 cm xy, 11° yaw — appropriate for field work
- Sim time: 2.0 s horizon

**Planner (NavFn A\*):**
- `allow_unknown: true` — plans into unexplored areas (useful during mapping)
- `use_astar: true` — more reliable than Dijkstra for large outdoor maps

**To tune Nav2 without rebuilding:**
Since `config/nav2_params.yaml` is installed with `--symlink-install`, edit it and restart Nav2.

---

## RTAB-Map Notes

**LiDAR-only mode** requires on both `rtabmap` and `rtabmap_viz`:
```python
'subscribe_scan_cloud': True,
'subscribe_rgb': False,
'subscribe_depth': False,
```

**rtabmap_viz bag replay warning**: set `subscribe_scan_cloud: False` in rtabmap_viz — it should only visualise rtabmap output, not raw scans (avoids TF extrapolation warnings).

**Expected SLAM output**: ICP odometry ~10 Hz, rtabmap ~1 Hz. `WM=1-3` in logs is normal.

---

## Velodyne Calibration File

`velodyne_transform_node` requires an absolute path for the calibration file:
```yaml
# config/velodyne_transform.yaml
calibration: /opt/ros/humble/share/velodyne_pointcloud/params/VLP16db.yaml
```
A bare filename or empty string crashes the node.

---

## Redundant Launch Files (legacy, do not use)

- `static_tf.launch.py`, `tf_static.launch.py` — superseded by `tf_static_base_to_velodyne.launch.py`
- `sensors_vlp16.launch.py`, `bringup_sensors.launch.py` — superseded by `sensors_live.launch.py`
