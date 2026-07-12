# ares_erc_bringup

ERC (European Rover Challenge) GPS-free autonomy bring-up for the ARES rover.

This package holds **ERC-specific launch/config**. The heavy lifting is done by
unmodified upstream packages:
- **`fast_lio`** — FAST-LIO2 (Ericsii/FAST_LIO_ROS2, `ros2` branch), vendored at `../FAST_LIO_ROS2`.
- **`livox_ros_driver2`** — Livox Mid-360 driver (already in the workspace).

> URC (GNSS) stack is untouched. Do **not** run this together with the URC bring-up:
> FAST-LIO owns `map → odom (→ body)`, which must stay a single TF authority (CLAUDE.md §6).

**Prior map location:** the localization/odometry launches load their map from
`maps/prior_map.pcd` by default (self-made *or* externally provided). See
[`maps/README.md`](maps/README.md) for the storage convention and
[`../doc/erc_new_environment_runbook.md`](../doc/erc_new_environment_runbook.md) for the
full "scan a new environment → localize + odometry" procedure.

---

## Phase 0 — Mid-360 + FAST-LIO2 mapping → saved `.pcd`

Goal: drive/walk the Mid-360 around, build a 3D LiDAR map with FAST-LIO2, and save it as `.pcd`.
Needs only **Mid-360 + LAN cable + mini PC** (no D435i, no mounting jig).

### 0. Network bring-up (one-time, do this first)
The Mid-360 talks over Ethernet. The host PC's wired NIC must be on the LiDAR's subnet
and its IP must match `host_net_info` in
`livox_ros_driver2/config/MID360_config.json` (currently host `192.168.1.50`, lidar `192.168.1.164`).

1. Plug the Mid-360 into a wired NIC (`enp2s0` or `eno1` on this PC).
2. Set that NIC to a static IP `192.168.1.50/24`, e.g.:
   ```bash
   nmcli con add type ethernet ifname enp2s0 con-name livox \
     ipv4.method manual ipv4.addresses 192.168.1.50/24
   nmcli con up livox
   ```
3. Find the Mid-360's real IP (pick one):
   - serial sticker on the unit → IP is `192.168.1.1XX` (XX = last 2 serial digits),
   - Livox Viewer 2 (auto-discovers), or
   - `sudo tcpdump -i enp2s0 -n` and watch for the device's packets.
4. If it differs from `192.168.1.164`, update `lidar_configs[0].ip` in `MID360_config.json`
   and rebuild `livox_ros_driver2`.
5. Verify reachability: `ping 192.168.1.164`.

> Until step 0 is done, the driver prints `bind failed → Init lds lidar fail!` — expected with no sensor/NIC.

### 1. Build
```bash
cd ~/real_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select fast_lio ares_erc_bringup --symlink-install
source install/setup.bash
```

### 2. Sensor sanity check (optional but recommended)
```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
# in another shell:
ros2 topic hz /livox/lidar    # expect ~10 Hz, type livox_ros_driver2/msg/CustomMsg
ros2 topic hz /livox/imu      # high rate, sensor_msgs/msg/Imu
```

### 3. Run mapping
```bash
ros2 launch ares_erc_bringup mapping_mid360.launch.py       # rviz on by default
# or headless:  ros2 launch ares_erc_bringup mapping_mid360.launch.py rviz:=false
```
Walk / drive the sensor slowly around the area (revisit start to close loops).

### 4. Save the map
`pcd_save_en` is **true** in `config/mid360_mapping.yaml`, so on **Ctrl-C** FAST-LIO writes the
full map to:
```
FAST_LIO_ROS2/PCD/scans.pcd
```
(The save path is hardcoded to FAST-LIO's `ROOT_DIR`; the `map_file_path` param does not control it.)

---

## How to tell it's working (verification)
| Check | Expect |
|---|---|
| `ros2 topic hz /livox/lidar` / `/livox/imu` | ~10 Hz / high rate |
| `ros2 topic list` | `/Odometry`, `/cloud_registered`, `/path` present |
| `ros2 run tf2_tools view_frames` | FAST-LIO publishes `camera_init → body` (single authority) |
| RViz | `/cloud_registered` accumulates into a coherent map; `/Odometry` trajectory drifts little and closes on loop; walls/objects are not doubled |
| `FAST_LIO_ROS2/PCD/scans.pcd` | opens in `pcl_viewer` and matches the real site |

## Phase 1 (1a) — locate the ERC waypoints on the scanned map

ERC gives 4 waypoints (+finish) in a **datum** frame: origin = start point, start-relative
Cartesian [m] (NOT GNSS). FAST-LIO's `camera_init` origin IS the start point, so the datum
maps to `camera_init` by a **pure yaw** (`datum.yaw_offset_deg`, 0 = datum axes == initial heading).

Edit the points in `config/waypoints_erc.yaml` (measure spots in your room relative to the
start pose: x = forward, y = left). Then:

**Offline (no hardware) — see where the 4 points land on a saved map:**
```bash
ros2 launch ares_erc_bringup place_waypoints.launch.py           # uses PCD/scans.pcd
#   pcd:=/path/to/other.pcd   waypoints_file:=/path/to/wp.yaml   # optional overrides
```
RViz shows the prior map (`/erc/prior_map`) + green waypoint spheres / red FINISH + labels.

**Live — overlay waypoints on a live map:**
```bash
ros2 launch ares_erc_bringup mapping_mid360.launch.py            # terminal A
ros2 launch ares_erc_bringup erc_waypoints.launch.py             # terminal B
```

**Check:** `ros2 topic echo /erc/waypoint_poses` prints each point (frame `datum`);
`ros2 run tf2_ros tf2_echo camera_init datum` shows the datum yaw. Set a waypoint to a known
feature (e.g. a table corner 2 m ahead) and confirm the marker sits on it in the cloud.
Node/topics: `erc_waypoints.py` → static TF `camera_init→datum`, `/erc/waypoint_markers`
(MarkerArray), `/erc/waypoint_poses` (PoseArray, for the Phase-2 Nav2 follower).

## Phase 1b — relocalize on the prior map (`map_anchor`)

Goal: during a run, don't just re-map (which drifts). Instead **localize on the saved
`.pcd`** so the pose stays globally consistent in a fixed `map` frame (CLAUDE.md §6.3).

**How it works.** `map_anchor` (C++, PCL GICP) loads the prior map, GICP-aligns the live
FAST-LIO cloud (`/cloud_registered`, in `camera_init`) onto it, and broadcasts a
low-passed **`map → camera_init`** correction. TF authority stays single:

| TF | Publisher |
|---|---|
| `map → camera_init` (odom) | **`map_anchor`** (this) |
| `camera_init → body` | FAST-LIO |
| `map → datum` (yaw) | `erc_waypoints.py` |

So the chain is `map → camera_init(odom) → body`, and waypoints (Phase 1a) live in `map`.

**Run (live, needs the Mid-360):**
```bash
ros2 launch ares_erc_bringup localize.launch.py     # Mid-360 + FAST-LIO + anchor + RViz
#   pcd:=/path/to/prior.pcd   loc_config:=/path/to/localization.yaml   # optional overrides
```
In RViz (Fixed Frame = `map`): the **prior map (grey, fixed)** and the **live
`/cloud_registered` (red)** should **overlap** = localized. Walk the sensor around — the
live cloud stays stuck to the prior map instead of drifting away, and the waypoint markers
stay on their real features. Stop and restart to confirm cross-session relocalization
(the ERC "map in prep, localize in the run" workflow).

**Monitor:**
```bash
ros2 run tf2_ros tf2_echo map camera_init      # the GICP correction
ros2 run tf2_ros tf2_echo map body             # global pose
ros2 topic echo /erc/localization_fitness      # GICP fitness (lower = better lock)
ros2 run tf2_tools view_frames                 # confirm map->camera_init has ONE publisher
```

**Tuning** (`config/localization.yaml`): `voxel_leaf` (map/scan downsample), `fitness_max`
(reject bad matches), `max_trans_step`/`max_rot_step` (reject jumps), `lowpass_alpha`
(smaller = smoother), `init_xyz`/`init_yaw` (initial guess if you don't start exactly at
the mapped start point).

**Offline self-test (no hardware):** feed the prior map back in as `/cloud_registered` and
start the anchor with a deliberate offset — GICP must pull it to ~0 with ~0 fitness:
```bash
PCD=~/real_ws/src/FAST_LIO_ROS2/PCD/scans.pcd
ros2 run pcl_ros pcd_to_pointcloud --ros-args -p file_name:=$PCD \
  -p tf_frame:=camera_init -p publishing_period_ms:=1000 -r cloud_pcd:=/cloud_registered &
ros2 run ares_erc_bringup map_anchor --ros-args -p prior_map_path:=$PCD \
  -p init_xyz:="[0.3, 0.0, 0.0]" -p voxel_leaf:=0.5    # watch: tf2_echo map camera_init -> x:0.3 -> 0
```

## Phase 2 (2a) — live global odometry output + display

Phase 1b gives two live TF links: `map -> camera_init` (map_anchor GICP correction) and
`camera_init -> body` (FAST-LIO). Their composition `map -> body` is the robot's
**drift-corrected global pose** in the prior-map / datum frame. `erc_odometry.py` looks
that up from TF and republishes it (it publishes **no** TF, so the single-authority rule
holds):
- `/erc/odometry` — `nav_msgs/Odometry`, `frame_id=map`, `child_frame_id=body` (pose +
  finite-difference twist in the body frame). This is what Nav2 will consume in 2b.
- `/erc/trajectory` — `nav_msgs/Path`, the accumulated path for RViz.

**Live (Mid-360 connected):**
```bash
ros2 launch ares_erc_bringup odometry.launch.py
```
Runs the whole Phase-1b stack (RViz off) + `erc_odometry` + one RViz (`erc_odometry.rviz`,
Fixed Frame `map`): prior map (grey) + live cloud (red) + waypoints + a **yellow pose
arrow** (`/erc/odometry`) + a **cyan trajectory** (`/erc/trajectory`). Move the sensor and
watch the arrow/trajectory track on the map.

**Check:**
```bash
ros2 topic echo /erc/odometry --once     # frame_id=map, child_frame_id=body
ros2 topic hz /erc/odometry              # ~30 Hz
```
Verified offline with a synthetic circular `map->body` (r=2 m, ω=0.4 rad/s): odometry
reported forward speed 0.80 m/s and yaw rate 0.40 rad/s (== r·ω and ω), trajectory
accumulated correctly.

## Troubleshooting
- **`bind failed` / `Init lds lidar fail!`** → network step 0 not done (NIC IP ≠ host IP, or sensor unreachable).
- **`Failed to find match for field 'time'`** → wrong Livox message format; use CustomMsg (this launch already does).
- **Cone-shaped / smeared cloud in RViz, but the saved map looks fine** → RViz **Fixed Frame** is wrong. Set Global Options → Fixed Frame to **`camera_init`** (not `body`). `body` views from the moving sensor, so the world smears.
- **Fine when still, but drifts violently the instant you MOVE** → LiDAR↔IMU rotation mismatch. Root cause here was the rover-mount `pitch/yaw = 180/180` in `livox_ros_driver2/config/MID360_config.json`, applied to the point cloud but NOT the IMU. **Fixed** by this package's `config/MID360_config_erc.json` (driver extrinsic zeroed); the launch uses it automatically. If it ever recurs, verify the driver config in use has a zero extrinsic, and as a fallback set `extrinsic_est_en: false` in `mid360_mapping.yaml` (with the known Mid-360 `extrinsic_T`).
- **IMU accel unit (g vs m/s²)** → verified OK on this unit (`/livox/imu` z ≈ 9.8 m/s²). If a future driver outputs ~1.0, that's g-units (CLAUDE.md §11).

## Patch applied to vendored FAST-LIO (important)
The upstream `Ericsii/FAST_LIO_ROS2` (`ros2` branch) **commented out** the full-map
accumulator (`pcl_wait_save`) in `src/laserMapping.cpp` (~L516/L543), which left the
end-of-run save writing an empty cloud → **no `scans.pcd`**. We re-enabled that block
(marked `// [ARES]`) so `Ctrl-C` reliably saves the full map. **If you ever re-clone
FAST_LIO_ROS2, re-apply this** (uncomment the `if (pcd_save_en) { ... }` block whose body
appends `*pcl_wait_save += *laserCloudWorld;`) and rebuild `fast_lio`.

Verified: a ~24 s stationary scan produced `PCD/scans.pcd` = ~23 MB / 731k points.

## Files
- `config/mid360_mapping.yaml` — FAST-LIO2 params (topics, Mid-360 LiDAR↔IMU extrinsic, pcd save).
- `config/MID360_config_erc.json` — Livox driver config for ERC: identical network settings to the
  workspace default but with the driver **extrinsic zeroed** (see the divergence note above).
- `launch/mapping_mid360.launch.py` — starts Livox driver (with the ERC config) + FAST-LIO2 (+ RViz).
