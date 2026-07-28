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

Do not compare `map -> camera_init` directly between separate launches as if it were
the rover pose. `camera_init` is FAST-LIO's session-local world and its unobservable yaw
gauge can change on restart. Use the composed `map -> body` (or `/erc/odometry`) for
cross-session accuracy tests and Nav2.

**Tuning** (`config/localization.yaml`): `voxel_leaf` (map/scan downsample), `fitness_max`
(reject bad matches), `max_trans_step`/`max_rot_step` (reject jumps), `lowpass_alpha`
(smaller = smoother), `init_xyz`/`init_yaw` (initial guess if you don't start exactly at
the mapped start point). `planar_correction: true` projects GICP rotation to yaw because
both FAST-LIO world frames are gravity aligned; this prevents a locally-good 3-D fit
from tilting the map and breaking Nav2's planar costmaps.

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

## Phase 3 — ArUco global initialization and drift correction

`aruco_localize.launch.py` adds global initialization to the local GICP stack. The
detector publishes every marker in a frame, the ERC adapter normalizes the shared
detector's mixed optical/camera convention, and publishes one TF per ID
(`aruco_marker_51`, `aruco_marker_52`, ...). With two or more well-separated known
markers, `aruco_map_anchor.py` computes an absolute `map→camera_init` candidate. Three
consistent candidates initialize `map_anchor`; only then does local GICP refinement
start. This allows a run to start away from the pose used to scan the prior map.

The fixed `map→datum` transform must already be calibrated for arbitrary-start
localization (`datum.calibrated: true`). During a run, **`aruco_map_anchor` owns
`map→datum`** and `map_anchor` remains the only publisher of `map→camera_init`; ArUco and
GICP are two correction inputs, not competing TF publishers.

Fill in before a live run:
- `config/aruco_anchors_erc.yaml` — known markers `id → datum (x,y,z)`, gates, update tuning.
- `config/extrinsics_erc.yaml` — measured `body → camera_link` (the accuracy ceiling, §6.4).
- marker size / dictionary: match the real tag. The ERC launch defaults to the Rev.3
  values used here: `0.150 m`, `5X5_100`.

**Offline (no camera / no LiDAR) — watch the correction converge:**
```bash
ros2 launch ares_erc_bringup aruco_anchor_demo.launch.py     # true 30 deg / 0.2,-0.1 by default
#   true_yaw_deg:=45.0  true_xy:="[0.3, 0.2]"                 # optional ground truth
```
`aruco_fake_detections` injects synthetic detections; in RViz (Fixed Frame `map`) the yellow
known-marker cubes (datum) slide onto the blue observed spheres (map) as `map→datum` converges.
Verified headless: from a 0° seed it reached yaw 29.6° / (0.197, −0.099) with residual ≈ 6e-16.

**Offline arbitrary-start self-test (no camera / no LiDAR):**
```bash
ros2 launch ares_erc_bringup aruco_global_init_demo.launch.py
ros2 run tf2_ros tf2_echo map camera_init
# expected: x≈1.0 m, y≈2.0 m, yaw≈10 deg
ros2 topic echo /erc/localization_initialized --once
# expected: data: true
```

**Live (D435i connected):**
```bash
ros2 launch ares_erc_bringup aruco_localize.launch.py
# If fewer than two known tags are available and the run starts at the mapped pose:
# ros2 launch ares_erc_bringup aruco_localize.launch.py global_init:=false
```
With the default `global_init:=true`, GICP intentionally waits until a trustworthy
multi-marker absolute pose is accepted. A single marker cannot initialize planar
position and yaw robustly. After initialization, however, one known marker supplies an
x/y-only correction while the current FAST-LIO/GICP yaw is preserved. Three independent
same-ID camera frames must agree before that correction is fused. Two or more visible
markers continue to supply the full x/y/yaw correction, and zero visible markers falls
back to FAST-LIO + GICP.

**Check:**
```bash
ros2 topic echo /aruco_detections --once       # one message should contain both IDs
ros2 run tf2_ros tf2_echo camera_color_optical_frame aruco_marker_51
ros2 run tf2_ros tf2_echo camera_color_optical_frame aruco_marker_52
ros2 topic echo /erc/localization_initialized --once
ros2 run tf2_ros tf2_echo map camera_init
ros2 topic echo /erc/localization_fitness
```

The desired tag-frame axes are the solvePnP object axes: viewed from the printed face,
`+x` points right, `+y` up, and `+z` toward the camera. Detection range, fit residual,
two-tag baseline, 3-D tag-spacing consistency, jump, and temporal-consistency gates
reject weak solutions. The 3-D spacing check is independent of robot pose and rigid
camera-LiDAR extrinsics, so bad hand-measured coordinates or marker scale cannot silently
become a global correction. Once initialized, losing the tags does not erase the pose;
GICP continues local refinement. Runtime behavior is therefore `0 tags = LiDAR/IMU`,
`1 known tag = translation aid`, and `2+ known tags = full planar landmark aid`.

### Survey indoor tags into the prior map (recommended calibration)

Do not derive localization landmark coordinates by measuring from an arbitrarily
placed LiDAR or by mixing prior-map and datum axes. Start the rig at the pose where the
prior map was scanned (the pose where GICP is already known to overlap well), keep the
rig and tags stationary, and run:

```bash
ros2 launch ares_erc_bringup aruco_calibrate.launch.py
```

The calibrator accepts samples only while localization is initialized and GICP fitness
is at most `0.02`. After 15 seconds it robustly rejects position outliers and writes:

```text
/tmp/aruco_anchors_measured.yaml
```

Inspect the per-ID `raw_samples`, `kept_samples`, `radial_mad_m`, and
`max_kept_residual_m`. Do not copy it into runtime config if either tag has fewer than
30 retained samples or centimetre-scale residuals. A safe first inspection is:

```bash
sed -n '1,200p' /tmp/aruco_anchors_measured.yaml
```

The generated file has `coordinate_frame: map`; pass it without overwriting the
checked-in fallback:

```bash
ros2 launch ares_erc_bringup aruco_localize.launch.py \
  anchors_file:=/tmp/aruco_anchors_measured.yaml
```

Then move to a substantially different starting pose, keep both tags visible, and
verify `/erc/localization_initialized`, `map -> camera_init`, and the grey/red cloud
overlap. The calibration is valid only while the camera-to-LiDAR mounting is unchanged.

`localize.launch.py` by itself is the LiDAR-only, local-registration mode. It assumes
the initial guess in `localization.yaml` is already close enough for GICP.
`aruco_localize.launch.py` is the intended ERC arbitrary-start mode.

The complete implemented node/topic graph, TF ownership, tag-count state transitions,
and correction rejection path are documented in
[`doc/erc_localization_graph.md`](../doc/erc_localization_graph.md).

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
