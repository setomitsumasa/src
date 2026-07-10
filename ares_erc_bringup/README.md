# ares_erc_bringup

ERC (European Rover Challenge) GPS-free autonomy bring-up for the ARES rover.

This package holds **ERC-specific launch/config**. The heavy lifting is done by
unmodified upstream packages:
- **`fast_lio`** — FAST-LIO2 (Ericsii/FAST_LIO_ROS2, `ros2` branch), vendored at `../FAST_LIO_ROS2`.
- **`livox_ros_driver2`** — Livox Mid-360 driver (already in the workspace).

> URC (GNSS) stack is untouched. Do **not** run this together with the URC bring-up:
> FAST-LIO owns `map → odom (→ body)`, which must stay a single TF authority (CLAUDE.md §6).

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
