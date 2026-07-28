# maps/ — prior maps for ERC localization

This directory is the **canonical home for prior LiDAR maps** (`.pcd`) used by the
localization / odometry launches. All of these read their map from here by default:

- `localize.launch.py`, `odometry.launch.py`, `place_waypoints.launch.py`,
  `localize_demo.launch.py`  →  default `pcd:=~/real_ws/src/ares_erc_bringup/maps/prior_map.pcd`

## The active map

**`prior_map.pcd`** is the *active* prior map (what the launches load with no `pcd:=`).
Put the map you want to localize against here under that name.

Two ways it gets populated:

1. **Self-made (scan the environment):** FAST-LIO writes the scan to
   `FAST_LIO_ROS2/PCD/scans.pcd` on Ctrl-C. After you verify the scan looks right,
   *promote* it to the active prior map through `downsample_map` (do **not** just `cp` it
   raw — FAST-LIO's accumulated cloud is tens of millions of points, which `pcd_to_pointcloud`
   then has to hold/render whole for the RViz `/erc/prior_map` display and can OOM-kill RViz;
   `map_anchor`'s own GICP re-downsamples to `voxel_leaf` — 0.3 m by default — regardless, so
   nothing downstream needs the raw resolution):
   ```bash
   ros2 run ares_erc_bringup downsample_map \
     ~/real_ws/src/FAST_LIO_ROS2/PCD/scans.pcd \
     ~/real_ws/src/ares_erc_bringup/maps/prior_map.pcd \
     0.05   # leaf size [m], optional (default 0.05 = 5 cm; keeps a room visually detailed)
   ```
2. **Externally provided map** (e.g. an ERC-supplied `.pcd`): just drop it here as
   `prior_map.pcd`:
   ```bash
   cp /path/to/given_map.pcd ~/real_ws/src/ares_erc_bringup/maps/prior_map.pcd
   ```
   Requirements for an external map to work: a PCL-readable `.pcd` whose **origin (0,0,0)
   is the start point** and whose **+Z is up (gravity-aligned)** — because the ERC datum
   frame is start-relative and `map_anchor` seeds GICP from near-identity. If the given
   map uses a different origin/orientation, set `init_xyz` / `init_yaw` in
   `config/localization.yaml` to the start pose expressed in that map's frame.

## Keeping several environments

Keep per-site maps under descriptive names and select one without renaming:
```
maps/
  prior_map.pcd     # active (loaded by default)
  lab_room.pcd
  marsyard.pcd
```
```bash
ros2 launch ares_erc_bringup odometry.launch.py \
  pcd:=$HOME/real_ws/src/ares_erc_bringup/maps/marsyard.pcd
```
or make `marsyard.pcd` the active one: `cp maps/marsyard.pcd maps/prior_map.pcd`.

## Git

`.pcd` files here are **git-ignored** (`**/maps/*.pcd`) — they are large and
machine/site-specific. Only this `README.md` is tracked. Back maps up out-of-band
(USB / shared drive), not in the repo.

See `../../doc/erc_new_environment_runbook.md` for the full new-environment procedure.
