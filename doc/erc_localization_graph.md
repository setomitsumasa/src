# ERC LiDAR–RealSense localization graph

This document describes the ROS 2 graph implemented for the ERC 2026 Traverse
localization experiments. It follows the code launched by:

```bash
ros2 launch ares_erc_bringup aruco_localize.launch.py \
  anchors_file:=<map-coordinate ArUco anchors.yaml> \
  global_init:=true
```

The design has one TF authority per edge. FAST-LIO supplies smooth local motion,
prior-map GICP supplies geometric global correction, and known ArUco landmarks supply
sparse absolute correction. GICP and ArUco are inputs to the same `map_anchor` node;
they do not publish competing `map -> camera_init` transforms.

## Runtime data flow

```mermaid
flowchart LR
  subgraph LiDAR["Mid-360 / FAST-LIO"]
    M360["Livox Mid-360"]
    LIV["livox_lidar_publisher"]
    FLIO["laser_mapping<br/>FAST-LIO2"]
    M360 --> LIV
    LIV -- "/livox/lidar<br/>CustomMsg" --> FLIO
    LIV -- "/livox/imu" --> FLIO
  end

  subgraph Camera["RealSense / ArUco"]
    D435["RealSense D435i"]
    RS["realsense_publisher"]
    DET["aruco_tracker"]
    ADAPT["aruco_detection_adapter"]
    AANCHOR["aruco_map_anchor"]
    D435 --> RS
    RS -- "/camera/color/image_raw" --> DET
    RS -- "/camera/color/camera_info" --> DET
    DET -- "/aruco_detections_raw" --> ADAPT
    ADAPT -- "/aruco_detections<br/>all IDs in the same image" --> AANCHOR
  end

  PCD[("prior_map.pcd")]
  MAPANCHOR["map_anchor<br/>GICP + ArUco fusion"]
  PRIORPUB["prior_map_publisher"]
  RVIZ["rviz2"]
  NAV["Nav2 / downstream consumer"]

  FLIO -- "/cloud_registered<br/>frame: camera_init" --> MAPANCHOR
  FLIO -. "TF camera_init -> body" .-> NAV
  PCD --> MAPANCHOR
  PCD --> PRIORPUB
  PRIORPUB -- "/erc/prior_map<br/>frame: map" --> RVIZ

  AANCHOR -- "/erc/aruco_camera_init_candidate" --> MAPANCHOR
  MAPANCHOR -- "/erc/localization_initialized" --> AANCHOR
  MAPANCHOR -- "/erc/localization_fitness" --> RVIZ
  MAPANCHOR -. "TF map -> camera_init" .-> NAV
  FLIO -- "/cloud_registered" --> RVIZ

  ADAPT -. "TF optical -> aruco_marker_ID" .-> RVIZ
  AANCHOR -- "/erc/aruco_known_markers<br/>/erc/aruco_observed" --> RVIZ
  NAV -- "composed map -> body" --> RVIZ
```

## TF tree and authority

```mermaid
flowchart LR
  MAP["map<br/>prior-map global frame"]
  CI["camera_init<br/>FAST-LIO session-local frame"]
  BODY["body<br/>rover / LiDAR-IMU body"]
  CAM["camera_link"]
  COLOR["camera_color_optical_frame"]
  DEPTH["camera_depth_optical_frame"]
  TAG["aruco_marker_ID"]
  DATUM["datum<br/>ERC waypoint frame"]

  MAP -- "dynamic<br/>map_anchor only" --> CI
  CI -- "dynamic<br/>FAST-LIO only" --> BODY
  BODY -- "static<br/>extrinsics_erc.yaml" --> CAM
  CAM -- "static<br/>realsense_publisher" --> COLOR
  CAM -- "static<br/>realsense_publisher" --> DEPTH
  COLOR -- "dynamic per detection<br/>aruco_detection_adapter" --> TAG
  MAP -- "fixed/calibrated<br/>aruco_map_anchor in ArUco mode" --> DATUM
```

The rover's global pose is the composed `map -> body` transform:

```text
T_map_body = T_map_camera_init * T_camera_init_body
```

`map -> camera_init` must not be interpreted as the rover pose. It aligns the current
FAST-LIO session frame with the saved prior map and can change as GICP or ArUco
corrections are fused.

## Landmark behavior while driving

```mermaid
stateDiagram-v2
  [*] --> WaitingForGlobalInit
  WaitingForGlobalInit --> WaitingForGlobalInit: 0 or 1 known tag
  WaitingForGlobalInit --> Initialized: 2+ known tags,\nadequate baseline,\n3 consistent candidates

  Initialized --> NoTag: no known tag visible
  Initialized --> OneTag: one known tag visible
  Initialized --> MultiTag: 2+ known tags visible
  NoTag --> OneTag: any YAML-known ID enters view
  NoTag --> MultiTag: multiple known IDs enter view
  OneTag --> NoTag: tag leaves view
  OneTag --> MultiTag: another known tag enters view
  MultiTag --> OneTag: only one known tag remains

  NoTag: FAST-LIO + GICP
  OneTag: x/y ArUco correction\ncurrent yaw retained
  MultiTag: full planar x/y/yaw\nArUco correction
```

After initialization, a known tag does **not** need to have been visible at startup.
History is stored independently by marker ID. If a previously unseen, YAML-known ID
enters the camera view while driving, three consistent same-ID camera frames enable its
translation-only correction. A simultaneous overlap with the old tag is not required.

A tag without a known map coordinate cannot provide an absolute correction on its first
observation. It may be registered as a new landmark for later loop closure, but its
initial stored coordinate inherits the localization error present at registration time;
that online-registration feature is not part of the current runtime.

## Main nodes and interfaces

| Node | Subscribes / input | Publishes / authority | Purpose |
|---|---|---|---|
| `livox_lidar_publisher` | Mid-360 Ethernet data | `/livox/lidar`, `/livox/imu` | Livox ROS driver |
| `laser_mapping` | `/livox/lidar`, `/livox/imu` | `/cloud_registered`, `/Odometry`, TF `camera_init -> body` | FAST-LIO local LiDAR-inertial motion |
| `realsense_publisher` | D435i USB streams | `/camera/color/image_raw`, `/camera/depth/image_raw`, `/camera/color/camera_info`, optical static TF | RGB/depth and calibrated camera information |
| `aruco_tracker` | color image and camera info | `/aruco_detections_raw` | OpenCV ArUco detection and PnP |
| `aruco_detection_adapter` | `/aruco_detections_raw` | `/aruco_detections`, TF `camera_color_optical_frame -> aruco_marker_<id>` | Correct detector frame convention and preserve all simultaneous IDs |
| `aruco_map_anchor` | `/aruco_detections`, TF, `/erc/localization_initialized`, anchor YAML | `/erc/aruco_camera_init_candidate`, `/erc/aruco_anchor_residual`, known/observed markers, TF `map -> datum` | Match known IDs to map coordinates and form gated correction candidates |
| `map_anchor` | `/cloud_registered`, prior-map PCD, `/erc/aruco_camera_init_candidate` | TF `map -> camera_init`, `/erc/localization_initialized`, `/erc/localization_fitness` | Single fusion/TF authority for GICP and ArUco corrections |
| `prior_map_publisher` | prior-map PCD | `/erc/prior_map` | Publish the saved map for RViz and diagnostics |
| `erc_waypoints` | waypoint YAML | `/erc/waypoint_markers`, `/erc/waypoint_poses` | Datum-frame mission waypoint output |
| `erc_odometry` | composed TF `map -> body` | `/erc/odometry`, `/erc/trajectory` | Optional Nav2-facing global odometry adapter |

## Correction and rejection path

```mermaid
flowchart TD
  OBS["corrected ArUco observation"]
  KNOWN{"ID exists in anchors YAML?"}
  RANGE{"range and view-angle gates pass?"}
  COUNT{"known tags in latest image"}
  CONS["same-ID temporal consensus<br/>minimum 3 independent frames"]
  FIT["multi-point SE(2) fit<br/>baseline + spacing + residual gates"]
  CAND["/erc/aruco_camera_init_candidate"]
  FUSION{"map_anchor gates"}
  APPLY["low-pass update<br/>map -> camera_init"]
  DROP["reject; FAST-LIO + GICP continue"]

  OBS --> KNOWN
  KNOWN -- no --> DROP
  KNOWN -- yes --> RANGE
  RANGE -- no --> DROP
  RANGE -- yes --> COUNT
  COUNT -- "1" --> CONS
  COUNT -- "2+" --> FIT
  CONS -- "x/y only; yaw unconstrained" --> CAND
  FIT -- "x/y/yaw" --> CAND
  CONS -- inconsistent --> DROP
  FIT -- inconsistent --> DROP
  CAND --> FUSION
  FUSION -- "initialized/residual/jump pass" --> APPLY
  FUSION -- fail --> DROP
```

Safety properties:

- One tag cannot initialize arbitrary global planar position and yaw.
- One tag after initialization corrects x/y only; it cannot overwrite yaw.
- Two or more tags must come from the exact same camera image.
- Pairwise 3-D tag spacing is checked before fitting, catching wrong YAML coordinates
  or marker scale independently of rover pose.
- Exact capture-time TF is used; using the latest TF while moving would introduce a
  velocity-times-latency bias.
- GICP fitness, ArUco residual, correction jump, range, viewing angle, baseline, and
  temporal-consistency gates reject unsafe corrections.

## Useful diagnostics

```bash
# Global initialization state
ros2 topic echo /erc/localization_initialized --once

# Prior-map registration quality; lower is better
ros2 topic echo /erc/localization_fitness

# Every marker detected in one camera frame
ros2 topic echo /aruco_detections --once

# Actual global rover pose and localization-frame correction
ros2 run tf2_ros tf2_echo map body
ros2 run tf2_ros tf2_echo map camera_init

# Confirm one-tag and multi-tag correction candidates
ros2 topic echo /erc/aruco_camera_init_candidate

# Verify that each TF edge has a single publisher
ros2 run tf2_tools view_frames
```

## Configuration files

- `config/mid360_mapping.yaml`: FAST-LIO Mid-360/IMU configuration.
- `config/MID360_config_erc.json`: Livox driver configuration without the conflicting
  driver-side 180-degree extrinsic.
- `config/localization.yaml`: GICP and candidate-fusion gates.
- `config/extrinsics_erc.yaml`: rigid `body -> camera_link` transform. The current
  zero placeholder must be replaced by a formal RealSense–Mid-360 calibration.
- `config/aruco_anchors_erc.yaml`: ERC landmark IDs, map/datum coordinates, and gates.
- `config/aruco_anchors_indoor_calibrated.yaml`: indoor test-only measured anchors.
- `maps/prior_map.pcd`: generated prior map. PCD files are intentionally excluded from
  Git because they are large environment-specific artifacts.

## Current validation status

Indoor hardware testing has confirmed:

- simultaneous ID 51/52 detection and multi-marker global initialization;
- global initialization remains false while the camera is covered;
- reacquisition initializes after consistent known-marker observations;
- post-initialization one-tag x/y correction while preserving yaw;
- automatic transition between two-tag and one-tag correction;
- approximately 0.37 m combined translation/rotation with one tag while the red live
  cloud remained visually aligned to the grey prior map;
- fitness remained below the configured `0.08` acceptance threshold.

Still required before ERC deployment:

- formal RealSense–Mid-360 rigid extrinsic calibration;
- a never-seen-at-start tag handover test (`ID51 -> none -> ID53`);
- longer loops with terrain, larger yaw changes, tag occlusion, and changing IDs;
- replacement of indoor anchors and zero extrinsics with the final Mars Yard survey.
