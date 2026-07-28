# ERC prior-map localization handoff

## Package boundary

`ares_erc_bringup` is already a standalone ROS 2 package. It owns the ERC localization
pipeline and exports the interface that a later Nav2 integration should consume:

- TF pose: `map -> camera_init -> body`
- odometry: `/erc/odometry` (`nav_msgs/Odometry`, `map` / `body`)
- readiness: `/erc/localization_initialized` (`std_msgs/Bool`, transient-local)
- quality: `/erc/localization_fitness` (`std_msgs/Float32`, lower is better)
- guarded command input/output (optional): `/erc/nav_cmd_vel` -> `/cmd_vel`

FAST-LIO and `livox_ros_driver2` remain upstream packages and are launched, not forked.
Nav2 configuration, rover footprint, kinematics, and controller tuning are deliberately
outside this package.

## Frame responsibilities

- `map`: prior point-cloud frame; localization landmarks are calibrated here.
- `camera_init`: FAST-LIO local world for the current process.
- `body`: FAST-LIO body pose.
- `datum`: ERC-provided waypoint coordinate frame; it is not a landmark-survey frame.

`map_anchor` is the sole authority for `map -> camera_init`. FAST-LIO is the sole
authority for `camera_init -> body`.

`map -> camera_init` is a session-local frame-alignment transform, not the rover pose.
FAST-LIO may choose a different unobservable yaw gauge for `camera_init` after each
restart. Cross-session position/yaw validation and Nav2 must therefore use the composed
`map -> body` transform (or `/erc/odometry`), never `map -> camera_init` alone.

## Runtime strategy

1. Detect all visible markers in one image.
2. With at least two known, well-separated map landmarks, produce an absolute planar
   `map -> camera_init` candidate.
3. Require three mutually consistent candidates before declaring initialization.
4. After initialization, use one known marker as an x/y-only constraint while retaining
   FAST-LIO/GICP yaw; require three independent same-ID frames to agree.
5. With two or more visible known markers, use the full x/y/yaw landmark constraint.
6. With no visible known markers, continue with FAST-LIO and planar GICP.
7. Reject stale, high-fitness, large-jump, or landmark-spacing-inconsistent updates.

This stage uses only IDs already present in the anchor YAML. Online registration of a
previously unknown landmark is a separate later stage: its first observation cannot
correct the same pose that was used to assign its map coordinate.

The current estimator uses marker centres. The next accuracy upgrade is a joint
image-corner PnP/reprojection estimator. That requires exposing the four pixel corners
for every detection and knowing each physical tag face's map pose; centre-only
`MarkerPose` messages cannot provide that information after detection.

## Required Nav2 inputs not established in this repository

The repository contains apparent provisional values (`max_vel_x: 0.26` and
`robot_radius: 1.0`) but no authoritative rover length, width, ground clearance,
Mid-360 height, steering limit/turning radius, or validated first-test speed. These
must be measured or obtained from the mechanical/control owners before creating a
footprint or controller limits.
