#!/usr/bin/env python3
"""Normalize the shared aruco_opencv output for ERC localization.

The shared detector currently labels its MarkerPose as camera optical-frame data, but
its translation has already been manually remapped to ROS ``camera_link`` axes while
its quaternion remains in OpenCV optical axes.  That makes the Pose internally
inconsistent.  URC consumers depend on the shared package, so ERC fixes the interface
with this adapter instead of modifying/forking the shared I/O layer.

Input:
  /aruco_detections_raw  (legacy/mixed convention from aruco_opencv)
Output:
  /aruco_detections      (position + orientation consistently in header.frame_id)
  TF <header.frame_id> -> aruco_marker_<id> for every valid detection

The corrected marker axes are the solvePnP object axes: +x right on the printed tag,
+y up, +z out of the tag plane toward a front-facing camera.
"""

import math

import rclpy
from aruco_opencv_msgs.msg import ArucoDetection, BoardPose, MarkerPose
from geometry_msgs.msg import Pose, TransformStamped
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def corrected_optical_pose(legacy_pose):
    """Undo aruco_opencv's translation-only optical->camera_link remap.

    utils.cpp currently emits ``(tz, -tx, -ty)`` but leaves the solvePnP quaternion
    untouched.  Recovering ``(tx, ty, tz)`` makes both parts of the Pose optical-frame
    quantities again, matching ArucoDetection.header.frame_id.
    """
    pose = Pose()
    pose.position.x = -legacy_pose.position.y
    pose.position.y = -legacy_pose.position.z
    pose.position.z = legacy_pose.position.x
    pose.orientation = legacy_pose.orientation
    return pose


def pose_is_finite(pose):
    values = (
        pose.position.x, pose.position.y, pose.position.z,
        pose.orientation.x, pose.orientation.y,
        pose.orientation.z, pose.orientation.w,
    )
    if not all(math.isfinite(v) for v in values):
        return False
    qn = math.sqrt(
        pose.orientation.x ** 2 + pose.orientation.y ** 2
        + pose.orientation.z ** 2 + pose.orientation.w ** 2)
    return qn > 1e-6


def correct_detection(raw):
    """Return a convention-corrected message without collapsing marker IDs."""
    corrected = ArucoDetection()
    corrected.header = raw.header
    dropped_ids = []
    for marker in raw.markers:
        pose = corrected_optical_pose(marker.pose)
        if not pose_is_finite(pose):
            dropped_ids.append(int(marker.marker_id))
            continue
        output_marker = MarkerPose()
        output_marker.marker_id = marker.marker_id
        output_marker.pose = pose
        corrected.markers.append(output_marker)

    for board in raw.boards:
        pose = corrected_optical_pose(board.pose)
        if not pose_is_finite(pose):
            continue
        output_board = BoardPose()
        output_board.board_name = board.board_name
        output_board.pose = pose
        corrected.boards.append(output_board)
    return corrected, dropped_ids


class ArucoDetectionAdapter(Node):
    def __init__(self):
        super().__init__('aruco_detection_adapter')
        input_topic = self.declare_parameter(
            'input_topic', '/aruco_detections_raw').value
        output_topic = self.declare_parameter(
            'output_topic', '/aruco_detections').value
        self.publish_marker_tf = bool(
            self.declare_parameter('publish_marker_tf', True).value)
        self.tf_prefix = str(
            self.declare_parameter('tf_prefix', 'aruco_marker_').value)

        self.pub = self.create_publisher(ArucoDetection, output_topic, 10)
        self.sub = self.create_subscription(
            ArucoDetection, input_topic, self._callback, 10)
        self.tf_bc = TransformBroadcaster(self) if self.publish_marker_tf else None
        self.get_logger().info(
            f'ArUco ERC adapter: {input_topic} -> {output_topic}; '
            f'per-ID TF={"on" if self.publish_marker_tf else "off"}')

    def _callback(self, raw):
        corrected, dropped_ids = correct_detection(raw)
        transforms = []
        ids = [int(marker.marker_id) for marker in corrected.markers]
        for marker in corrected.markers:
            if self.tf_bc is not None:
                tf = TransformStamped()
                tf.header = corrected.header
                tf.child_frame_id = f'{self.tf_prefix}{int(marker.marker_id)}'
                tf.transform.translation.x = marker.pose.position.x
                tf.transform.translation.y = marker.pose.position.y
                tf.transform.translation.z = marker.pose.position.z
                tf.transform.rotation = marker.pose.orientation
                transforms.append(tf)

        self.pub.publish(corrected)
        if transforms:
            self.tf_bc.sendTransform(transforms)
        if dropped_ids:
            self.get_logger().warn(
                f'dropping non-finite ArUco pose ids={sorted(dropped_ids)}',
                throttle_duration_sec=2.0)
        if ids:
            self.get_logger().info(
                f'corrected ArUco detections: ids={sorted(ids)}',
                throttle_duration_sec=2.0)


def main():
    rclpy.init()
    node = ArucoDetectionAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
