#!/usr/bin/env python3
"""Survey fixed ArUco landmarks directly in the prior-map frame.

Run this only while local GICP is locked to the prior map. Each detected marker
centre is transformed at the image timestamp into ``map`` and accumulated. The final
coordinate is a robust median after a MAD outlier gate.
"""

import math
import os
from collections import defaultdict
from datetime import datetime, timezone

import numpy as np
import rclpy
import yaml
from aruco_opencv_msgs.msg import ArucoDetection
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy,
)
from std_msgs.msg import Bool, Float32
from tf2_ros import (
    Buffer, ConnectivityException, ExtrapolationException, LookupException,
    TransformListener,
)


def quat_to_rotmat(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def robust_position(samples, mad_scale=3.5, absolute_gate=0.03):
    """Return median, kept count, radial MAD and max kept radial residual."""
    points = np.asarray(samples, dtype=float)
    centre = np.median(points, axis=0)
    radial = np.linalg.norm(points - centre, axis=1)
    radial_median = float(np.median(radial))
    mad = float(np.median(np.abs(radial - radial_median)))
    gate = max(float(absolute_gate), float(mad_scale) * 1.4826 * mad)
    kept = points[radial <= gate]
    if len(kept) == 0:
        kept = points
    result = np.median(kept, axis=0)
    kept_radial = np.linalg.norm(kept - result, axis=1)
    max_residual = float(np.max(kept_radial)) if len(kept_radial) else 0.0
    return result, len(kept), mad, max_residual


class ArucoMapCalibrator(Node):
    def __init__(self):
        super().__init__('aruco_map_calibrator')
        self.map_frame = str(self.declare_parameter('map_frame', 'map').value)
        detections_topic = str(
            self.declare_parameter('detections_topic', '/aruco_detections').value)
        self.output_file = os.path.expanduser(str(
            self.declare_parameter(
                'output_file', '/tmp/aruco_anchors_measured.yaml').value))
        self.template_file = os.path.expanduser(str(
            self.declare_parameter('template_file', '').value))
        self.duration = float(self.declare_parameter('duration_sec', 15.0).value)
        self.min_samples = int(self.declare_parameter('min_samples', 30).value)
        self.max_range = float(self.declare_parameter('max_range', 6.0).value)
        self.fitness_max = float(
            self.declare_parameter('fitness_max', 0.02).value)
        self.mad_scale = float(self.declare_parameter('mad_scale', 3.5).value)
        self.absolute_gate = float(
            self.declare_parameter('absolute_outlier_gate', 0.03).value)
        self.face_to_pole_depth = float(
            self.declare_parameter('face_to_pole_depth', 0.0).value)
        self.max_latest_tf_skew = float(
            self.declare_parameter('max_latest_tf_skew_sec', 0.25).value)
        self.overwrite = bool(self.declare_parameter('overwrite', False).value)

        self.initialized = False
        self.fitness = None
        self.samples = defaultdict(list)
        self.start_time = None
        self.finished = False
        self.last_fitness_time = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(
            ArucoDetection, detections_topic, self._detections_cb, 10)
        # map_anchor publishes this state once at startup with transient-local
        # durability. Match that QoS so a calibrator created a few milliseconds later
        # still receives the latched ``true`` sample.
        latched = QoSProfile(depth=1)
        latched.reliability = QoSReliabilityPolicy.RELIABLE
        latched.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            Bool, '/erc/localization_initialized', self._initialized_cb, latched)
        self.create_subscription(
            Float32, '/erc/localization_fitness', self._fitness_cb, 10)
        self.create_timer(0.2, self._tick)
        self.get_logger().info(
            f'ArUco map survey armed: waiting for initialized GICP with fitness <= '
            f'{self.fitness_max:.3f}; then collecting {self.duration:.1f}s into '
            f'{self.output_file}')

    def _initialized_cb(self, msg):
        self.initialized = bool(msg.data)

    def _fitness_cb(self, msg):
        self.fitness = float(msg.data)
        self.last_fitness_time = self.get_clock().now()

    def _quality_ok(self):
        fitness_fresh = (
            self.last_fitness_time is not None
            and (self.get_clock().now() - self.last_fitness_time).nanoseconds * 1e-9
            <= 2.0)
        return (self.initialized and self.fitness is not None and fitness_fresh
                and math.isfinite(self.fitness)
                and self.fitness <= self.fitness_max)

    def _detections_cb(self, msg):
        if self.finished or not self._quality_ok():
            return
        stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, msg.header.frame_id, stamp)
        except (LookupException, ConnectivityException, ExtrapolationException) as ex:
            # Camera frames can arrive a few tens of milliseconds ahead of the 20 Hz
            # dynamic TF broadcaster. During this explicitly stationary survey, using
            # the latest transform is safe within a small bounded skew.
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.map_frame, msg.header.frame_id, rclpy.time.Time())
                detect_sec = (
                    float(msg.header.stamp.sec)
                    + float(msg.header.stamp.nanosec) * 1e-9)
                tf_sec = (
                    float(tf.header.stamp.sec)
                    + float(tf.header.stamp.nanosec) * 1e-9)
                skew = abs(detect_sec - tf_sec)
                if skew > self.max_latest_tf_skew:
                    self.get_logger().warn(
                        f'latest TF skew {skew:.3f}s exceeds '
                        f'{self.max_latest_tf_skew:.3f}s; sample rejected',
                        throttle_duration_sec=2.0)
                    return
                self.get_logger().warn(
                    f'exact-time TF unavailable ({ex}); using latest TF with '
                    f'{skew * 1000.0:.0f} ms skew during stationary calibration',
                    throttle_duration_sec=3.0)
            except (LookupException, ConnectivityException,
                    ExtrapolationException) as latest_ex:
                self.get_logger().warn(
                    f'cannot transform detection, including latest fallback: '
                    f'{latest_ex}', throttle_duration_sec=2.0)
                return

        q_tf = (tf.transform.rotation.x, tf.transform.rotation.y,
                tf.transform.rotation.z, tf.transform.rotation.w)
        rotation = quat_to_rotmat(q_tf)
        translation = np.array([
            tf.transform.translation.x,
            tf.transform.translation.y,
            tf.transform.translation.z,
        ])
        accepted_any = False
        for marker in msg.markers:
            p_face = np.array([
                marker.pose.position.x,
                marker.pose.position.y,
                marker.pose.position.z,
            ])
            if not np.all(np.isfinite(p_face)):
                continue
            marker_range = float(np.linalg.norm(p_face))
            if marker_range > self.max_range:
                continue
            q_marker = (
                marker.pose.orientation.x, marker.pose.orientation.y,
                marker.pose.orientation.z, marker.pose.orientation.w,
            )
            marker_normal = quat_to_rotmat(q_marker)[:, 2]
            p_landmark = p_face - self.face_to_pole_depth * marker_normal
            self.samples[int(marker.marker_id)].append(
                rotation @ p_landmark + translation)
            accepted_any = True
        if accepted_any and self.start_time is None:
            self.start_time = self.get_clock().now()
            self.get_logger().info(
                'GICP lock and TF accepted; keep the rig and tags stationary while '
                f'{self.duration:.1f} seconds of samples are collected.')

    def _tick(self):
        if self.finished:
            return
        if self.start_time is None:
            fitness_text = (
                'none' if self.fitness is None else f'{self.fitness:.4f}')
            self.get_logger().info(
                f'waiting: initialized={self.initialized}, '
                f'fitness={fitness_text} (need <= {self.fitness_max:.4f} and fresh)',
                throttle_duration_sec=3.0)
            return
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        if elapsed >= self.duration:
            self._write_result()
            self.finished = True

    def _write_result(self):
        if os.path.exists(self.output_file) and not self.overwrite:
            self.get_logger().error(
                f'output already exists: {self.output_file}; refusing to overwrite. '
                'Choose output_file:=... or set overwrite:=true.')
            return

        markers = []
        diagnostics = {}
        for marker_id in sorted(self.samples):
            raw = self.samples[marker_id]
            if len(raw) < self.min_samples:
                self.get_logger().warn(
                    f'id{marker_id}: only {len(raw)} samples (< {self.min_samples}); '
                    'not writing this marker.')
                continue
            position, kept, mad, max_residual = robust_position(
                raw, self.mad_scale, self.absolute_gate)
            markers.append({
                'id': int(marker_id),
                'x': round(float(position[0]), 6),
                'y': round(float(position[1]), 6),
                'z': round(float(position[2]), 6),
            })
            diagnostics[str(marker_id)] = {
                'raw_samples': len(raw),
                'kept_samples': kept,
                'radial_mad_m': round(mad, 6),
                'max_kept_residual_m': round(max_residual, 6),
            }

        if not markers:
            self.get_logger().error(
                'calibration produced no marker with enough samples; no output file '
                'was written.')
            return

        document = {}
        if self.template_file:
            try:
                with open(self.template_file, encoding='utf-8') as stream:
                    document = yaml.safe_load(stream) or {}
            except (OSError, yaml.YAMLError) as ex:
                self.get_logger().error(
                    f'cannot read template_file {self.template_file}: {ex}')
                return
        document['coordinate_frame'] = self.map_frame
        document['calibration'] = {
            'created_utc': datetime.now(timezone.utc).isoformat(),
            'method': 'median of timestamped detections during accepted GICP lock',
            'fitness_max': self.fitness_max,
            'face_to_pole_depth': self.face_to_pole_depth,
            'diagnostics': diagnostics,
        }
        document['markers'] = markers
        os.makedirs(os.path.dirname(os.path.abspath(self.output_file)), exist_ok=True)
        with open(self.output_file, 'w', encoding='utf-8') as stream:
            yaml.safe_dump(document, stream, sort_keys=False)

        ids = [m['id'] for m in markers]
        self.get_logger().info(
            f'CALIBRATION COMPLETE: wrote ids={ids} to {self.output_file}. '
            'Review diagnostics before promoting this file to runtime config.')


def main():
    rclpy.init()
    node = ArucoMapCalibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if not node.finished and node.samples:
            node.get_logger().warn(
                'interrupted before duration elapsed; no calibration file written.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
