#!/usr/bin/env python3
"""ERC Phase 3 offline tester — synthesize ArUco detections for aruco_map_anchor.

No camera here, so we fake it. Each known datum point is transformed through a selected
ground-truth map->datum and then into ``parent_frame`` using a selected ground-truth
map->parent pose. An identity parent->camera TF makes the result look like detections
from a camera attached to that parent.

This supports two tests:
* parent_frame=map, true parent pose=identity: map->datum convergence;
* parent_frame=camera_init, non-identity true parent pose: arbitrary-start
  map->camera_init initialization.
"""
import math
import os

import numpy as np
import rclpy
import yaml
from aruco_opencv_msgs.msg import ArucoDetection, MarkerPose
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster


class FakeDetections(Node):
    def __init__(self):
        super().__init__('aruco_fake_detections')
        anchors_file = self.declare_parameter('anchors_file', '').value
        self.frame = self.declare_parameter('camera_frame', 'camera_color_optical_frame').value
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.parent_frame = self.declare_parameter('parent_frame', self.map_frame).value
        true_yaw = math.radians(float(self.declare_parameter('true_yaw_deg', 30.0).value))
        true_xy = self.declare_parameter('true_xy', [0.2, -0.1]).value
        parent_yaw = math.radians(
            float(self.declare_parameter('true_parent_yaw_deg', 0.0).value))
        parent_xy = self.declare_parameter('true_parent_xy', [0.0, 0.0]).value
        rate = float(self.declare_parameter('rate', 10.0).value)

        with open(anchors_file) as f:
            cfg = yaml.safe_load(f) or {}
        c, s = math.cos(true_yaw), math.sin(true_yaw)
        R_map_datum = np.array([[c, -s], [s, c]])
        t_map_datum = np.array([float(true_xy[0]), float(true_xy[1])])
        cp, sp = math.cos(parent_yaw), math.sin(parent_yaw)
        R_map_parent = np.array([[cp, -sp], [sp, cp]])
        t_map_parent = np.array([float(parent_xy[0]), float(parent_xy[1])])

        # p_parent = inv(T_map_parent) * T_map_datum * p_datum.
        self.markers = []
        for m in (cfg.get('markers', []) or []):
            p_d = np.array([float(m.get('x', 0.0)), float(m.get('y', 0.0))])
            p_map = R_map_datum @ p_d + t_map_datum
            p_parent = R_map_parent.T @ (p_map - t_map_parent)
            self.markers.append(
                (int(m['id']), p_parent[0], p_parent[1], float(m.get('z', 0.0))))

        # Pin the synthetic camera to its parent. map_anchor supplies map->camera_init
        # during the arbitrary-start test, so this publisher never duplicates that edge.
        self.static_bc = StaticTransformBroadcaster(self)
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.parent_frame
        tf.child_frame_id = self.frame
        tf.transform.rotation.w = 1.0
        self.static_bc.sendTransform(tf)

        self.pub = self.create_publisher(ArucoDetection, '/aruco_detections', 10)
        self.create_timer(1.0 / max(1.0, rate), self.tick)
        self.get_logger().info(
            f'fake detections: {len(self.markers)} markers, true yaw={math.degrees(true_yaw):.1f} '
            f'deg, true xy=({t_map_datum[0]:.2f},{t_map_datum[1]:.2f}); '
            f'true map->{self.parent_frame}=({t_map_parent[0]:.2f},'
            f'{t_map_parent[1]:.2f},{math.degrees(parent_yaw):.1f} deg)')

    def tick(self):
        msg = ArucoDetection()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame
        for mid, x, y, z in self.markers:
            mp = MarkerPose()
            mp.marker_id = mid
            mp.pose.position.x = x
            mp.pose.position.y = y
            mp.pose.position.z = z
            mp.pose.orientation.w = 1.0
            msg.markers.append(mp)
        self.pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(FakeDetections())


if __name__ == '__main__':
    main()
