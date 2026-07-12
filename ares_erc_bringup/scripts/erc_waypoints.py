#!/usr/bin/env python3
"""ERC Phase 1 (Milestone 1a): datum <-> map alignment + waypoint placement.

Reads waypoints_erc.yaml (datum-frame, start-relative Cartesian coords) and:
  1) broadcasts a static TF  map_frame(map) -> datum  (pure yaw),
  2) publishes the waypoints as an RViz MarkerArray (spheres + text, frame=datum),
  3) publishes them as a PoseArray (frame=datum) for the Phase-2 Nav2 follower.

Why a pure yaw: the prior-map `map` origin IS the start point (FAST-LIO's camera_init
at mapping time, gravity-aligned z). So the ERC datum (origin = start) coincides in
position; only the heading differs -> a single yaw (datum.yaw_offset_deg). 0 = datum
axes == initial heading.

Convention: the static TF parent=map_frame, child=datum, rotation = Rz(yaw_offset). A
point p in the datum frame therefore appears at Rz(yaw)*p in the map frame.

Frames (Phase 1b): map_frame defaults to `map` (the global/prior-map frame). In live
localization, map_anchor bridges map -> camera_init(odom) and FAST-LIO owns
camera_init -> body, so datum stays fixed in the prior map. For the offline demo (no
FAST-LIO), the prior cloud is published directly in `map`.
"""
import math
import os

import rclpy
import yaml
from geometry_msgs.msg import Pose, PoseArray, TransformStamped
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from tf2_ros import StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class ErcWaypoints(Node):
    def __init__(self):
        super().__init__('erc_waypoints')
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('datum_frame', 'datum')

        wp_file = self.get_parameter('waypoints_file').value
        self.map_frame = self.get_parameter('map_frame').value
        self.datum_frame = self.get_parameter('datum_frame').value

        if not wp_file or not os.path.isfile(wp_file):
            raise RuntimeError(f'waypoints_file not found: {wp_file!r}')
        with open(wp_file) as f:
            cfg = yaml.safe_load(f)
        self.yaw_offset = math.radians(float(cfg.get('datum', {}).get('yaw_offset_deg', 0.0)))
        self.waypoints = cfg.get('waypoints', []) or []
        if not self.waypoints:
            raise RuntimeError('no waypoints in ' + wp_file)

        # Latched QoS so RViz opened after start still receives the last message.
        latched = QoSProfile(depth=1)
        latched.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.marker_pub = self.create_publisher(MarkerArray, '/erc/waypoint_markers', latched)
        self.pose_pub = self.create_publisher(PoseArray, '/erc/waypoint_poses', latched)

        self.static_bc = StaticTransformBroadcaster(self)
        self._publish_static_tf()

        self._publish_waypoints()
        self.create_timer(1.0, self._publish_waypoints)  # cheap refresh
        self.get_logger().info(
            f'ERC waypoints: {len(self.waypoints)} pts, datum yaw='
            f'{math.degrees(self.yaw_offset):.1f} deg, TF {self.map_frame} -> {self.datum_frame}')

    def _publish_static_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame     # map (parent)
        t.child_frame_id = self.datum_frame    # datum (child)
        qx, qy, qz, qw = yaw_to_quat(self.yaw_offset)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.static_bc.sendTransform(t)

    def _publish_waypoints(self):
        stamp = self.get_clock().now().to_msg()
        arr = MarkerArray()
        poses = PoseArray()
        poses.header.frame_id = self.datum_frame
        poses.header.stamp = stamp
        for i, wp in enumerate(self.waypoints):
            name = str(wp.get('name', f'WP{i + 1}'))
            x, y, z = float(wp.get('x', 0.0)), float(wp.get('y', 0.0)), float(wp.get('z', 0.0))
            is_finish = name.upper().startswith('FIN')

            m = Marker()
            m.header.frame_id = self.datum_frame
            m.header.stamp = stamp
            m.ns = 'erc_wp'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.4
            if is_finish:
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.2, 0.2, 0.9
            else:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 1.0, 0.3, 0.9
            arr.markers.append(m)

            tm = Marker()
            tm.header.frame_id = self.datum_frame
            tm.header.stamp = stamp
            tm.ns = 'erc_wp_label'
            tm.id = i
            tm.type = Marker.TEXT_VIEW_FACING
            tm.action = Marker.ADD
            tm.pose.position.x, tm.pose.position.y, tm.pose.position.z = x, y, z + 0.5
            tm.pose.orientation.w = 1.0
            tm.scale.z = 0.35
            tm.color.r = tm.color.g = tm.color.b = tm.color.a = 1.0
            tm.text = name
            arr.markers.append(tm)

            p = Pose()
            p.position.x, p.position.y, p.position.z = x, y, z
            p.orientation.w = 1.0  # Phase 2 may orient toward the next waypoint
            poses.poses.append(p)

        self.marker_pub.publish(arr)
        self.pose_pub.publish(poses)


def main():
    rclpy.init()
    node = ErcWaypoints()
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
