#!/usr/bin/env python3
"""ERC Phase 2 (Milestone 2a): global odometry output from the live localization.

Phase 1 gave us two TF links that are updated live while the GICP relocalization
runs (`map_anchor`) and FAST-LIO tracks the sensor:

    map --(map_anchor, GICP global correction)--> camera_init(odom)
    camera_init --(FAST-LIO local odometry)--> body

Their composition  map -> body  is the robot's *drift-corrected global pose* in the
prior-map / datum frame. This node looks that composed transform up from TF and
republishes it as:
  * `nav_msgs/Odometry` on `/erc/odometry` (header.frame_id=map, child_frame_id=body)
    -- the pose (and a finite-difference twist) that Nav2 will later consume,
  * `nav_msgs/Path` on `/erc/trajectory` -- the accumulated trajectory for RViz.

It is a *pure consumer* of TF: it publishes no transform, so the single-authority TF
rule (CLAUDE.md §6.2) is preserved (map_anchor owns map->camera_init, FAST-LIO owns
camera_init->body). Run it on top of `localize.launch.py`.
"""
import math
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, \
    ExtrapolationException


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_inv(q):
    x, y, z, w = q          # unit quaternion -> inverse == conjugate
    return (-x, -y, -z, w)


def quat_to_rotvec(q):
    """Rotation-vector (axis*angle) of a unit quaternion, in the same frame as q."""
    x, y, z, w = q
    w = max(-1.0, min(1.0, w))
    angle = 2.0 * math.acos(w)
    s = math.sqrt(max(0.0, 1.0 - w * w))
    if s < 1e-9:
        return (0.0, 0.0, 0.0)      # ~no rotation
    return (angle * x / s, angle * y / s, angle * z / s)


def quat_to_rotmat(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class ErcOdometry(Node):
    def __init__(self):
        super().__init__('erc_odometry')
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.body_frame = self.declare_parameter('body_frame', 'body').value
        rate = float(self.declare_parameter('publish_rate', 30.0).value)
        self.path_max = int(self.declare_parameter('path_max_poses', 3000).value)
        # only append a Path vertex once the robot has moved this far (keeps Path light)
        self.path_min_step = float(self.declare_parameter('path_min_step', 0.05).value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.odom_pub = self.create_publisher(Odometry, '/erc/odometry', 10)
        self.path_pub = self.create_publisher(Path, '/erc/trajectory', 10)

        self.path = Path()
        self.path.header.frame_id = self.map_frame
        self.path_vertices = deque(maxlen=self.path_max)

        self.prev = None       # (t_sec, pos np(3), quat tuple)
        self.timer = self.create_timer(1.0 / max(1.0, rate), self._tick)
        self.get_logger().info(
            f'erc_odometry up: publishing {self.map_frame}->{self.body_frame} as '
            f'/erc/odometry (+/erc/trajectory) at {rate:.0f} Hz')

    def _tick(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.body_frame, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().warn(
                f'waiting for TF {self.map_frame}->{self.body_frame} '
                '(is localize.launch.py running?)', throttle_duration_sec=3.0)
            return

        tr = tf.transform.translation
        rot = tf.transform.rotation
        pos = np.array([tr.x, tr.y, tr.z])
        quat = (rot.x, rot.y, rot.z, rot.w)
        stamp = tf.header.stamp
        t_sec = stamp.sec + stamp.nanosec * 1e-9

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.map_frame
        odom.child_frame_id = self.body_frame
        odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = pos
        odom.pose.pose.orientation = rot

        # Finite-difference twist, expressed in the body (child) frame per REP-105.
        if self.prev is not None:
            dt = t_sec - self.prev[0]
            if dt > 1e-3:
                v_map = (pos - self.prev[1]) / dt
                r_bm = quat_to_rotmat(quat).T          # map -> body rotation
                v_body = r_bm @ v_map
                dq = quat_mul(quat_inv(self.prev[2]), quat)   # body-relative rotation
                wx, wy, wz = (c / dt for c in quat_to_rotvec(dq))
                odom.twist.twist.linear.x = float(v_body[0])
                odom.twist.twist.linear.y = float(v_body[1])
                odom.twist.twist.linear.z = float(v_body[2])
                odom.twist.twist.angular.x = wx
                odom.twist.twist.angular.y = wy
                odom.twist.twist.angular.z = wz
        self.prev = (t_sec, pos, quat)
        self.odom_pub.publish(odom)

        # Append to the trajectory only when we have moved enough.
        if not self.path_vertices or \
                np.linalg.norm(pos - self.path_vertices[-1]) >= self.path_min_step:
            ps = PoseStamped()
            ps.header.stamp = stamp
            ps.header.frame_id = self.map_frame
            ps.pose = odom.pose.pose
            self.path_vertices.append(pos)
            self.path.poses.append(ps)
            while len(self.path.poses) > self.path_max:
                self.path.poses.pop(0)
        self.path.header.stamp = stamp
        self.path_pub.publish(self.path)


def main():
    rclpy.init()
    node = ErcOdometry()
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
