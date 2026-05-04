#!/usr/bin/env python3
"""
pyrealsense2 を用いて RealSense の RGB と Depth をパブリッシュするノード。
align.process により Depth を Color に合わせて画角を揃えてから配信する。
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from cv_bridge import CvBridge
import numpy as np
import json
import time
try:
    import pyrealsense2 as rs
except ImportError:
    raise ImportError("pyrealsense2 がインストールされていません: pip install pyrealsense2")


class RealSensePublisherNode(Node):
    """RealSense の RGB と Aligned Depth をパブリッシュするノード。"""

    def __init__(self):
        super().__init__("realsense_publisher")

        # パラメータ（オプション）
        self.declare_parameter("color_width", 640)
        self.declare_parameter("color_height", 480)
        self.declare_parameter("depth_width", 640)
        self.declare_parameter("depth_height", 480)
        self.declare_parameter("fps", 15)
        self.declare_parameter("color_topic", "camera/color/image_raw")
        self.declare_parameter("depth_topic", "camera/depth/image_raw")
        self.declare_parameter("camera_info_topic", "camera/color/camera_info")
        self.declare_parameter("camera_link_frame", "camera_link")
        self.declare_parameter("color_optical_frame", "camera_color_optical_frame")
        self.declare_parameter("depth_optical_frame", "camera_depth_optical_frame")

        cw = self.get_parameter("color_width").value
        ch = self.get_parameter("color_height").value
        dw = self.get_parameter("depth_width").value
        dh = self.get_parameter("depth_height").value
        fps = self.get_parameter("fps").value
        color_topic = self.get_parameter("color_topic").value
        depth_topic = self.get_parameter("depth_topic").value
        camera_info_topic = self.get_parameter("camera_info_topic").value
        self.camera_link_frame = self.get_parameter("camera_link_frame").value
        self.color_optical_frame = self.get_parameter("color_optical_frame").value
        self.depth_optical_frame = self.get_parameter("depth_optical_frame").value

        self.cv_bridge = CvBridge()
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # QoS: RELIABLE にすることで RViz2 のデフォルト購読と接続できる
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.pub_color = self.create_publisher(Image, color_topic, qos)
        self.pub_depth = self.create_publisher(Image, depth_topic, qos)
        self.pub_camera_info = self.create_publisher(CameraInfo, camera_info_topic, qos)
        self.pub_realsense_info = self.create_publisher(String, 'realsense_info', qos)

        # RealSense パイプライン
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.camera_info_msg = None


        try:
            # RGBとDepthストリームの有効化
            self.config.enable_stream(rs.stream.depth, dw, dh, rs.format.z16, fps)
            self.config.enable_stream(rs.stream.color, cw, ch, rs.format.bgr8, fps)
            # パイプラインの開始
            self.profile = self.pipeline.start(self.config)
            # 深度スケールを取得 (深度値をメートルに変換するために必要)
            self.depth_sensor = self.profile.get_device().first_depth_sensor()
            self.depth_scale = self.depth_sensor.get_depth_scale()
            # 内部パラメータ (intrinsics) の取得
            self.color_profile = rs.video_stream_profile(self.profile.get_stream(rs.stream.color))
            self.color_intrinsics = self.color_profile.get_intrinsics()
            # fxとfyはピクセルを実世界座標に変換するのに重要
            self.fx = self.color_intrinsics.fx
            self.fy = self.color_intrinsics.fy
            self.camera_info_msg = self.create_camera_info_msg()

            # Depth を Color に合わせるアラインメント
            self.align_to = rs.stream.color
            self.align = rs.align(self.align_to)
            self.publish_camera_frames()
            self.get_logger().info("RealSense パイプラインを開始しました。")
        except Exception as e:
            self.get_logger().error(f"RealSense の起動に失敗しました: {e}")
            raise

        # タイマーでキャプチャ＆パブリッシュ（fps に合わせて周期を設定）
        timer_period = 1.0 / max(1, fps)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def make_optical_transform(self, child_frame_id: str) -> TransformStamped:
        """camera_link から optical frame への static TF を作る。"""
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.camera_link_frame
        transform.child_frame_id = child_frame_id
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = -0.5
        transform.transform.rotation.y = 0.5
        transform.transform.rotation.z = -0.5
        transform.transform.rotation.w = 0.5
        return transform

    def publish_camera_frames(self):
        """camera_link と optical frame 群の static TF を publish する。"""
        transforms = [
            self.make_optical_transform(self.color_optical_frame),
            self.make_optical_transform(self.depth_optical_frame),
        ]
        self.static_tf_broadcaster.sendTransform(transforms)
        self.get_logger().info(
            f"Published static TF: {self.camera_link_frame} -> "
            f"{self.color_optical_frame}, {self.depth_optical_frame}"
        )

    def create_camera_info_msg(self) -> CameraInfo:
        """RealSense intrinsics から CameraInfo を作成する。"""
        camera_info = CameraInfo()
        camera_info.width = self.color_intrinsics.width
        camera_info.height = self.color_intrinsics.height
        camera_info.distortion_model = "plumb_bob"

        coeffs = list(self.color_intrinsics.coeffs)
        if len(coeffs) < 5:
            coeffs.extend([0.0] * (5 - len(coeffs)))
        camera_info.d = coeffs[:5]

        fx = self.color_intrinsics.fx
        fy = self.color_intrinsics.fy
        cx = self.color_intrinsics.ppx
        cy = self.color_intrinsics.ppy

        camera_info.k = [
            fx, 0.0, cx,
            0.0, fy, cy,
            0.0, 0.0, 1.0,
        ]
        camera_info.p = [
            fx, 0.0, cx, 0.0,
            0.0, fy, cy, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        camera_info.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ]
        camera_info.header.frame_id = self.color_optical_frame
        return camera_info

    def timer_callback(self):
        """フレームを取得し、アラインメント後に RGB と Depth をパブリッシュする。"""
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)
        except RuntimeError as e:
            self.get_logger().warn(f"フレーム取得タイムアウト: {e}")
            return

        # Depth を Color の画角に合わせる
        aligned_frames = self.align.process(frames)

        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()

        if not color_frame or not depth_frame:
            return

        # numpy に変換
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        # 現在時刻を ROS2 の時間形式に変換
        stamp = self.get_clock().now().to_msg()

        # BGR8 で配信（RViz2 や OpenCV 系ツールでそのまま正しく表示される）
        msg_color = self.cv_bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
        msg_color.header.stamp = stamp
        msg_color.header.frame_id = self.color_optical_frame
        self.pub_color.publish(msg_color)

        if self.camera_info_msg is not None:
            self.camera_info_msg.header.stamp = stamp
            self.camera_info_msg.header.frame_id = self.color_optical_frame
            self.pub_camera_info.publish(self.camera_info_msg)

        # Depth (16bit 1 channel, 単位は mm)
        msg_depth = self.cv_bridge.cv2_to_imgmsg(depth_image, encoding="16UC1")
        msg_depth.header.stamp = stamp
        msg_depth.header.frame_id = self.depth_optical_frame
        self.pub_depth.publish(msg_depth)


        realsense_info_msg = String()
        realsense_info_msg.data = json.dumps({
            'depth_scale': self.depth_scale,
            'fx': self.fx,
            'fy': self.fy,
        })
        self.pub_realsense_info.publish(realsense_info_msg)

    def destroy_node(self):
        try:
            self.pipeline.stop()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RealSensePublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
