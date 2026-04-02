#!/usr/bin/env python3
"""
pyrealsense2 を用いて RealSense の RGB と Depth をパブリッシュするノード。
align.process により Depth を Color に合わせて画角を揃えてから配信する。
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import numpy as np
import json
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
        self.declare_parameter("fps", 30)
        self.declare_parameter("color_topic", "camera/color/image_raw")
        self.declare_parameter("depth_topic", "camera/depth/image_raw")

        cw = self.get_parameter("color_width").value
        ch = self.get_parameter("color_height").value
        dw = self.get_parameter("depth_width").value
        dh = self.get_parameter("depth_height").value
        fps = self.get_parameter("fps").value
        color_topic = self.get_parameter("color_topic").value
        depth_topic = self.get_parameter("depth_topic").value

        self.cv_bridge = CvBridge()

        # QoS: RELIABLE にすることで RViz2 のデフォルト購読と接続できる
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.pub_color = self.create_publisher(Image, color_topic, qos)
        self.pub_depth = self.create_publisher(Image, depth_topic, qos)
        self.pub_realsense_info = self.create_publisher(String, 'realsense_info', qos)

        # RealSense パイプライン
        self.pipeline = rs.pipeline()
        self.config = rs.config()


        try:
            # RGBとDepthストリームの有効化
            self.config.enable_stream(rs.stream.depth, dw, dh, rs.format.z16, fps)
            self.config.enable_stream(rs.stream.color, dw, dh, rs.format.bgr8, fps)
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

            # Depth を Color に合わせるアラインメント
            self.align_to = rs.stream.color
            self.align = rs.align(self.align_to)
            self.get_logger().info("RealSense パイプラインを開始しました。")
        except Exception as e:
            self.get_logger().error(f"RealSense の起動に失敗しました: {e}")
            raise

        # タイマーでキャプチャ＆パブリッシュ（fps に合わせて周期を設定）
        timer_period = 1.0 / max(1, fps)
        self.timer = self.create_timer(timer_period, self.timer_callback)

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

        stamp = self.get_clock().now().to_msg()

        # BGR8 で配信（RViz2 や OpenCV 系ツールでそのまま正しく表示される）
        msg_color = self.cv_bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
        msg_color.header.stamp = stamp
        msg_color.header.frame_id = "camera_color_optical_frame"
        self.pub_color.publish(msg_color)

        # Depth (16bit 1 channel, 単位は mm)
        msg_depth = self.cv_bridge.cv2_to_imgmsg(depth_image, encoding="16UC1")
        msg_depth.header.stamp = stamp
        msg_depth.header.frame_id = "camera_color_optical_frame"
        self.pub_depth.publish(msg_depth)


        realsense_info_msg = String()
        realsense_info_msg.data = json.dumps({'depth_scale': self.depth_scale, 'fx': self.fx, 'fy': self.fy})
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
