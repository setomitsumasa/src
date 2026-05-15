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
import os
import time
try:
    import pyrealsense2 as rs
except ImportError:
    raise ImportError("pyrealsense2 がインストールされていません: pip install pyrealsense2")


DS5_PRODUCT_IDS = {
    "0AD1", "0AD2", "0AD3", "0AD4", "0AD5", "0AF6", "0AFE", "0AFF",
    "0B00", "0B01", "0B03", "0B07", "0B3A", "0B5C", "0B5B",
}

CUSTOM_JSON_PREFIXES = (
    "controls-autoexposure-roi-",
    "controls-color-autoexposure-roi-",
)


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
        self.declare_parameter("json_file_path", "")
        self.declare_parameter("enable_advanced_mode", True)

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
        json_file_path = self.get_parameter("json_file_path").value
        enable_advanced_mode = self.get_parameter("enable_advanced_mode").value

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
            stream_settings = self.load_json_settings(
                json_file_path,
                enable_advanced_mode=enable_advanced_mode,
            )
            dw = self.get_json_int(stream_settings, "stream-width", dw)
            dh = self.get_json_int(stream_settings, "stream-height", dh)
            cw = self.get_json_int(stream_settings, "stream-width", cw)
            ch = self.get_json_int(stream_settings, "stream-height", ch)
            fps = self.get_json_int(stream_settings, "stream-fps", fps)

            # RGBとDepthストリームの有効化
            self.config.enable_stream(rs.stream.depth, dw, dh, rs.format.z16, fps)
            self.config.enable_stream(rs.stream.color, cw, ch, rs.format.bgr8, fps)
            # パイプラインの開始
            self.profile = self.pipeline.start(self.config)
            self.apply_auto_exposure_roi(
                stream_settings,
                self.profile.get_device(),
                depth_size=(dw, dh),
                color_size=(cw, ch),
            )
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

    def normalize_optional_path(self, value) -> str:
        """launch から渡る空文字表現を通常の空文字にそろえる。"""
        if value is None:
            return ""
        path = str(value).strip().strip("'").strip('"')
        return os.path.expanduser(path)

    def get_json_int(self, settings: dict, key: str, default: int) -> int:
        if not settings or key not in settings:
            return default
        try:
            return int(settings[key])
        except (TypeError, ValueError):
            self.get_logger().warn(
                f"JSON の {key}={settings[key]!r} を整数として読めないため、{default} を使います。"
            )
            return default

    def get_json_bool(self, settings: dict, key: str, default: bool) -> bool:
        if not settings or key not in settings:
            return default
        value = settings[key]
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        return bool(value)

    def find_advanced_mode_device(self):
        ctx = rs.context()
        for dev in ctx.query_devices():
            if not dev.supports(rs.camera_info.product_id):
                continue
            product_id = str(dev.get_info(rs.camera_info.product_id)).upper()
            if product_id not in DS5_PRODUCT_IDS:
                continue
            if dev.supports(rs.camera_info.name):
                self.get_logger().info(
                    f"Advanced Mode 対応デバイスを検出: {dev.get_info(rs.camera_info.name)}"
                )
            return dev
        raise RuntimeError("Advanced Mode 対応の RealSense D400 系デバイスが見つかりません。")

    def load_json_settings(self, json_file_path: str, enable_advanced_mode: bool) -> dict:
        json_file_path = self.normalize_optional_path(json_file_path)
        if not json_file_path:
            return {}
        if not os.path.isfile(json_file_path):
            raise FileNotFoundError(f"RealSense JSON 設定ファイルが見つかりません: {json_file_path}")

        with open(json_file_path, "r", encoding="utf-8") as file:
            json_text = file.read().strip()
        settings = json.loads(json_text)
        advanced_settings = {
            key: value
            for key, value in settings.items()
            if not key.startswith(CUSTOM_JSON_PREFIXES)
        }

        dev = self.find_advanced_mode_device()
        advnc_mode = rs.rs400_advanced_mode(dev)
        retry_count = 0
        while enable_advanced_mode and not advnc_mode.is_enabled() and retry_count < 3:
            self.get_logger().info("RealSense Advanced Mode を有効化します。デバイスが再接続されます。")
            advnc_mode.toggle_advanced_mode(True)
            time.sleep(5)
            dev = self.find_advanced_mode_device()
            advnc_mode = rs.rs400_advanced_mode(dev)
            retry_count += 1

        if not advnc_mode.is_enabled():
            raise RuntimeError("RealSense Advanced Mode が無効なため JSON 設定を読み込めません。")

        advnc_mode.load_json(json.dumps(advanced_settings))
        self.get_logger().info(f"RealSense JSON 設定を読み込みました: {json_file_path}")
        return settings

    def make_roi(self, settings: dict, prefix: str, size: tuple):
        if not self.get_json_bool(settings, prefix + "enabled", False):
            return None

        width, height = size
        min_x = self.get_json_int(settings, prefix + "min-x", width // 8)
        min_y = self.get_json_int(settings, prefix + "min-y", height // 8)
        max_x = self.get_json_int(settings, prefix + "max-x", width - (width // 8) - 1)
        max_y = self.get_json_int(settings, prefix + "max-y", height - (height // 8) - 1)

        min_x = max(0, min(min_x, width - 1))
        min_y = max(0, min(min_y, height - 1))
        max_x = max(0, min(max_x, width - 1))
        max_y = max(0, min(max_y, height - 1))
        if min_x >= max_x or min_y >= max_y:
            raise ValueError(
                f"不正な ROI 設定です: {prefix} min=({min_x}, {min_y}) max=({max_x}, {max_y})"
            )

        roi = rs.region_of_interest()
        roi.min_x = min_x
        roi.min_y = min_y
        roi.max_x = max_x
        roi.max_y = max_y
        return roi

    def apply_auto_exposure_roi(self, settings: dict, device, depth_size: tuple, color_size: tuple):
        if not settings:
            return

        depth_roi = self.make_roi(settings, "controls-autoexposure-roi-", depth_size)
        color_roi = self.make_roi(settings, "controls-color-autoexposure-roi-", color_size)
        if depth_roi is None and color_roi is None:
            return

        for sensor in device.query_sensors():
            if not sensor.is_roi_sensor():
                continue
            sensor_name = sensor.get_info(rs.camera_info.name) if sensor.supports(rs.camera_info.name) else ""
            if "RGB" in sensor_name.upper() or "COLOR" in sensor_name.upper():
                roi = color_roi
                label = "Color"
            else:
                roi = depth_roi
                label = "Depth"

            if roi is None:
                continue
            sensor.as_roi_sensor().set_region_of_interest(roi)
            self.get_logger().info(
                f"{label} auto exposure ROI を設定しました: "
                f"min=({roi.min_x}, {roi.min_y}) max=({roi.max_x}, {roi.max_y})"
            )

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
