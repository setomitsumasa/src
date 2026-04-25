import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String
import base64
import cv2
import json
import time

class Encode_image(Node):

    def __init__(self):
        super().__init__('encode_image')
        self.cv_bridge = CvBridge()  # Image メッセージ変換用
        self.max_publish_rate_hz = self.declare_parameter(
            'max_publish_rate_hz', 10.0
        ).value
        self.publish_interval_ns = self._rate_to_interval_ns(self.max_publish_rate_hz)
        self.last_publish_time_ns = 0

        # Subscriber の作成
        self.subscription = self.create_subscription(
            Image,
            'camera/color/image_raw',
            self.encode_image_callback,
            qos_profile_sensor_data)
        
        # Publisher の作成
        self.publisher_ = self.create_publisher(String, 'encoded_image', 10)

    def _rate_to_interval_ns(self, rate_hz):
        if rate_hz is None or rate_hz <= 0.0:
            return 0
        return int(1e9 / rate_hz)

    def _should_publish_now(self):
        if self.publish_interval_ns <= 0:
            return True
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_publish_time_ns < self.publish_interval_ns:
            return False
        self.last_publish_time_ns = now_ns
        return True

    def encode_image_callback(self, msg):
        if not self._should_publish_now():
            return

        self.get_logger().info('Received image, encoding...')
        received_time = self.get_clock().now()
        # ROS Image メッセージを OpenCV 形式に変換
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 画像をエンコードして文字列に変換
        success, buffer = cv2.imencode('.jpg', cv_image)
        if not success:
            self.get_logger().warning('Failed to encode image')
            return
        
        finish_time = self.get_clock().now()
        elapsed_time = (finish_time - received_time).nanoseconds / 1e6  # ミリ秒単位
        self.get_logger().info(f'Image encoded in {elapsed_time:.2f} ms')

        encoded_image_str = base64.b64encode(buffer.tobytes()).decode('ascii')

        ros2_time_capture = msg.header.stamp.nanosec
        ros2_time_now = self.get_clock().now().to_msg().nanosec
        self.get_logger().info(f"ROS2 time now: {ros2_time_now}, capture time: {ros2_time_capture}")
        now_nanosec = time.time() * 1e9
        capture_time = now_nanosec - (ros2_time_now - ros2_time_capture)
        payload = {
            'capture_nanosec': capture_time,
            'image': encoded_image_str,
        }        

        # エンコードした画像を文字列メッセージとしてパブリッシュ
        string_msg = String()
        string_msg.data = json.dumps(payload)
        self.publisher_.publish(string_msg)

def main(args=None):
    rclpy.init(args=args)
    encode_image_node = Encode_image()
    rclpy.spin(encode_image_node)
    encode_image_node.destroy_node()
    rclpy.shutdown()
