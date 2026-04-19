import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from ultralytics import YOLO
import os


from std_msgs.msg import String, Int16MultiArray
from geometry_msgs.msg import Twist
import math


def resolve_model_path():
    model_name = 'train260402s_best.pt'
    module_dir = os.path.dirname(os.path.abspath(__file__))
    package_root = os.path.dirname(module_dir)
    candidates = []

    current_dir = module_dir
    while True:
        if os.path.basename(current_dir) == 'install':
            workspace_root = os.path.dirname(current_dir)
            candidates.append(os.path.join(workspace_root, 'src', 'YOLO_detection_v4', model_name))
            break
        parent_dir = os.path.dirname(current_dir)
        if parent_dir == current_dir:
            break
        current_dir = parent_dir

    candidates.append(os.path.join(package_root, model_name))

    for candidate in candidates:
        if os.path.exists(candidate):
            return candidate

    raise FileNotFoundError(
        f'YOLO model not found. Tried: {candidates}'
    )


class Make_detection(Node):

    def __init__(self):
        super().__init__('make_detection')
        self.cv_bridge = CvBridge()  # Image メッセージ変換用

        # Subscriber の作成
        self.subscription = self.create_subscription(
            Image,
            'camera/color/image_raw',
            self.infer,
            qos_profile_sensor_data)
        
        # Load YOLO model
        self.model_path = resolve_model_path()
        self.model = YOLO(self.model_path)

        # Publisher の作成
        self.detection_publisher = self.create_publisher(String, 'bounding_box2', 10)

        #object list
        self.object_list = {
            0: 'mallet',
            1: 'hammer',
            2: 'bottle',
        }
    

    def infer(self, data):
        # Image メッセージを OpenCV 形式に変換
        cv_image = self.cv_bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        # 画像サイズを取得
        height, width = cv_image.shape[:2]    

        # 推論
        results = self.model(cv_image, iou=0.6)
        self.get_logger().info(f'cls={results[0].boxes.cls}')
        self.get_logger().info(f'conf={results[0].boxes.conf}')
        self.get_logger().info(f'box={results[0].boxes.xywh}')

        # 推論結果をリスト形式で構築
        detection = {
            'class': None,
            'confidence': 0.0,
            'box': [0.0, 0.0, 0.0, 0.0]
        }
        if len(results[0].boxes) == 0: # 検出なし
            self.get_logger().info('No detections found')
        else:
            for i in range(len(results[0].boxes)):
                box_xyxy = results[0].boxes.xyxy[i].tolist()
                i_detection = {
                    'class': self.object_list.get(int(results[0].boxes.cls[i].item()), 'unknown'),
                    'confidence': float(results[0].boxes.conf[i].item()),
                    'box': box_xyxy,
                }
                # 最も確信度の高い検出結果を保持
                if i_detection['confidence'] >= detection['confidence']:
                    detection = i_detection
        
        self.get_logger().info(f'highest confidence detection: {detection}')
        
        # 検出結果を文字列メッセージとしてパブリッシュ
        string_msg = String()
        string_msg.data = str(detection)
        self.detection_publisher.publish(string_msg)


def main():
    rclpy.init()
    make_detection = Make_detection()
    try:
        rclpy.spin(make_detection)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
