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
    model_name = 'train260205s_best.pt'
    module_dir = os.path.dirname(os.path.abspath(__file__))
    package_root = os.path.dirname(module_dir)
    candidates = []

    current_dir = module_dir
    while True:
        if os.path.basename(current_dir) == 'install':
            workspace_root = os.path.dirname(current_dir)
            candidates.append(os.path.join(workspace_root, 'src', 'YOLO_detection_v2', model_name))
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


class Make_direction(Node):

    def __init__(self):
        super().__init__('make_direction')
        self.cv_bridge = CvBridge()  # Image メッセージ変換用

        # Subscriber の作成
        self.subscription = self.create_subscription(
            Image,
            'camera/camera/color/image_raw',
            self.infer,
            qos_profile_sensor_data)
        
        # Load YOLO model
        self.model_path = resolve_model_path()
        self.model = YOLO(self.model_path)

        # 検出履歴の初期化
        self.detection_history = [
            {'class': None, 'confidence': None, 'box': []},
            {'class': None, 'confidence': None, 'box': []},
        ]
        self.index = 0

        # Publisher の作成
        self.publisher_ = self.create_publisher(String, 'direction', 10)
        self.cmd_vel_pub_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.uart_command_pub_ = self.create_publisher(Int16MultiArray, 'uart_command', 10)

        # 速度パラメータ（必要に応じて調整）
        self.linear_speed = 0.2   # 直進時の並進速度 [m/s]
        self.angular_speed = 0.5  # 旋回時の角速度 [rad/s]

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
            'box': []
        }
        if len(results[0].boxes) == 0: # 検出なし
            self.get_logger().info('No detections found')
        else:
            for i in range(len(results[0].boxes)):
                box_xywh = results[0].boxes.xywh[i].tolist()
                # 中心座標を0-1に正規化
                normalized_box = [
                    box_xywh[0] / width,      # 中心x座標
                    box_xywh[1] / height,     # 中心y座標
                    box_xywh[2] / width,      # 幅
                    box_xywh[3] / height      # 高さ
                ]                
                i_detection = {
                    'class': int(results[0].boxes.cls[i].item()),
                    'confidence': float(results[0].boxes.conf[i].item()),
                    'box': normalized_box
                }
                # 最も確信度の高い検出結果を保持
                if i_detection['confidence'] >= detection['confidence']:
                    detection = i_detection
        # 履歴を更新
        self.detection_history[self.index] = detection
        self.index = (self.index + 1) % 2

        if (self.detection_history[0]['class'] == self.detection_history[1]['class'] 
            and math.dist(self.detection_history[0]['box'][0:1], self.detection_history[1]['box'][0:1]) < 0.5
            ): # 検出が安定している場合

            if detection['class'] is None: # 検出なし
                direction = 'no_detection'
            else:
                # 検出位置により方向を決定
                if detection['box'][2]*detection['box'][3] > 0.004:  # boxの面積が閾値以上なら停止(1.3-2m)
                    direction = 'stop'
                elif detection['box'][0] > 0.6: # boxの中心x座標が0.6以上なら右折
                    direction = 'right'
                elif detection['box'][0] < 0.4: # boxの中心x座標が0.4以下なら左折
                    direction = 'left'
                else: # boxの中心x座標が0.4-0.6なら直進
                    direction = 'straight' 
        else: # 検出が安定しない場合
            direction = 'unstable' 

        msg = String()
        msg.data = direction
        self.publisher_.publish(msg)

        # /cmd_vel に Twist を publish
        cmd = Twist()
        if direction == 'straight':
            cmd.linear.x = float(self.linear_speed)
            cmd.angular.z = 0.0
        elif direction == 'left':
            cmd.linear.x = float(self.linear_speed * 0.5)
            cmd.angular.z = float(self.angular_speed)
        elif direction == 'right':
            cmd.linear.x = float(self.linear_speed * 0.5)
            cmd.angular.z = float(-self.angular_speed)
        elif direction == 'stop':
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            # 停止時は uart_command で 0x451 を送信（serial_publiasher が UART で MCU に送る）
            uart_msg = Int16MultiArray()
            uart_msg.data = [0x451, 0, 0x451, 0]  # angle_id, angle_val, speed_id, speed_val
            self.uart_command_pub_.publish(uart_msg)
        else:  # no_detection, unstable
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        self.cmd_vel_pub_.publish(cmd)


def main():
    rclpy.init()
    make_direction = Make_direction()
    try:
        rclpy.spin(make_direction)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
