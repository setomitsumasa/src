import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

from ultralytics import YOLO
import os
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String, Int16MultiArray
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster
import math
import json
import numpy as np




class Make_YOLOtf(Node):

    def __init__(self):
        super().__init__('make_direction')
        self.cv_bridge = CvBridge()  # Image メッセージ変換用
        self.depth_data = None
        self.depth_scale = None
        self.fx = None
        self.fy = None

        # Subscriber の作成
        self.realsense_info_subscription = self.create_subscription(
            String,
            'realsense_info',
            self.realsense_info_callback,
            qos_profile_sensor_data)

        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            'camera/color/camera_info',
            self.camera_info_callback,
            qos_profile_sensor_data)
        
        self.depth_subscription = self.create_subscription(
            Image,
            'camera/depth/image_raw',
            self.depth_callback,
            qos_profile_sensor_data)

        self.bgr_subscription = self.create_subscription(
            Image,
            'camera/color/image_raw',
            self.bgr_callback,
            qos_profile_sensor_data)

        # Load YOLO model
        self.package_dir = get_package_share_directory('YOLO_detection_v2')
        self.model_path = os.path.join(self.package_dir, 'train260205s_best.pt')
        self.model = YOLO(self.model_path)

        # 検出履歴の初期化
        self.detection_history = [
            {'class': None, 'confidence': None, 'box': [], 'TF_success': False, 'real_world_cordinates': [None, None, None], 'real_world_width': None, 'real_world_height': None},
            {'class': None, 'confidence': None, 'box': [], 'TF_success': False, 'real_world_cordinates': [None, None, None], 'real_world_width': None, 'real_world_height': None},
        ]

        # Publisher の作成
        self.detection_publisher = self.create_publisher(String, 'detection', 10)

        # TF は TransformBroadcaster で /tf に publish（sensor_tf.cpp と同様）
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # 静的 TF: map -> camera_link を /tf_static に publish
        static_tf = TransformStamped()
        static_tf.header.stamp = self.get_clock().now().to_msg()
        static_tf.header.frame_id = 'map'
        static_tf.child_frame_id = 'camera_link'
        static_tf.transform.translation.x = 0.0
        static_tf.transform.translation.y = 0.0
        static_tf.transform.translation.z = 0.0
        static_tf.transform.rotation.x = 0.0
        static_tf.transform.rotation.y = 0.0
        static_tf.transform.rotation.z = 0.0
        static_tf.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(static_tf)

        #物体の大きさ参考値
        self.object_data_list = {
            0: {'class': 'mallet', 'size': [0.04,0.27]}, #mallet
            1: {'class': 'hammer', 'size': [0.16,0.34]}, #hammer
            2: {'class': 'bottle', 'size': [0.04,0.27]}, #bottle
        }



    # RealSenseの情報を取得するcallback関数
    def realsense_info_callback(self, data):
        self.realsense_info = json.loads(data.data)
        #self.get_logger().info(f'realsense_info: {self.realsense_info}')
        self.depth_scale = self.realsense_info['depth_scale']
        self.fx = self.realsense_info['fx']
        self.fy = self.realsense_info['fy']

    def camera_info_callback(self, data):
        self.fx = data.k[0]
        self.fy = data.k[4]



    # 深度データを取得するcallback関数
    def depth_callback(self, raw_depth_data):
        self.depth_data = self.cv_bridge.imgmsg_to_cv2(raw_depth_data, desired_encoding='16UC1')



    # BGRデータを取得しdepthデータとともに検出結果を取得し、TFをpublishするcallback関数
    def bgr_callback(self, raw_bgr_data):
        # BGR 形式の画像データを OpenCV 形式に変換
        self.bgr_data = self.cv_bridge.imgmsg_to_cv2(raw_bgr_data, desired_encoding='bgr8')

        if self.depth_data is None:
            self.get_logger().info('Depth image has not been received yet')
            return

        if self.depth_scale is None or self.fx is None or self.fy is None:
            self.get_logger().info(
                f'Camera/depth parameters have not been received yet '
                f'(depth_scale={self.depth_scale}, fx={self.fx}, fy={self.fy})'
            )
            return

        highest_confidence_detection =self.detection(self.bgr_data)
        # 検出なしの場合
        if highest_confidence_detection['class'] is None:
            TF_success = False
            real_world_x = None
            real_world_y = None
            real_world_z = None
            real_world_width = None
            real_world_height = None
            self.get_logger().info('No YOLOtf')
        # 検出ありの場合
        else:
            # 物体の実世界座標を計算
            TF_success, real_world_x, real_world_y, real_world_z, real_world_width, real_world_height = self.measure_dimensions(self.depth_data, highest_confidence_detection['box'], self.depth_scale, self.fx, self.fy)
            self.get_logger().info(f'TF_success: {TF_success}, real_world_x: {real_world_x}, real_world_y: {real_world_y}, real_world_z: {real_world_z}, real_world_width: {real_world_width}, real_world_height: {real_world_height}')

        # 検出結果を履歴に更新
        self.detection_history[1] = self.detection_history[0]
        self.detection_history[0] = {'class': highest_confidence_detection['class'],
                                     'confidence': highest_confidence_detection['confidence'],
                                     'box': highest_confidence_detection['box'],
                                     'TF_success': TF_success,
                                     'real_world_cordinates': [real_world_x, real_world_y, real_world_z],
                                     'real_world_width': real_world_width,
                                     'real_world_height': real_world_height,
                                    } 


        # TFの安定性を確認
        if self.tf_stabilizer(self.detection_history):
            # 物体の実世界座標をTFに変換してpublish
            tf_msg = TransformStamped()
            tf_msg.header.stamp = self.get_clock().now().to_msg()
            tf_msg.header.frame_id = 'camera_link'
            tf_msg.child_frame_id = self.object_data_list[highest_confidence_detection['class']]['class']
            tf_msg.transform.translation.x = float(real_world_z)
            tf_msg.transform.translation.y = -float(real_world_x)
            tf_msg.transform.translation.z = -float(real_world_y)
            tf_msg.transform.rotation.x = 0.0
            tf_msg.transform.rotation.y = 0.0
            tf_msg.transform.rotation.z = 0.0
            tf_msg.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(tf_msg)
        else:
            self.get_logger().info('not publishing TF')


    # 物体を検出し、最も確信度の高い検出結果を返す
    def detection(self, bgr_data):
        # 推論
        results = self.model(bgr_data, iou=0.6)
        self.get_logger().info(f'cls={results[0].boxes.cls}, conf={results[0].boxes.conf}, box={results[0].boxes.xyxy}')

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
                    'class': int(results[0].boxes.cls[i].item()),
                    'confidence': float(results[0].boxes.conf[i].item()),
                    'box': box_xyxy,
                }
                # 最も確信度の高い検出結果を保持
                if i_detection['confidence'] >= detection['confidence']:
                    detection = i_detection

            self.get_logger().info(f'highest confidence detection: {detection}')
        
        return detection


    # 物体の実寸をメートル単位で計算
    def measure_dimensions(self, depth_frame, detection_data, depth_scale, fx, fy):
        """
        検出結果と深度フレームから物体の実寸をメートル単位で計算します。
        detection_data: [x_min, y_min, x_max, y_max] (ピクセル座標)
        """
        x_min, y_min, x_max, y_max = [int(val) for val in detection_data]
        center_x = (x_min + x_max) / 2
        center_y = (y_min + y_max) / 2

        # バウンディングボックス内の深度データの中央値を計算し、外れ値の影響を減らす
        depth_image = np.asanyarray(depth_frame)
        full_width = depth_image.shape[1]
        full_height = depth_image.shape[0]

        # ROI (Region of Interest) の深度データ
        # 深度画像とカラー画像は整列済みのため、カラーの座標をそのまま使用可能
        roi_depth = depth_image[y_min:y_max, x_min:x_max]
        # ゼロでない有効な深度値のみを選択
        non_zero_depths = roi_depth[roi_depth != 0]
        if len(non_zero_depths) < 100:  # 安定性のために最低ピクセル数をチェック
            return False, None, None, None, None, None  # 深度データが少なすぎる

        # 中央値深度 (メートル単位) - 距離D
        real_world_z = np.median(non_zero_depths) * depth_scale

        center_x_roi = center_x - full_width/2
        center_y_roi = center_y - full_height/2
        # 実世界でのx座標 (X) とy座標 (Y) の計算 (三角測量の原理)
        real_world_x = (center_x_roi * real_world_z) / fx
        real_world_y = (center_y_roi * real_world_z) / fy

        box_width = x_max - x_min
        box_height = y_max - y_min
        real_world_width = (box_width * real_world_z) / fx
        real_world_height = (box_height * real_world_z) / fy

        return True, real_world_x, real_world_y, real_world_z, real_world_width, real_world_height


    # TFの安定性を確認
    def tf_stabilizer(self, detection_history):
        # 物体のTFが成功しているか確認
        if detection_history[0]['TF_success'] and detection_history[1]['TF_success']:
            # 物体のクラスが同じか確認
            if detection_history[0]['class'] == detection_history[1]['class']:
                # 物体の大きさが正常な範囲内か確認
                if (self.object_data_list[detection_history[0]['class']]['size'][0] * 0.5 < detection_history[0]['real_world_width'] 
                    and detection_history[0]['real_world_width'] < self.object_data_list[detection_history[0]['class']]['size'][1] * 1.5
                    and self.object_data_list[detection_history[0]['class']]['size'][0] * 0.5 < detection_history[0]['real_world_height']
                    and detection_history[0]['real_world_height'] < self.object_data_list[detection_history[0]['class']]['size'][1] * 1.5
                    ):
                    # 物体の位置が安定しているか確認
                    if math.dist(detection_history[0]['real_world_cordinates'], detection_history[1]['real_world_cordinates']) < 1.0:
                        return True
                    else:
                        return False
                else:
                    return False
            else:
                return False
        else:
            return False   




def main():
    rclpy.init()
    make_YOLOtf = Make_YOLOtf()
    try:
        rclpy.spin(make_YOLOtf)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
