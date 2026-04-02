import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
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

class Make_direction(Node):

    def __init__(self):
        super().__init__('make_direction')
        self.cv_bridge = CvBridge()  # Image メッセージ変換用

        # Subscriber の作成
        self.depth_subscription = self.create_subscription(
            Image,
            'camera/camera/depth/image_rect_raw',
            self.depth_callback,
            qos_profile_sensor_data)
        
  
    
    def depth_callback(self, depth_data):
        Dwidth, Dheight = depth_data.width, depth_data.height 
        # boxは画像中心原点座標系[中心x, 中心y, 幅, 高さ]
        pixel_x = Dwidth/2
        pixel_y = Dheight/2
        object_depth = depth_data.data[int(pixel_y * depth_data.width + pixel_x)]
        self.get_logger().info(f'{object_depth}')
 

def main():
    rclpy.init()
    make_direction = Make_direction()
    try:
        rclpy.spin(make_direction)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
