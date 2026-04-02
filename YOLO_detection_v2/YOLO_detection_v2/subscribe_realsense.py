import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class ImgReceiver(Node):

    def __init__(self):
        super().__init__('img_receiver')
        self.subscription = self.create_subscription(
            Image,
            'camera/camera/color/image_raw',
            self.image_callback,
            qos_profile_sensor_data)

    def image_callback(self, data):
        self.get_logger().info('got image!')
        pass


def main():
    rclpy.init()
    img_receiver = ImgReceiver()
    try:
        rclpy.spin(img_receiver)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
