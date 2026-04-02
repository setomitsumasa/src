"""
uart_control/serial.py

これは、ROS2のノードで処理したデータをUARTとして、ローバー内のSTM32 MCUに通信するためのノード
USBポーろは必要によって書き換える必要がある
デフォルト設定
    USBポーター: /dev/ttyUSB1 (ローバー内のSTM32 MCUのUARTポーター)
    Baudrate: 115200
    Data: 16bit
    Parity: None
    Stop Bit: 1
    Flow Control: None
"""

#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
import serial
import time

class XbeeSubscriber(Node):

    def __init__(self):
        super().__init__('uart_communication')
        self.subscription = self.create_subscription(
            Int16MultiArray,
            'uart_command',
            self.listener_callback,
            10)
        self.subscription

        #****** Serial setting begin *****
        try:
            self.ser = serial.Serial('/dev/ttyUSB1', 115200, timeout=0.01, write_timeout=0.01)
            self.get_logger().info('Serial setting Succeeded!! Port: /dev/ttyUSB1, Baudrate: 115200')

        except serial.SerialException as e:
            self.get_logger().error(f"Serial setting failed for /dev/ttyUSB1: {e}")

            self.ser = None
        #******* Serial setting end ******

    def listener_callback(self, msg):
        # シリアルポートが正常に開けていない場合は処理しない
        self.get_logger().debug(f'Received Angle: ID={hex(msg.data[0])}, Data={msg.data[1]}') # Rover Angle
        # self.get_logger().debug(f'Received Speed: ID={hex(msg.data[2])}, Data={msg.data[3]}') # Rover Speed
        #****** Serial export begin ******
        try:
            angle_str = f"{hex(msg.data[0])},{msg.data[1]}\r\n"
            speed_str = f"{hex(msg.data[2])},{msg.data[3]}\r\n"

            self.get_logger().info(f"Angle: {angle_str.strip()}, Speed: {speed_str.strip()}")

            # Serial 送信
            self.ser.write(angle_str.encode('utf-8'))
            time.sleep(0.001)
            self.ser.write(speed_str.encode('utf-8'))
            time.sleep(0.001)
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to send Serial command")
        except Exception as e:
            self.get_logger().error(f"An unexpected error occurred in listener_callback: {e}")
        #****** Serial export end *******

def main(args=None):
    rclpy.init(args=args)
    uart_communication = XbeeSubscriber()
    if uart_communication.ser is None:
        uart_communication.get_logger().error("Failed to initialize serial port. Shutting down.")
    else:
        try:
            rclpy.spin(uart_communication)
        except KeyboardInterrupt:
            uart_communication.get_logger().info('KeyboardInterrupt, shutting down.')
        finally:
            # ノードが終了する前にシリアルポートを閉じる
            if uart_communication.ser and uart_communication.ser.is_open:
                uart_communication.get_logger().info('Closing serial port.')
                uart_communication.ser.close()
            # Destroy the node explicitly
            uart_communication.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
