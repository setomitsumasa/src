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
from serial import SerialException
import termios
import time

class XbeeSubscriber(Node):

    def __init__(self):
        super().__init__('uart_communication')
        self.port = '/dev/ttyUSB1'
        self.baudrate = 115200
        self.reconnect_period_sec = 1.0
        self._last_reconnect_log_ok = False
        self._last_disconnect_log_sent = False

        self.subscription = self.create_subscription(
            Int16MultiArray,
            'uart_command',
            self.listener_callback,
            10)
        self.subscription

        #****** Serial setting begin *****
        self.ser = None
        self.connect_serial()
        self.health_check_timer = self.create_timer(
            self.reconnect_period_sec,
            self.check_and_reconnect
        )
        #******* Serial setting end ******

    def connect_serial(self) -> bool:
        """シリアルポートへ接続する。成功時は True を返す。"""
        self.close_serial()

        try:
            self.ser = serial.Serial(
                self.port,
                self.baudrate,
                timeout=0.01,
                write_timeout=0.01
            )
            self._last_reconnect_log_ok = True
            self._last_disconnect_log_sent = False
            self.get_logger().info(
                f'Serial connected. Port: {self.port}, Baudrate: {self.baudrate}'
            )
            return True
        except SerialException as e:
            self.ser = None
            if self._last_reconnect_log_ok:
                self.get_logger().warn(
                    f'Serial connection failed for {self.port}: {e}'
                )
                self._last_reconnect_log_ok = False
            else:
                self.get_logger().debug(
                    f'Serial reconnection retry failed for {self.port}: {e}'
                )
            return False

    def close_serial(self) -> None:
        """シリアルポートを安全に閉じる。"""
        if self.ser is not None:
            try:
                if self.ser.is_open:
                    self.ser.close()
            except SerialException as e:
                self.get_logger().debug(f'Failed to close serial port cleanly: {e}')
        self.ser = None

    def is_serial_healthy(self) -> bool:
        """シリアルポートが実際に生きているかを軽く確認する。"""
        if self.ser is None or not self.ser.is_open:
            return False

        try:
            _ = self.ser.out_waiting
            return True
        except (SerialException, OSError, termios.error) as e:
            if not self._last_disconnect_log_sent:
                self.get_logger().warn(
                    f'Serial port {self.port} became unavailable: {e}'
                )
                self._last_disconnect_log_sent = True
            self.close_serial()
            return False

    def check_and_reconnect(self) -> None:
        """定期的に接続状態を確認し、切断時は再接続を試みる。"""
        if self.is_serial_healthy():
            return

        self.get_logger().warn(
            f'Serial port {self.port} is disconnected. Attempting reconnection.'
        )
        self.connect_serial()

    def listener_callback(self, msg):
        # シリアルポートが正常に開けていない場合は処理しない
        if not self.is_serial_healthy():
            self.get_logger().warn('Serial port is not available. Skipping UART send.')
            return
        if len(msg.data) < 4:
            self.get_logger().warn(
                f'Invalid uart_command length: expected 4 values, got {len(msg.data)}'
            )
            return

        self.get_logger().debug(f'Received Angle: ID={hex(msg.data[0])}, Data={msg.data[1]}') # Rover Angle
        # self.get_logger().debug(f'Received Speed: ID={hex(msg.data[2])}, Data={msg.data[3]}') # Rover Speed
        #****** Serial export begin ******
        try:
            angle_str = f"{hex(msg.data[0])},{msg.data[1]}\r\n"
            speed_str = f"{hex(msg.data[2])},{msg.data[3]}\r\n"

            self.get_logger().info(f"Angle: {angle_str.strip()}, Speed: {speed_str.strip()}")

            # Serial 送信
            angle_bytes = angle_str.encode('utf-8')
            speed_bytes = speed_str.encode('utf-8')
            angle_written = self.ser.write(angle_bytes)
            time.sleep(0.001)
            speed_written = self.ser.write(speed_bytes)
            time.sleep(0.001)
            if angle_written != len(angle_bytes) or speed_written != len(speed_bytes):
                self.get_logger().warn(
                    f"Serial write length mismatch: Angle {angle_written}/{len(angle_bytes)}, "
                    f"Speed {speed_written}/{len(speed_bytes)}"
                )
            if msg.data[0] == 0x481 or msg.data[2] == 0x481:
                self.get_logger().info(
                    f"0x481をUARTへ送信しました: Angle: {angle_str.strip()}, "
                    f"Speed: {speed_str.strip()}"
                )
        except (SerialException, OSError, termios.error) as e:
            self.get_logger().error(f"Failed to send Serial command: {e}")
            self.close_serial()
        except Exception as e:
            self.get_logger().error(f"An unexpected error occurred in listener_callback: {e}")
        #****** Serial export end *******

def main(args=None):
    rclpy.init(args=args)
    uart_communication = XbeeSubscriber()
    try:
        rclpy.spin(uart_communication)
    except KeyboardInterrupt:
        uart_communication.get_logger().info('KeyboardInterrupt, shutting down.')
    finally:
        if uart_communication.ser and uart_communication.ser.is_open:
            uart_communication.get_logger().info('Closing serial port.')
        uart_communication.close_serial()
        uart_communication.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
