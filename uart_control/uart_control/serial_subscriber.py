"""
uart_control/serial_subscriber.py

これは、UART(USB0)から取得したデータをROS2トピックとしてPublishするノードです。

想定する受信フォーマットの例:
    408,-65.88
    415,0.00000000000

各行を「ID,データ」の2列に分割し、ROS2では
    Float64MultiArray.data = [ID, DATA]
という2要素の配列としてpublishします。

デフォルト設定:
    USBポート : /dev/ttyUSB0
    Baudrate : 115200
"""

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import serial
from serial import SerialException
import termios


class SerialSubscriber(Node):
    """USB0 からのシリアルデータを読み取り、IDと値を2列配列でpublishするノード"""

    def __init__(self) -> None:
        super().__init__('uart_serial_subscriber')

        self.port = '/dev/uart_sensor'
        self.baudrate = 115200
        self.reconnect_period_sec = 1.0
        self._last_reconnect_log_ok = False
        self._last_disconnect_log_sent = False

        # Publish 先トピック: uart_data
        self.publisher_ = self.create_publisher(
            Float64MultiArray,
            'uart_data',
            10
        )

        # ****** Serial setting begin *****
        self.ser = None

        # 受信バッファ（行末で切れた残りを次回に持ち越す）
        self._line_buffer = ''

        self.connect_serial()
        # 高頻度ポーリングでバッファを溜めない（1ms = 1000Hz）
        self.timer = self.create_timer(0.001, self.read_and_publish)
        # 定期的に接続状態を確認し、切断時は再接続する
        self.health_check_timer = self.create_timer(
            self.reconnect_period_sec,
            self.check_and_reconnect
        )
        # ****** Serial setting end *****

    def connect_serial(self) -> bool:
        """シリアルポートへ接続する。成功時は True を返す。"""
        self.close_serial(clear_buffer=False)

        try:
            self.ser = serial.Serial(
                self.port,
                self.baudrate,
                timeout=0.0
            )
            self._line_buffer = ''
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

    def close_serial(self, clear_buffer: bool = True) -> None:
        """シリアルポートを安全に閉じる。"""
        if self.ser is not None:
            try:
                if self.ser.is_open:
                    self.ser.close()
            except SerialException as e:
                self.get_logger().debug(f'Failed to close serial port cleanly: {e}')
        self.ser = None
        if clear_buffer:
            self._line_buffer = ''

    def is_serial_healthy(self) -> bool:
        """シリアルポートが実際に生きているかを軽く確認する。"""
        if self.ser is None or not self.ser.is_open:
            return False

        try:
            _ = self.ser.in_waiting
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

    def read_and_publish(self) -> None:
        """シリアルバッファを一括読みし、届いている全行を処理してpublishする"""
        if not self.is_serial_healthy():
            return

        try:
            # 受信バッファに溜まっているバイトをまとめて読む（取りこぼし防止）
            n = self.ser.in_waiting
            if n > 0:
                self._line_buffer += self.ser.read(n).decode('utf-8', errors='replace')

            # 行区切りで分割（最後の要素は改行未到着の可能性あり）
            lines = self._line_buffer.splitlines()
            if not lines:
                return
            # 最後の1つは改行が来てないかもしれないのでバッファに戻す
            self._line_buffer = lines.pop() if self._line_buffer and self._line_buffer[-1] != '\n' else ''

            for line in lines:
                line = line.strip()
                if not line:
                    continue

                # 例: "408,-65.88"
                parts = line.split(',')
                if len(parts) != 2:
                    self.get_logger().warn(f"Invalid format: {line}")
                    continue

                id_str, data_str = parts[0].strip(), parts[1].strip()

                try:
                    id_val = float(int(id_str, 10))
                    data_val = float(data_str)
                except ValueError as e:
                    self.get_logger().warn(f"Failed to parse line '{line}': {e}")
                    continue

                msg = Float64MultiArray()
                msg.data = [id_val, data_val]
                self.publisher_.publish(msg)
                self.get_logger().debug(f"Published: ID={id_val}, DATA={data_val}")

        except (SerialException, OSError, termios.error) as e:
            self.get_logger().error(f"Serial read error: {e}")
            self.close_serial()
        except Exception as e:
            self.get_logger().error(
                f"Unexpected error in read_and_publish: {e}"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SerialSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt, shutting down.')
    finally:
        if node.ser and node.ser.is_open:
            node.get_logger().info('Closing serial port.')
        node.close_serial()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
