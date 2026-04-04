# uart_control

## 概要

`uart_control` は、ROS 2 とローバー内の UART 機器をつなぐ双方向ブリッジです。  
このワークスペースでは、センサ MCU からの受信と、駆動 MCU への送信の両方をこのパッケージが担当しています。

## このパッケージが担当すること

- UART 受信データを ROS トピックへ流す
- ROS から来た制御コマンドを UART に戻す
- シリアル切断時の再接続を行う

## 主な実行ファイル

### `serial_subscriber`

センサ側の UART を読み、`uart_data` として publish します。

デフォルト設定:

- ポート: `/dev/ttyUSB0`
- Baudrate: `115200`

想定する受信形式:

```text
408,-65.88
415,0.00000000000
```

ROS では以下の形に変換されます。

```text
Float64MultiArray.data = [ID, DATA]
```

Publish:

- `uart_data`

このトピックを `ares_sensor` が購読して、IMU や GPS を復元します。

### `serial_publiasher`

制御側の UART 送信ノードです。  
名前に typo がありますが、現状の launch や setup もこの綴りを前提にしているため、そのまま使われています。

デフォルト設定:

- ポート: `/dev/ttyUSB1`
- Baudrate: `115200`

Subscribe:

- `uart_command`

期待するメッセージ形式:

```text
Int16MultiArray.data = [angle_id, angle_val, speed_id, speed_val]
```

ノード内部ではこれを 2 行の UART 文字列に分解して送信します。

```text
0x310,180
0x312,140
```

## 主な launch

- `launch/serial_subscriber.launch.py`
- `launch/serial_publiasher.launch.py`

## このワークスペースの中での位置づけ

`uart_control` は、ROS 側とハードウェア側の境界にいるパッケージです。

受信側の流れ:

1. `serial_subscriber`
2. `uart_data`
3. `ares_sensor`
4. `imu/data` と `gps/fix`

送信側の流れ:

1. `rover_controller` または `ares_nav2` または `YOLO_detection_v2`
2. `uart_command`
3. `serial_publiasher`
4. MCU

## どこで使われているか

- `ares_nav2/controller_bringup.launch.py`
  - 受信ノードと送信ノードの両方を起動
- `ares_sensor`
  - `uart_data` を利用
- `rover_controller`
  - `uart_command` を publish
- `ares_nav2`
  - waypoint 到達時の停止コマンドを `uart_command` に publish
- `YOLO_detection_v2/make_direction.py`
  - 停止判定時に `uart_command` を publish

## 初見の人が最初に確認するとよいファイル

- `uart_control/serial_subscriber.py`
- `uart_control/serial_publiasher.py`
- `launch/serial_subscriber.launch.py`
- `launch/serial_publiasher.launch.py`

## 補足

- シリアルポートが抜けても定期再接続する実装になっているため、実機 bringup 時の耐久性を意識した作りです。
- `uart_data` と `uart_command` は、このワークスペース固有の中間表現です。外部パッケージは基本的にこの形式を直接は知りません。
