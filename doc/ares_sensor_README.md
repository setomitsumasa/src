# ares_sensor

## 概要

`ares_sensor` は、ローバーのセンサ情報を ROS で扱いやすい標準トピックへ変換するパッケージです。  
UART から届く生データをそのまま各ノードが読むのではなく、このパッケージが `imu/data`、`gps/fix`、各種 TF、`imu/roll` などの中間表現を作ることで、Nav2 や認識系が使いやすい形に整えています。

## このパッケージが担当すること

- `uart_data` から IMU を復元
- `uart_data` から GPS を復元
- IMU のクォータニオンから roll/pitch/yaw を計算
- `base_link` 周りの主要な TF をまとめて配信
- 必要なら点群の姿勢補正も行う

## 主な実行ファイル

### `imu_node`

`uart_data` を購読し、IMU の各成分を `sensor_msgs/msg/Imu` に変換して `imu/data` に publish します。

UART ID の対応:

- `400` gyro.x
- `401` gyro.y
- `402` gyro.z
- `403` acc.x
- `404` acc.y
- `405` acc.z
- `412` yaw

このノードがいるおかげで、上位側は UART ID を知らなくても IMU として扱えます。

### `gps_node`

`uart_data` から緯度経度を抜き出し、`sensor_msgs/msg/NavSatFix` として `gps/fix` に publish します。

UART ID の対応:

- `415` latitude
- `416` longitude

`robot_localization/navsat_transform_node` が使う入力をここで作っています。

### `sensor_node`

`imu/data` を購読し、クォータニオンを roll/pitch/yaw に変換して以下を publish します。

- `imu/roll`
- `imu/pitch`
- `imu/yaw`

Nav2 本体が直接これを使うわけではありませんが、姿勢可視化や点群補正、デバッグに便利です。

### `sensor_tf_node`

ロボット座標系の基本 TF をまとめて出すノードです。

主なフレーム:

- `map -> odom`
- `base_link -> imu_link`
- `base_link -> gps_link`
- `base_link -> camera_link`
- `base_link -> livox_frame`
- `camera_color_frame -> camera_color_optical_frame`
- `camera_depth_frame -> camera_depth_optical_frame`

`robot_localization`、Nav2、ArUco、YOLO、LiDAR 変換など、ほぼ全体がこの TF を前提に動きます。

### `pointcloud_stabilizer_node`

`imu/roll`、`imu/pitch`、`imu/yaw` を使って点群の姿勢を補正するノードです。

- Subscribe: `/realsense/cloud`
- Publish: `/realsense/cloud/stabilized`

ただし、現状の主要 launch からは呼ばれていないため、補助機能または検証用の位置づけです。

## 主な launch

### `sensor_data_publisher.launch.py`

センサ関連の主要ノードをまとめて起動します。

起動内容:

- `imu_node`
- `sensor_node`
- `gps_node`
- `sensor_tf_node`

### `imu_rpy_publisher.launch.py`

`sensor_node` だけを単独で起動する簡易 launch です。

## このワークスペースの中での位置づけ

`uart_control` が「UART を ROS に入れる入口」だとすると、`ares_sensor` は「その生データを ROS 標準の意味ある情報に変える層」です。

データの流れ:

1. `uart_control/serial_subscriber` が `uart_data` を出す
2. `imu_node` が `imu/data` を作る
3. `gps_node` が `gps/fix` を作る
4. `sensor_node` が `imu/roll` `imu/pitch` `imu/yaw` を作る
5. `sensor_tf_node` が座標系を整える

## どこで使われているか

- `ares_nav2/navigation_sim.launch.py`
  - `imu_rpy_publisher.launch.py` を使用
- `ares_nav2/controller_bringup.launch.py`
  - `sensor_data_publisher.launch.py` を使用
- `robot_localization`
  - `imu/data` と `gps/fix` を使用
- ArUco/YOLO/RealSense/LiDAR 系
  - `base_link` 周辺の TF を使用

## 初見の人が最初に確認するとよいファイル

- `launch/sensor_data_publisher.launch.py`
- `src/imu.cpp`
- `src/gps.cpp`
- `src/imu_utils.cpp`
- `src/sensor_tf.cpp`

## 補足

- `config/config.yaml` は `sensor_node` がどの IMU トピックを読むかを決めるだけの軽い設定です。
- 姿勢のヨー角は UART 側の方位角定義を ROS の ENU 系へ変換してからクォータニオン化しています。
