# パッケージ別ドキュメント

この `doc` ディレクトリは、このワークスペースに含まれる各 ROS 2 パッケージの役割を、初めて見る人でも追いやすい形で整理したものです。  
特にこのリポジトリは、`ares_nav2` を中心に、UART 経由のセンサ入力、RealSense/Livox/ArUco/YOLO の認識、`cmd_vel` から MCU 向けコマンドへの変換、という複数の流れが組み合わさっているため、パッケージ単位で見ると全体像をつかみやすくなります。

## まず何を読むべきか

1. [ares_nav2](./ares_nav2_README.md)
2. [ares_sensor](./ares_sensor_README.md)
3. [uart_control](./uart_control_README.md)
4. [rover_controller](./rover_controller_README.md)
5. [realsense_from_lib](./realsense_from_lib_README.md)
6. [aruco_opencv](./aruco_opencv_README.md)
7. [YOLO_detection_v2](./YOLO_detection_v2_README.md)
8. [livox_ros_driver2](./livox_ros_driver2_README.md)
9. [livox_to_pointcloud2](./livox_to_pointcloud2_README.md)
10. [pointcloud_to_laserscan](./pointcloud_to_laserscan_README.md)

## 全体構成

### 自作の中核パッケージ

| パッケージ | 役割 |
| --- | --- |
| [ares_nav2](./ares_nav2_README.md) | Nav2 起動、GPS waypoint 管理、ArUco/YOLO 接近フェーズ制御 |
| [ares_sensor](./ares_sensor_README.md) | UART の生データを IMU/GPS/TF に変換 |
| [uart_control](./uart_control_README.md) | UART 受信と UART 送信の双方向ブリッジ |
| [rover_controller](./rover_controller_README.md) | `/cmd_vel` をローバー向け `uart_command` に変換 |

### 認識・センサ系

| パッケージ | 役割 |
| --- | --- |
| [realsense_from_lib](./realsense_from_lib_README.md) | RealSense の RGB/Depth/CameraInfo 配信 |
| [realsense_from_libC](./realsense_from_libC_README.md) | 上記の C++ 試作版 |
| [aruco_opencv](./aruco_opencv_README.md) | ArUco 検出、`/aruco/id` と `aruco_marker` TF の供給 |
| [aruco_opencv_msgs](./aruco_opencv_msgs_README.md) | ArUco 検出メッセージ定義 |
| [YOLO_detection_v2](./YOLO_detection_v2_README.md) | YOLO による物体検出、TF 化、簡易追従 |
| [livox_ros_driver2](./livox_ros_driver2_README.md) | Livox MID360 ドライバ |
| [livox_to_pointcloud2](./livox_to_pointcloud2_README.md) | Livox 独自形式を `PointCloud2` に変換 |
| [pointcloud_to_laserscan](./pointcloud_to_laserscan_README.md) | 3D 点群を `/scan` に変換して Nav2 に渡す |

### 可視化・補助ツール

| パッケージ | 役割 |
| --- | --- |
| [rviz_nav2](./rviz_nav2_README.md) | `/aruco/id` を表示する RViz パネル |
| [rviz_2d_overlay_msgs](./rviz_2d_overlay_msgs_README.md) | RViz 2D オーバーレイ用メッセージ |
| [rviz_2d_overlay_plugins](./rviz_2d_overlay_plugins_README.md) | RViz 2D オーバーレイ表示プラグイン |
| [log_maker](./log_maker_README.md) | 実験ログ収集用の補助パッケージ |

## 運用上の大きな流れ

### 1. センサ入力

- `uart_control/serial_subscriber` が `/dev/ttyUSB0` から UART データを受信し、`uart_data` に流します。
- `ares_sensor/imu_node` が `uart_data` を `imu/data` に変換します。
- `ares_sensor/gps_node` が `uart_data` を `gps/fix` に変換します。
- `ares_sensor/sensor_node` が `imu/data` から `imu/roll` `imu/pitch` `imu/yaw` を作ります。
- `ares_sensor/sensor_tf_node` が `base_link` 周りの静的 TF をまとめて供給します。

### 2. 認識入力

- `realsense_from_lib` が `camera/color/image_raw` と `camera/depth/image_raw` を配信します。
- `aruco_opencv` がカメラ画像から ArUco を検出し、`/aruco/id` と `aruco_marker` TF を出します。
- `YOLO_detection_v2/YOLOtf.py` が YOLO 検出結果を `mallet` `hammer` `bottle` などの TF に変換します。
- `livox_ros_driver2` と `livox_to_pointcloud2` が LiDAR 点群を用意し、`pointcloud_to_laserscan` が `/scan` を作ります。

### 3. ナビゲーションと制御

- `ares_nav2/navigation_sim.launch.py` が `robot_localization`、Nav2、ArUco、`pointcloud_to_laserscan`、RViz などを起動します。
- `ares_nav2/gps_waypoint_follower_node` が `config/waypoints.yaml` を読み、GPS waypoint を順番に Nav2 ゴールへ変換します。
- waypoint ごとに必要なら ArUco 接近フェーズまたは YOLO 接近フェーズへ制御を渡します。
- Nav2 や `YOLO_detection_v2/make_direction.py` が出した `/cmd_vel` は、`rover_controller` で `uart_command` に変換され、最後に `uart_control/serial_publiasher` が MCU に送信します。

## launch の見方

- `ares_nav2/controller_bringup.launch.py`
  - センサ、UART、RealSense、ArUco、Livox、ローバー制御をまとめて起動する統合 bringup
- `ares_nav2/navigation_sim.launch.py`
  - `robot_localization` と Nav2、本番系の自己位置推定・障害物回避スタック
- `ares_nav2/main.launch.py`
  - waypoint follower と ArUco/YOLO 接近ノードをまとめて起動

## 読むときの注意点

- このワークスペースには、自作パッケージと外部由来パッケージが混在しています。
- 外部由来パッケージも、このリポジトリ内では「そのまま使っているもの」と「ワークスペース向けに少し手を入れているもの」があります。
- 一部の補助パッケージは、現状の主要 launch からは呼ばれていません。各 README では、その点も含めて「今の運用で使われているかどうか」を明記しています。
