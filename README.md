# URC2026 Autonomous Controller Workspace

## 概要

この `src` ディレクトリは、URC2026 向けローバーの ROS 2 ワークスペースです。  
GPS waypoint ナビゲーション、ArUco / YOLO による最終接近、UART を介した車体制御、RealSense / LiDAR 入力を組み合わせた構成になっています。

大まかな流れは次の通りです。

1. `ares_sensor` が GPS / IMU を ROS topic に変換する
2. `ares_nav2` が waypoint を Nav2 goal に変換する
3. 必要に応じて ArUco または YOLO 接近フェーズへ切り替える
4. `rover_controller` と `uart_control` が MCU へ最終コマンドを送る

## 主要 Launch

### 制御・センサ bringup

```bash
ros2 launch ares_nav2 controller_bringup.launch.py
```

主な起動対象:

- UART subscriber / publisher
- RealSense
- ArUco tracker
- ares_sensor
- rover_controller
- YOLO TF node
- Livox driver

### Nav2 bringup

```bash
ros2 launch ares_nav2 navigation_sim.launch.py
```

主な起動対象:

- `robot_localization`
- Nav2
- `pointcloud_to_laserscan`
- RViz

### ミッション制御

```bash
ros2 launch ares_nav2 main.launch.py
```

主な起動対象:

- `gps_waypoint_follower_node`
- `aruco_nav2_goal_node`
- `yolo_tf_nav2_goal_node`

## パッケージ一覧

### 中核パッケージ

- [ares_nav2](./ares_nav2/README.md)
  GPS waypoint 管理、ArUco / YOLO 接近制御、Nav2 連携
- [ares_sensor](./doc/ares_sensor_README.md)
  UART 由来の GPS / IMU データを ROS topic に変換
- [rover_controller](./doc/rover_controller_README.md)
  `/cmd_vel` をローバー向け `uart_command` に変換
- [uart_control](./doc/uart_control_README.md)
  UART 送受信ブリッジ

### 認識・センサ

- [YOLO_detection_v2](./YOLO_detection_v2/README.md)
  YOLO 物体検出、TF 化、簡易追従
- [aruco_opencv](./aruco_opencv/README.md)
  ArUco 検出、`/aruco/id` と `aruco_marker` TF の供給
- [realsense_from_lib](./doc/realsense_from_lib_README.md)
  RealSense の RGB / Depth / CameraInfo 配信
- [livox_ros_driver2](./livox_ros_driver2/README.md)
  Livox MID360 ドライバ
- [livox_to_pointcloud2](./livox_to_pointcloud2/README.md)
  Livox 点群を `PointCloud2` に変換
- [pointcloud_to_laserscan](./doc/pointcloud_to_laserscan_README.md)
  3D 点群を `/scan` に変換して Nav2 に渡す

### 可視化・補助

- [doc/README.md](./doc/README.md)
  パッケージ別ドキュメントの入口
- [rviz_2d_overlay_plugins](./rviz_2d_overlay_plugins/README.md)
  RViz オーバーレイ表示プラグイン

## フォルダ構成

```text
src/
├── README.md
├── doc/
├── ares_nav2/
├── ares_sensor/
├── YOLO_detection_v2/
├── aruco_opencv/
├── uart_control/
├── rover_controller/
├── realsense_from_lib/
├── livox_ros_driver2/
├── livox_to_pointcloud2/
├── pointcloud_to_laserscan-humble/
└── rviz_2d_overlay_plugins/
```

## よく見るファイル

- `ares_nav2/config/waypoints.yaml`
  GPS waypoint と ArUco / YOLO 条件
- `ares_nav2/launch/controller_bringup.launch.py`
  センサ・制御 bringup
- `ares_nav2/launch/navigation_sim.launch.py`
  Nav2 bringup
- `ares_nav2/launch/main.launch.py`
  ミッション制御ノード起動
- `YOLO_detection_v2/YOLO_detection_v2/YOLOtf.py`
  YOLO から TF を出す本番系実装

## 最近の編集履歴

### 2026-04-15

- `YOLO_detection_v2/publish_YOLOtf` を `ares_nav2/controller_bringup.launch.py` から起動する構成に変更
- `ares_nav2/main.launch.py` は mission control ノードのみを起動する構成に整理
- `gps_waypoint_follower` から `/yolo/enabled` を publish するように変更
- YOLO は対象 waypoint のときだけ有効化する軽量運用に変更
- 学習済みモデル `train260205s_best.pt` を `src/YOLO_detection_v2/` に配置する前提へ整理
- `waypoints.yaml` の YOLO ターゲット名 `hummer` を `hammer` に修正
- `ares_nav2` と `YOLO_detection_v2` にパッケージ README を追加

## 運用メモ

- 実運用では `controller_bringup.launch.py`、`navigation_sim.launch.py`、`main.launch.py` を役割ごとに分けて起動します。
- YOLO ノードは起動していても、`/yolo/enabled` が `false` の間は待機状態です。
- ROS Humble と `cv_bridge` を使う都合上、Python 環境の NumPy は `numpy<2` が安全です。
