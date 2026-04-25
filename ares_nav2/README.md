# ares_nav2

## 概要

`ares_nav2` は、このワークスペースのナビゲーション統括パッケージです。  
GPS waypoint による移動、ArUco/YOLO を使った最終接近、Nav2 や `robot_localization` の起動設定をまとめて扱います。

このパッケージを見ると、ローバーが

1. GPS waypoint に向かう
2. 必要ならスパイラル探索する
3. ArUco または YOLO による接近へ切り替える
4. 次の waypoint に進む

という流れをどう実装しているかが分かります。

## 主なノード

### `gps_waypoint_follower_node`

- `config/waypoints.yaml` を読み込みます
- 最初の `gps/fix` を基準に waypoint をローカル座標へ変換します
- waypoint ごとに ArUco / YOLO 接近フェーズへ切り替えます
- waypoint 到達時に `uart_command` へ停止コマンドを送ります

主な topic:

- Subscribe: `gps/fix`
- Subscribe: `/aruco/id`
- Subscribe: `/aruco/goal_reached`
- Subscribe: `/yolo/goal_reached`
- Publish: `/aruco/enabled`
- Publish: `/aruco/target_marker_id`
- Publish: `/yolo/enabled`
- Publish: `/yolo/target_frame`
- Publish: `uart_command`

### `aruco_nav2_goal_node`

- `aruco_marker` TF を Nav2 の goal に変換します
- `/aruco/target_marker_id` で指定された ID のマーカーだけを追跡します
- 到達時に `/aruco/goal_reached` を publish します

### `yolo_tf_nav2_goal_node`

- `mallet` `hammer` `bottle` などの TF を Nav2 の goal に変換します
- `/yolo/target_frame` で追跡対象を切り替えます
- 到達時に `/yolo/goal_reached` を publish します

## Launch

### `launch/controller_bringup.launch.py`

センサ・制御系の bringup です。主に次を起動します。

- `uart_control`
- `realsense_from_lib`
- `aruco_opencv`
- `ares_sensor`
- `rover_controller`
- `YOLO_detection_v2/publish_YOLOtf`
- `livox_ros_driver2`
- `livox_to_pointcloud2`

### `launch/navigation_sim.launch.py`

Nav2 側の bringup です。主に次を起動します。

- `dual_ekf_navsat.launch.py`
- `nav2_bringup/navigation_launch.py`
- `pointcloud_to_laserscan`
- RViz

### `launch/main.launch.py`

ミッション制御ノードをまとめて起動します。

- `gps_waypoint_follower_node`
- `aruco_nav2_goal_node`
- `yolo_tf_nav2_goal_node`

## フォルダ構成

```text
ares_nav2/
├── config/
│   ├── waypoints.yaml
│   ├── nav2_no_map_params.yaml
│   ├── dual_ekf_navsat_params.yaml
│   ├── custom_nav_to_pose.xml
│   └── custom_nav_through_poses.xml
├── include/ares_nav2/
│   ├── gps_waypoint_follower.hpp
│   ├── aruco_nav2_goal.hpp
│   └── yolo_tf_nav2_goal.hpp
├── launch/
│   ├── controller_bringup.launch.py
│   ├── navigation_sim.launch.py
│   ├── main.launch.py
│   ├── gps_waypoint_follower.launch.py
│   ├── aruco_nav2_goal.launch.py
│   └── yolo_tf_nav2_goal.launch.py
├── src/
│   ├── gps_waypoint_follower.cpp
│   ├── aruco_nav2_goal.cpp
│   ├── yolo_tf_nav2_goal.cpp
│   ├── waypoint_loader.cpp
│   └── geo_utils.cpp
├── CMakeLists.txt
└── package.xml
```

## 主要ファイル

- `config/waypoints.yaml`
  waypoint と、到達後に使う ArUco / YOLO 条件を定義します。
- `src/gps_waypoint_follower.cpp`
  ミッションの流れを決める中心実装です。
- `src/aruco_nav2_goal.cpp`
  ArUco の TF を Nav2 goal に変換します。
- `src/yolo_tf_nav2_goal.cpp`
  YOLO 物体 TF を Nav2 goal に変換します。

## 最近の編集履歴

### 2026-04-15

- `controller_bringup.launch.py` から `YOLO_detection_v2/publish_YOLOtf` を起動する構成に変更
- `main.launch.py` は mission control ノードだけを起動する構成に整理
- `gps_waypoint_follower` に `/yolo/enabled` publish を追加
- YOLO waypoint のときだけ YOLO を有効化する軽量運用に変更
- `waypoints.yaml` の `hummer` を `hammer` に修正

## 運用メモ

- YOLO 接近は常時有効ではなく、YOLO 対象 waypoint のときだけ有効になります。
- `controller_bringup.launch.py` の `sim:=true` では `publish_YOLOtf` は起動しません。
- Nav2 側と制御系 bringup は別 launch なので、実運用では `controller_bringup.launch.py` と `navigation_sim.launch.py` と `main.launch.py` を役割ごとに組み合わせて使います。
