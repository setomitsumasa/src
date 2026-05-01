# ares_nav2

## 概要

`ares_nav2` は、このワークスペースの中核にあるナビゲーション統括パッケージです。  
Nav2 の起動そのものに加えて、「GPS waypoint に向かう」「到着後に ArUco や YOLO の認識結果を使って最終接近する」といったミッション手順もここで管理しています。

このパッケージを読むと、このローバーがどの順番で動くのかが最もよく分かります。

## このパッケージが担当すること

- `robot_localization` と Nav2 の起動設定
- GPS 座標を Nav2 の `NavigateToPose` ゴールへ変換
- waypoint ごとの ArUco 接近条件と YOLO 接近条件の管理
- LiDAR を `/scan` に変換して Nav2 costmap に渡す構成
- 独自 Behavior Tree とリカバリ動作の差し替え

## 主な実行ファイル

### `gps_waypoint_follower_node`

`config/waypoints.yaml` を読み、`gps/fix` で得た最初の GNSS 座標を基準点として、各 waypoint をローカル平面座標に変換して Nav2 に送ります。

特徴:

- waypoint ごとに `yaw` を設定可能
- `spiral_search: true` のときは、到着後にらせん状の探索 waypoint を追加送信
- spiral 探索点に到着したら、その場で `/cmd_vel` 回転スキャンを行い、対象が見えたら接近フェーズへ移行
- `aruco` と `marker_id` が設定されていれば、ArUco 接近フェーズへ移行
- `yolo` が設定されていれば、YOLO 物体接近フェーズへ移行
- waypoint 到達時に `uart_command` へ停止コマンドを送る

関係する主なトピック:

- Subscribe: `gps/fix`
- Subscribe: `/aruco/id`
- Subscribe: `/aruco/goal_reached`
- Subscribe: `/yolo/goal_reached`
- Publish: `/aruco/enabled`
- Publish: `/aruco/target_marker_id`
- Publish: `/yolo/target_frame`
- Publish: `/cmd_vel`
- Publish: `uart_command`

回転スキャンの主なパラメータ:

- `spiral_spin_scan_enabled`: spiral 探索点でのその場回転を有効化
- `spiral_spin_scan_total_angle_rad`: 1地点で回す合計角度。既定値は `2π`
- `spiral_spin_scan_angular_speed_rad_s`: 回転中に `/cmd_vel.angular.z` へ出す角速度
- `spiral_spin_scan_direction`: `1` で左回り、`-1` で右回り

`main.launch.py` では同名の launch argument で上書きできます。

### `aruco_nav2_goal_node`

ArUco マーカーの TF を Nav2 ゴールに変換するノードです。  
`/aruco/target_marker_id` で指定された ID と、実際に検出された `/aruco/id` が一致したときだけ `aruco_marker` TF を採用し、Nav2 にゴールを送り続けます。

役割:

- waypoint follower から「どの ArUco を追うべきか」を受け取る
- `aruco_marker` TF を `map` 基準で取得
- マーカー位置が動いたらゴールを更新
- 到達したら `/aruco/goal_reached` を publish

### `yolo_tf_nav2_goal_node`

YOLO 検出から生成された TF を Nav2 ゴールに変換するノードです。  
`/yolo/target_frame` に `mallet` や `bottle` などの名前が流れてくると、その TF を追跡対象として扱います。

役割:

- waypoint follower から「どの YOLO 物体を追うか」を受け取る
- `map -> <target frame>` の TF を監視
- TF が有効かつ新しければ Nav2 へゴール送信
- 到達したら `/yolo/goal_reached` を publish

### `ares_nav2_bt_nodes`

Behavior Tree 用の独自ノード群です。  
現状は `RoverRecovery` が含まれており、`cmd_vel` に後退速度を流して単純なバック動作をさせます。

## 主な launch

### `navigation_sim.launch.py`

このワークスペースで Nav2 を動かすための中心 launch です。

起動するもの:

- `dual_ekf_navsat.launch.py`
- `nav2_bringup/navigation_launch.py`
- `ares_sensor/imu_rpy_publisher.launch.py`
- `aruco_opencv/aruco_tracker.launch.xml`
- `pointcloud_to_laserscan/sample_pointcloud_to_laserscan_launch.py`
- RViz

### `controller_bringup.launch.py`

ローバーのセンサ・制御系をまとめて立ち上げる bringup です。

起動するもの:

- `uart_control` の受信ノードと送信ノード
- `realsense_from_lib`
- `aruco_opencv`
- `ares_sensor`
- `rover_controller`
- `livox_ros_driver2`
- `livox_to_pointcloud2`

### `main.launch.py`

ミッション制御ノードだけをまとめて起動する launch です。

- `gps_waypoint_follower_node`
- `aruco_nav2_goal_node`
- `yolo_tf_nav2_goal_node`

## 設定ファイル

- `config/waypoints.yaml`
  - 目的地の緯度経度と、到達後に使う ArUco/YOLO 条件
- `config/nav2_no_map_params.yaml`
  - map-less 運用向けの Nav2 パラメータ
- `config/dual_ekf_navsat_params.yaml`
  - `robot_localization` の EKF 設定
- `config/custom_nav_to_pose.xml`
  - Nav2 の NavigateToPose 用 BT
- `config/custom_nav_through_poses.xml`
  - Nav2 の NavigateThroughPoses 用 BT

## このワークスペースの中での位置づけ

`ares_nav2` は、単に Nav2 を置いているだけのパッケージではありません。  
このロボット特有の運用フローを Nav2 に橋渡ししている「ミッションオーケストレータ」と考えると理解しやすいです。

整理すると次の流れです。

1. `gps/fix` から基準点を決める
2. waypoint を `map` 座標へ変換して Nav2 に送る
3. waypoint に ArUco 条件があるなら ArUco 接近へ移る
4. waypoint に YOLO 条件があるなら YOLO 接近へ移る
5. 完了したら次の waypoint へ進む

## 初見の人が最初に確認するとよいファイル

- `launch/navigation_sim.launch.py`
- `launch/controller_bringup.launch.py`
- `src/gps_waypoint_follower.cpp`
- `src/aruco_nav2_goal.cpp`
- `src/yolo_tf_nav2_goal.cpp`
- `config/waypoints.yaml`

## 補足

- `waypoints.yaml` の `aruco` や `yolo` は、単なるメモではなく実際の制御分岐に使われます。
- LiDAR は `pointcloud_to_laserscan` を通して `/scan` に変換される前提です。Nav2 の costmap 設定もその前提で書かれています。
- GPS を基準にした map-less ナビゲーションなので、通常の 2D 地図ベース Nav2 と比べると設定の考え方が少し異なります。
