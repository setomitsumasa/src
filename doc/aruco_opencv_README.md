# aruco_opencv

## 概要

`aruco_opencv` は、OpenCV の ArUco モジュールを使ってマーカーを検出するパッケージです。  
このワークスペースでは、外部由来パッケージをベースにしつつ、`/aruco/id` と `aruco_marker` TF を使いやすく出すようにしてあります。

## このパッケージが担当すること

- カメラ画像から ArUco を検出
- 各マーカーの pose 推定
- ArUco 検出結果メッセージの publish
- 最初に検出したマーカー ID の publish
- `aruco_marker` TF の publish
- `/aruco/enabled` による検出の有効化制御

## 主な出力

- `aruco_detections`
  - 詳細な検出結果
- `/aruco/id`
  - 最初に見つかったマーカー ID を `Float32` として publish
- `aruco_marker`
  - 最初に見つかったマーカーの TF

この 2 つの簡易出力が、`ares_nav2` と直接つながっています。

## 主な入力

- `camera/color/image_raw`
- `camera/color/camera_info`
- `/aruco/enabled`

## 主な launch

### `launch/aruco_tracker.launch.xml`

以下を読み込みます。

- `config/aruco_tracker.yaml`
- `config/board_descriptions.yaml`

## このワークスペースでの使われ方

`ares_nav2` 側では次のように使われます。

1. `gps_waypoint_follower_node` が waypoint 到達後に `/aruco/target_marker_id` を publish
2. `aruco_opencv` がカメラ画像からマーカーを検出して `/aruco/id` と `aruco_marker` TF を出す
3. `aruco_nav2_goal_node` が対象 ID と一致したときだけ `aruco_marker` を Nav2 ゴールにする

つまり、ArUco は単なる可視化ではなく、GPS waypoint の最後の接近フェーズに使われています。

## どこで使われているか

- `ares_nav2/navigation_sim.launch.py`
- `ares_nav2/controller_bringup.launch.py`

## 初見の人が最初に確認するとよいファイル

- `src/aruco_tracker.cpp`
- `launch/aruco_tracker.launch.xml`
- `config/aruco_tracker.yaml`
- `config/board_descriptions.yaml`

## 補足

- `marker_size` や board 設定は距離推定に直結するため、実機のマーカー寸法に合わせて必ず確認するべき箇所です。
- このワークスペースでは `/aruco/id` と `aruco_marker` TF の存在が重要で、`ares_nav2` はそれを前提に組まれています。
