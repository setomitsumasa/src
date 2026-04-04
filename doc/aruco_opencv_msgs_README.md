# aruco_opencv_msgs

## 概要

`aruco_opencv_msgs` は、`aruco_opencv` が使うメッセージ型を定義するためのパッケージです。  
認識ロジックそのものは持たず、「ArUco 検出結果をどう ROS メッセージで表現するか」を提供します。

## 定義している主なメッセージ

### `MarkerPose.msg`

- `marker_id`
- `geometry_msgs/Pose pose`

1 枚のマーカーの ID と姿勢を表します。

### `BoardPose.msg`

- `board_name`
- `geometry_msgs/Pose pose`

複数マーカーで構成されるボードの姿勢を表します。

### `ArucoDetection.msg`

- `std_msgs/Header header`
- `MarkerPose[] markers`
- `BoardPose[] boards`

1 フレーム内の ArUco 検出結果全体を表します。

## このワークスペースでの位置づけ

`ares_nav2` が直接使っているのは `/aruco/id` と `aruco_marker` TF ですが、`aruco_opencv` のより本来の出力はこのメッセージ群です。  
つまりこのパッケージは、「詳細な検出情報」を保持する層です。

## どこで使われているか

- `aruco_opencv`

## 初見の人が最初に確認するとよいファイル

- `msg/MarkerPose.msg`
- `msg/BoardPose.msg`
- `msg/ArucoDetection.msg`

## 補足

- 今のミッション制御では、詳細な `ArucoDetection` 全体ではなく、簡略化した `/aruco/id` と `aruco_marker` TF が主に使われています。
