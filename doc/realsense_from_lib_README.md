# realsense_from_lib

## 概要

`realsense_from_lib` は、`pyrealsense2` を使って Intel RealSense の RGB・Depth・CameraInfo を ROS 2 に配信する自作パッケージです。  
このワークスペースでは、ArUco 認識と YOLO 認識の両方の入力源になっています。

## このパッケージが担当すること

- RealSense ストリームの開始
- RGB と Depth の取得
- Depth を Color にアラインして配信
- `CameraInfo` の生成
- カメラ光学フレームの static TF 配信
- 深度変換に必要な内部パラメータの簡易 publish

## 実行ファイル

### `publish_realsense`

Publish:

- `camera/color/image_raw`
- `camera/depth/image_raw`
- `camera/color/camera_info`
- `realsense_info`

配信する追加情報:

- `realsense_info`
  - `depth_scale`
  - `fx`
  - `fy`

配信する TF:

- `camera_link -> camera_color_optical_frame`
- `camera_link -> camera_depth_optical_frame`

## パラメータ

主なパラメータ:

- `color_width`
- `color_height`
- `depth_width`
- `depth_height`
- `fps`
- `color_topic`
- `depth_topic`
- `camera_info_topic`
- `camera_link_frame`
- `color_optical_frame`
- `depth_optical_frame`

## 主な launch

- `launch/publish_realsense.launch.py`

既定値では 640x480, 15fps で起動するようになっています。

## このワークスペースの中での位置づけ

このパッケージは、現在の主要 launch で使われている RealSense 入力の本命です。

利用先:

- `aruco_opencv`
  - カラー画像と CameraInfo を使用
- `YOLO_detection_v2`
  - カラー画像、Depth、CameraInfo、`realsense_info` を使用
- RViz
  - カメラ画像の可視化に使用

## どこで使われているか

- `ares_nav2/controller_bringup.launch.py`
  - `publish_realsense.launch.py` を読み込んで起動

## 初見の人が最初に確認するとよいファイル

- `realsense_from_lib/publish_realsense.py`
- `launch/publish_realsense.launch.py`

## 補足

- `align.process` により Depth を Color に合わせているため、YOLO の検出 box と Depth の対応が取りやすくなっています。
- 公式 `realsense2_camera` ではなく独自実装を使っているため、認識パイプラインに必要な最小限の情報だけを明示的に出したい、という意図が読み取れます。
