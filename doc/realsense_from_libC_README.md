# realsense_from_libC

## 概要

`realsense_from_libC` は、`librealsense2` を使った C++ 実装の RealSense 配信パッケージです。  
役割は `realsense_from_lib` とほぼ同じですが、こちらは簡易版または代替実装という位置づけです。

## このパッケージが担当すること

- RealSense の RGB と Depth を取得
- Depth を Color にアラインして publish
- `realsense_info` を publish

## 実行ファイル

### `publish_realsense`

Publish:

- `camera/color/image_raw`
- `camera/depth/image_raw`
- `realsense_info`

`realsense_info` には以下が入ります。

- `depth_scale`
- `fx`
- `fy`

## `realsense_from_lib` との違い

- C++ 実装
- `CameraInfo` の publish がない
- static TF の publish がない
- 主要 launch からは現状呼ばれていない

## このワークスペースの中での位置づけ

現状の統合 bringup では `realsense_from_lib` が使われており、`realsense_from_libC` は予備実装または検証用の立ち位置です。  
Python 実装の代替候補として残してあると考えると分かりやすいです。

## どこで使われているか

- 現在の `ares_nav2/controller_bringup.launch.py` からは使われていません

## 初見の人が最初に確認するとよいファイル

- `src/publish_realsense.cpp`
- `CMakeLists.txt`

## 補足

- YOLO や ArUco まで含めた現行パイプラインでそのまま使うには、`CameraInfo` や TF の不足分を別途補う必要があります。
