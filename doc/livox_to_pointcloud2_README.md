# livox_to_pointcloud2

## 概要

`livox_to_pointcloud2` は、`livox_ros_driver2` が publish する Livox 独自形式の点群を、一般的な `sensor_msgs/msg/PointCloud2` に変換するパッケージです。  
Nav2 や RViz、一般的な ROS ツールとつなぐための中継役です。

## このパッケージが担当すること

- `livox_ros_driver2::msg::CustomMsg` の購読
- `PointCloud2` 形式への変換
- `x y z intensity tag line` フィールドの整形

## 実行ファイル

### `livox_to_pointcloud2_node`

Subscribe:

- `livox_pointcloud`

Publish:

- `converted_pointcloud2`

現在の実装は、Livox 独自データに含まれる各点をそのまま `PointCloud2` の field に詰め替える、非常に素直な変換です。

## このワークスペースでの使われ方

`ares_nav2/controller_bringup.launch.py` では、入力だけが次のように remap されています。

- `/livox_pointcloud` -> `/livox/lidar`

そのため、現在の bringup では最終出力は次のトピック名のままです。

- `/converted_pointcloud2`

この `/converted_pointcloud2` を、さらに `pointcloud_to_laserscan` が受け取って `/scan` を作ります。

## どこで使われているか

- `ares_nav2/controller_bringup.launch.py`
- `pointcloud_to_laserscan` の入力元

## 初見の人が最初に確認するとよいファイル

- `src/livox_to_pointcloud2.cpp`
- `launch/livox_to_pointcloud2.launch.yml`

## 補足

- `launch/livox_to_pointcloud2.launch.yml` には `/converted_pointcloud2` を `/livox/lidar/pcd2` へ remap する例がありますが、現在の統合 launch では使われていません。
- このパッケージがあることで、LiDAR ラインは「Livox 専用」から「一般的な ROS 点群」へ変換されます。
