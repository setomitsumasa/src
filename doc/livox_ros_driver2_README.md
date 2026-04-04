# livox_ros_driver2

## 概要

`livox_ros_driver2` は、Livox MID360 などの LiDAR を ROS 2 に接続する外部由来パッケージです。  
このワークスペースでは、LiDAR の生データを ROS に取り込む最初の入口として使われています。

## このパッケージが担当すること

- Livox センサとの通信
- Livox 独自形式または PointCloud2 形式での点群配信
- LiDAR 固有設定の読み込み

## このワークスペースでの使われ方

`ares_nav2/controller_bringup.launch.py` では、以下の launch を呼び出しています。

- `launch_ROS2/msg_MID360_launch.py`

ここで重要なのは、現在の構成では「Livox 独自形式のメッセージ」を出す設定を使っている点です。  
そのため、後段に `livox_to_pointcloud2` が必要になります。

## 典型的な後段構成

1. `livox_ros_driver2`
2. `livox_to_pointcloud2`
3. `pointcloud_to_laserscan`
4. Nav2 の `/scan`

## なぜこのパッケージ単体で終わらないのか

Nav2 は通常 `LaserScan` または `PointCloud2` を直接扱いますが、このワークスペースの Nav2 設定は `/scan` を前提に書かれています。  
そのため、Livox の生データは次の 2 段階で変換されます。

- Livox 独自形式 -> `PointCloud2`
- `PointCloud2` -> `LaserScan`

## どこで使われているか

- `ares_nav2/controller_bringup.launch.py`

## 初見の人が最初に確認するとよいファイル

- `launch_ROS2/msg_MID360_launch.py`
- `config` ディレクトリの LiDAR 設定
- 既存の `README.md`

## 補足

- このパッケージ自体は外部由来ですが、ワークスペース全体の障害物検出ラインの起点なので役割は大きいです。
- もし `livox_ros_driver2` 側を最初から `PointCloud2` 出力に切り替えるなら、`livox_to_pointcloud2` は不要になる可能性があります。
