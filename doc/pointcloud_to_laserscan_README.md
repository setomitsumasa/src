# pointcloud_to_laserscan

## 概要

`pointcloud_to_laserscan` は、3D 点群を 2D の `LaserScan` に投影する外部由来パッケージです。  
このワークスペースでは、Livox LiDAR の 3D 点群を Nav2 の costmap が扱える `/scan` に変換するために使われています。

注意:

- ディレクトリ名は `pointcloud_to_laserscan-humble`
- ROS パッケージ名は `pointcloud_to_laserscan`

## このパッケージが担当すること

- `PointCloud2` を `LaserScan` に変換
- 高さ制限や距離制限で必要な点だけを抽出
- TF を使って指定フレームに合わせたスキャンを作成

## このワークスペースでの使われ方

`launch/sample_pointcloud_to_laserscan_launch.py` では、以下の設定で使われています。

- Input: `/converted_pointcloud2`
- Output: `/scan`
- `target_frame`: `livox_frame`
- `min_height`: `0.0`
- `max_height`: `1.0`
- `range_min`: `2.0`
- `range_max`: `8.0`

つまり、LiDAR 点群全体をそのまま Nav2 に渡すのではなく、「地面付近の一定高さ範囲だけを 2D スキャンに落とす」形です。

## なぜ必要か

このワークスペースの `ares_nav2/config/nav2_no_map_params.yaml` では、ローカル costmap とグローバル costmap の両方が `/scan` を観測入力として設定されています。  
そのため、Livox 点群を Nav2 が使うには、このパッケージが不可欠です。

## どこで使われているか

- `ares_nav2/navigation_sim.launch.py`

## 初見の人が最初に確認するとよいファイル

- `pointcloud_to_laserscan-humble/launch/sample_pointcloud_to_laserscan_launch.py`
- `pointcloud_to_laserscan-humble/README.md`

## 補足

- このワークスペースでは `/scan` は LiDAR 由来であり、2D LiDAR 実機が直接つながっているわけではありません。
- `target_frame` が `livox_frame` なので、`ares_sensor/sensor_tf_node` が `base_link -> livox_frame` を出していることも重要です。
