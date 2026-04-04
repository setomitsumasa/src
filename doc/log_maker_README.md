# log_maker

## 概要

`log_maker` は、走行中の各種トピックを CSV に記録し、通過チェックポイントや判定値を残すための補助パッケージです。  
ナビゲーション本体ではなく、実験・競技・評価向けのログ収集に寄った役割を持っています。

## このパッケージが担当すること

- GPS、IMU、ArUco、`cmd_vel` の記録
- checkpoint 通過判定
- CSV 追記

## 実行ファイル

### `TRC_log_maker`

主な購読トピック:

- `gps/fix`
- `imu/data`
- `/aruco/id`
- `cmd_vel`
- `imu/yaw`

出力:

- `TRC_log_test.csv`

加えて、以下の CSV を前提にしています。

- `arucoid_gps_B.csv`
- `checkpoint_number.csv`

## このワークスペースの中での位置づけ

メインの自律走行そのものには直接参加しません。  
どちらかというと「あとで結果を解析するための記録係」です。

## 現状の注意点

コードを見ると、`/aruco/id` を `visualization_msgs/MarkerArray` として受け取る前提になっています。  
一方、現在の `aruco_opencv` は `/aruco/id` を `std_msgs/Float32` で publish しています。

そのため、このパッケージを今の主要構成でそのまま動かすには、入力トピック設計の見直しまたはコード修正が必要です。

## どこで使われているか

- 現在の主要 launch からは使われていません

## 初見の人が最初に確認するとよいファイル

- `log_maker/TRC_log_maker.py`
- `setup.py`

## 補足

- 競技や実験の後解析用に残してある補助ツール、と理解すると位置づけが分かりやすいです。
- 実運用に組み込む前には、入力トピック型と参照 CSV の所在を確認してください。
