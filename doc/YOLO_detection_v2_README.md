# YOLO_detection_v2

## 概要

`YOLO_detection_v2` は、RealSense の RGB/Depth を使って物体検出を行うパッケージです。  
このワークスペースでは大きく 2 つの使い方があります。

- 物体の位置を TF として publish し、Nav2 の目標点にする
- 検出結果から直接 `/cmd_vel` を作る簡易追従ノードとして使う

## このパッケージが担当すること

- YOLO モデルの読み込み
- カメラ画像からの物体検出
- Depth と内部パラメータを使った実座標推定
- 物体位置の TF 化
- 簡易な左右判断による追従制御

## 主な実行ファイル

### `publish_YOLOtf`

実体は `YOLO_detection_v2/YOLOtf.py` です。  
RGB、Depth、CameraInfo を組み合わせて物体の 3D 位置を推定し、TF を publish します。

主な入力:

- `camera/color/image_raw`
- `camera/depth/image_raw`
- `camera/color/camera_info`
- `realsense_info`

主な出力:

- `detection`
- TF child frame
  - `mallet`
  - `hammer`
  - `bottle`

`ares_nav2/yolo_tf_nav2_goal_node` は、この TF を Nav2 ゴールへ変換して使います。

### `publish_direction`

実体は `make_direction.py` です。  
RGB 画像だけを使って物体の左右と大きさを見て、簡易的な追従用 `direction` と `/cmd_vel` を作ります。

主な出力:

- `direction`
- `/cmd_vel`
- `uart_command`

このノードは Nav2 を使わず、視野中央に物体が来るように単純に車体を動かす用途です。

## このワークスペースでの使われ方

### Nav2 と組み合わせる使い方

1. waypoint に `yolo: mallet` のような条件を書く
2. `gps_waypoint_follower_node` が `/yolo/target_frame` に `mallet` を publish
3. `publish_YOLOtf` が `mallet` TF を配信
4. `yolo_tf_nav2_goal_node` がその TF を Nav2 ゴールへ変換

### 単独で簡易追従する使い方

- `publish_direction` を起動して、視覚追従だけで動かす

## launch

- `launch/make_direction.launch.py`
  - `publish_direction` を起動

注意:

- TF ベースの `publish_YOLOtf` 用 launch は現状用意されていません。
- Nav2 と組み合わせて使う場合は、`ros2 run YOLO_detection_v2 publish_YOLOtf` のように別途起動する前提です。

## モデルファイル

コードは次のファイルを前提にしています。

- `train260205s_best.pt`

つまり、このパッケージはソースだけでなく学習済みモデルの配置も前提です。

## どこで使われているか

- `ares_nav2`
  - `yolo_tf_nav2_goal_node` と連携
- `realsense_from_lib`
  - 入力画像を供給
- `uart_control`
  - `publish_direction` が停止コマンドを送るときに連携

## 初見の人が最初に確認するとよいファイル

- `YOLO_detection_v2/YOLOtf.py`
- `YOLO_detection_v2/make_direction.py`
- `launch/make_direction.launch.py`

## 補足

- TF ベースの構成と `/cmd_vel` 直出し構成が同居しているため、用途によって見るファイルが異なります。
- 実運用では「Nav2 に統合したいのか」「視覚追従だけしたいのか」を先に決めると理解しやすいです。
