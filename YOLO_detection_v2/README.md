# YOLO_detection_v2

## 概要

`YOLO_detection_v2` は、RealSense の RGB/Depth を使って物体検出を行うパッケージです。  
このワークスペースでは主に 2 つの用途があります。

- 検出結果を TF にして Nav2 の追跡目標にする
- 検出結果から直接 `/cmd_vel` を作る簡易追従に使う

## 主な実行ファイル

### `publish_YOLOtf`

実体は `YOLO_detection_v2/YOLOtf.py` です。

役割:

- `camera/color/image_raw`
- `camera/depth/image_raw`
- `camera/color/camera_info`
- `realsense_info`

を使って物体の 3D 位置を推定し、TF を publish します。

主な出力:

- `detection`
- TF child frame
  - `mallet`
  - `hammer`
  - `bottle`

さらに現在は `/yolo/enabled` を購読し、必要なときだけ YOLO を有効化する構成です。

### `publish_direction`

実体は `YOLO_detection_v2/make_direction.py` です。

役割:

- RGB 画像だけを使って物体の左右位置を判定
- `direction` や `/cmd_vel` を publish
- 簡易的な視覚追従に使う

## Launch

### `launch/make_direction.launch.py`

- `publish_direction` を起動します

### `publish_YOLOtf` の起動場所

`publish_YOLOtf` 専用の launch はこのパッケージ内には置かず、  
現在は `ares_nav2/launch/controller_bringup.launch.py` から起動する構成です。

## フォルダ構成

```text
YOLO_detection_v2/
├── YOLO_detection_v2/
│   ├── YOLOtf.py
│   ├── make_direction.py
│   ├── subscribe_realsense.py
│   ├── subscribe_depth.py
│   └── subscribe_YOLO.py
├── launch/
│   └── make_direction.launch.py
├── resource/
│   └── YOLO_detection_v2
├── test/
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
├── train260205s_best.pt
├── setup.py
├── setup.cfg
├── package.xml
└── LICENSE
```

## モデルファイル

学習済みモデルは次の場所を前提にしています。

- `YOLO_detection_v2/train260205s_best.pt`

実装上は、`ros2 run` で `install` 側から起動された場合でも、

- `urc_ws/src/YOLO_detection_v2/train260205s_best.pt`

を優先して探すようにしています。

## このワークスペースでの使われ方

### Nav2 と組み合わせる流れ

1. `gps_waypoint_follower_node` が `/yolo/enabled` を `true` にする
2. 同時に `/yolo/target_frame` に `hammer` などの対象名を publish する
3. `publish_YOLOtf` が対象物を検出して TF を publish する
4. `yolo_tf_nav2_goal_node` が TF を Nav2 goal に変換する
5. 目標到達後に `gps_waypoint_follower_node` が `/yolo/enabled` を `false` に戻す

### 簡易追従として使う流れ

- `publish_direction` を単独起動して `/cmd_vel` を直接使います

## 主要ファイル

- `YOLO_detection_v2/YOLOtf.py`
  TF ベースの本番系実装です。
- `YOLO_detection_v2/make_direction.py`
  `/cmd_vel` を直接出す簡易追従です。
- `train260205s_best.pt`
  学習済み重みです。

## 最近の編集履歴

### 2026-04-15

- `publish_YOLOtf` を `ares_nav2/controller_bringup.launch.py` から起動する運用に変更
- `/yolo/enabled` による on-demand 起動に対応
- 無効時は画像処理を止め、モデルを解放するように変更
- モデルパスを `src/YOLO_detection_v2/train260205s_best.pt` 優先で解決するように変更
- TF 名 `hammer` を waypoint 側と合わせて運用する前提に整理

## 注意点

- `publish_YOLOtf` はノード起動直後から常に推論するわけではありません。
- `/yolo/enabled` が `false` の間は待機状態です。
- `cv_bridge` を使うため、実行環境の NumPy は ROS Humble と互換のある構成を使ってください。
  問題が出る場合は `numpy<2` を使うのが安全です。
