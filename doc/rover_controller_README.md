# rover_controller

## 概要

`rover_controller` は、ROS 標準の速度指令 `/cmd_vel` を、ローバー MCU が理解できる `uart_command` に変換するパッケージです。  
Nav2 や簡易追従ノードは `/cmd_vel` を出すだけでよく、車体固有の角度 ID や速度 ID への変換はこのパッケージが引き受けます。

## このパッケージが担当すること

- `/cmd_vel` の受信
- 操舵角と速度の計算
- `uart_command` へのパック

## 実行ファイル

### `rover_controller_node`

Subscribe:

- `/cmd_vel`

Publish:

- `uart_command`

内部でやっていること:

- `linear.x` を前進速度として扱う
- `angular.z` を進行方向角へ変換する
- 右旋回と左旋回で異なる CAN ID を使い分ける
- 速度値には上限制限をかける

出力のデータ形式:

```text
Int16MultiArray.data = [angle_can_id, angle_value, speed_can_id, speed_value]
```

## 主な launch

- `launch/rover_controller.launch.py`

## このワークスペースの中での位置づけ

`rover_controller` は、ソフトウェア側の共通運動指令と、実機ローバー固有の駆動プロトコルの変換層です。

主な利用元:

- Nav2 controller server が出す `/cmd_vel`
- `YOLO_detection_v2/make_direction.py` が出す `/cmd_vel`

そのため、このパッケージがあることで上位ロジックは「どう動きたいか」だけを考えればよく、「MCU に何番の ID をどう送るか」は意識しなくて済みます。

## どこで使われているか

- `ares_nav2/controller_bringup.launch.py`
  - 統合 bringup の一部として起動
- `uart_control/serial_publiasher`
  - `uart_command` の最終送信先として連携

## 初見の人が最初に確認するとよいファイル

- `src/rover_controller_node.cpp`
- `launch/rover_controller.launch.py`

## 補足

- `config/rover_controller.yaml` は現状ほぼ例示用で、ソース側では積極的には使われていません。
- `timer_period_` はコメントでは 50Hz とありますが、コード上の値は `0.01` 秒なので実際には 100Hz 相当です。仕様確認時はコメントよりコードを優先してください。
