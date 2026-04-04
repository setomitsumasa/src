# rviz_2d_overlay_plugins

## 概要

`rviz_2d_overlay_plugins` は、RViz の 3D 画面の上に 2D テキストやゲージを重ねて表示するためのプラグイン集です。  
このワークスペースでは主要 launch からは使われていませんが、運用用 UI を RViz 上に載せたいときに便利な補助パッケージです。

## このパッケージが担当すること

- テキストオーバーレイ表示
- 円形ゲージ表示
- `std_msgs/String` を `OverlayText` に変換する補助ノードの提供

## 主な構成要素

### `OverlayTextDisplay`

`rviz_2d_overlay_msgs/msg/OverlayText` を RViz 上に描画します。

向いている用途:

- ミッション状態表示
- 認識結果の文字表示
- バッテリやモードの表示

### `PieChartDisplay`

`std_msgs/msg/Float32` を円形ゲージとして表示します。

向いている用途:

- 進捗率
- センサ値
- 信頼度や残量の簡易表示

### `string_to_overlay_text`

`std_msgs/String` を `OverlayText` に変換する小さなノードです。  
既存の文字列トピックを手早く RViz 表示したいときに便利です。

## このワークスペースの中での位置づけ

現状のローバー制御や Nav2 の本流には入っていません。  
ただし、デバッグ時の状態表示やオペレータ向け UI を強化したい場合に、そのまま流用しやすいパッケージです。

## どこで使われているか

- 現在の主要 launch からは未使用
- `rviz_2d_overlay_msgs` とセットで利用

## 初見の人が最初に確認するとよいファイル

- `src/overlay_text_display.cpp`
- `src/pie_chart_display.cpp`
- `src/string_to_overlay_text.cpp`
- 既存の `README.md`

## 補足

- ArUco や YOLO の状態を operator UI として重ねたい場合、このパッケージは有力な候補です。
- すでに `rviz_nav2` という専用パネルもあるので、用途に応じて「独自パネル」と「オーバーレイ表示」を使い分けるとよいです。
