# rviz_nav2

## 概要

`rviz_nav2` は、`/aruco/id` の状態を RViz 上で見やすく表示するためのカスタムパネルプラグインです。  
ArUco が見えているかどうかと、その ID をオペレータが即座に確認できるようにするのが役割です。

## このパッケージが担当すること

- `/aruco/id` の購読
- 「Detected / None」の状態表示
- 受信中の ArUco ID の表示

## プラグインの内容

### `ArucoIdDisplay`

Subscribe:

- `/aruco/id`

表示内容:

- メッセージが一定時間内に届いていれば `Detected`
- 届いていなければ `None`
- 直近の ID 値を表示

## このワークスペースの中での位置づけ

ナビゲーションや制御そのものには直接関与しません。  
ただし、ArUco ベース接近のデバッグ時には非常に分かりやすい補助表示です。

## どこで使われているか

- 主要 launch から自動では読み込まれていません
- RViz プロファイルや手動追加で使う想定です

## 初見の人が最初に確認するとよいファイル

- `src/aruco_id_display.cpp`
- `include/rviz_nav2/aruco_id_display.hpp`
- `plugins_description.xml`

## 補足

- `aruco_opencv` が `/aruco/id` を出していることが前提です。
- 「検出しているかどうか」だけを素早く見たいときに向いており、詳細な pose 可視化は別の RViz Display の方が適しています。
