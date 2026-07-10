# CLAUDE.md — ARES PROJECT 自律班 / ERC GPS-free 自律走行システム

> このファイルは、実装エージェント（Claude Code）が **最初に全文を読む** ための統合コンテキストです。
> 設計・調査・レビューは「設計チャット」で行い、実装はここを唯一の起点として進めます。
> 迷ったら **推測で埋めず、`<要確認>` / `<要記入>` を人間（ユーザ）に質問** してください。

---

## 0. このファイルの使い方（Claude Code への最初の指示）

1. まずこのファイルを最後まで読む。
2. 次にリポジトリ内の `doc/*.md`（後述）を読み、現行システムの実体を把握する。
3. `<要記入>` / `<要確認>` の項目は **勝手に値を作らない**。着手前にユーザへ確認する。
4. 変更は小さく・ビルド可能な単位で進め、各ステップで `colcon build` とテスト（rosbag / 実機）を回す。
5. **不変条件＝URC を回帰させない**。ERC の頭脳は既存（`ares_nav2` 等）を継承せず、同一ワークスペース内の **新規 ERC パッケージ** で設計する（§1.4）。
6. 学び（ビルド手順・ハマりどころ）はこのファイルまたは `doc/` に追記して蓄積する。

---

## 1. エージェント運用ルール（最重要・厳守）

### 1.1 体制と役割（三分割）
- **設計・調査・レビュー**＝設計チャット（人間＋設計用アシスタント）。アーキ確定はここ。
- **実装・実験・監査**＝Claude Code（このファイルを起点に実装）。
- **意思決定・承認**＝ユーザ。アーキ確定・共有リポジトリへの反映は **ユーザ承認後**。

### 1.2 ハードルール
- **捏造しない**：ルール数値・部品仕様・座標・外部パラメータ・URL を作らない。未確定は `<要確認>` と明示。
- **機構で語る**：入力→変換→出力／トピック／TF／式で説明できる形にする。抽象語だけの主張はしない。
- **層で扱いを変える**：共有 I/O 層（ドライバ/UART 等）は再利用しフォークしない。ERC の頭脳（localization/costmap/Nav2 config/ミッション統括）は **新規 ERC パッケージでゼロ設計** してよく、既存 `ares_nav2` を改変しない（§1.4）。コード様式（C++/Python 混在）には合わせる。
- **単一の権威ある TF**：`map → odom → base_link` の publisher を二重化しない（§6）。
- **承認ゲート**：大規模リファクタ、共有リポジトリへの反映、破壊的操作（削除・履歴改変）は着手前に確認。

### 1.3 git 安全運用（初心者運用・最重要）
> 背景：ユーザは現状 `push` 中心の操作しか慣れておらず、共有リポジトリ（他班＝電装等も使用）へ直 push する事故リスクがある。以下を厳守。

- リモート構成：
  - `origin` = **個人リポジトリ/フォーク** `<要記入: 個人リポジトリURL>`
  - `upstream` = **ARES 大元リポジトリ（共有）** `<要記入: 大元リポジトリURL>`
- **共有（upstream / main）へは直 push しない。** 必ずブランチ＋Pull Request。
- 作業ブランチ命名：`feat/slam-fastlio`, `feat/erc-localization`, `fix/...` など。
- push は原則 `origin` の作業ブランチへ。共有へは PR 経由（`gh pr create` を利用可）。
- **push / PR 作成の前に必ずユーザへ一言確認**（何を・どのリモートへ）。
- 権限プロンプトを面倒がって全許可にしない。こまめな commit で、失敗してもブランチごと破棄できる状態を保つ。
- コミットメッセージは命令形・簡潔。何を・なぜ を書く。

### 1.4 URC 保全と ERC の分離方針（回帰防止＋設計自由度）
**不変条件（守るべき結果。手段ではない）**：URC 構成（GNSS ベース）が動く状態を回帰させない。ERC 実装中も URC はいつでも起動できること。

**分離方針＝同一ワークスペース＋新規 ERC パッケージ**（この ROS2 ワークスペースに ERC 用パッケージを追加する。`ares_nav2` 等の URC 頭脳は触らない。別リポジトリ化はしない）:
- ERC の頭脳（localization=FAST-LIO2、costmap、Nav2 config、ArUco datum 補正、Cartesian ミッション統括）は **新規 ERC パッケージでゼロから設計してよい**。既存 `ares_nav2` を継承・改変しない＝**設計は既存に縛られない**。
- **共有 I/O 層はフォークせず再利用**：`uart_control`, `rover_controller`, `livox_ros_driver2`, `realsense_from_lib`, `aruco_opencv` は機体 I/O（設計要素ではない）で、大元リポジトリ＝電装班とも共有。ERC パッケージはこれらに **依存** するだけで作り直さない（再利用は制約ではない）。
- 起動は **launch 1本で切替**：URC 系 launch と ERC 系 launch を別に持ち、`ros2 launch <erc_pkg> <bringup>.launch.py` で起動集合を切り替える。デバッグ分離は「パッケージ/config の分離」で担保され、リポジトリ分割は不要。

| 層 | 例 | 扱い |
|---|---|---|
| 共有 I/O（機体） | uart_control, rover_controller, livox_ros_driver2, realsense_from_lib, aruco_opencv | 再利用（フォークしない）。電装班と共有 |
| ERC 頭脳（自由設計） | 新規 localization(FAST-LIO2), costmap, Nav2 config, ArUco datum 補正, ミッション統括 | **新規 ERC パッケージ**でゼロ設計 |
| URC 専用 | robot_localization(GPS), gps_waypoint_follower, YOLO 接近, URC 用 Nav2 config | ERC では起動しない・改変しない |

### 1.5 実装の進め方
- 1 変更 = 1 目的。大きな変更は分割し、各段階で `colcon build --symlink-install` が通ることを確認。
- センサ・SLAM は **rosbag を記録してオフライン再現** できる形で検証（実機依存を減らす）。
- 変更ごとに「確認方法（どのトピック/TF/RViz をどう見れば正しいと分かるか）」を残す。

---

## 2. プロジェクト概要と最終目標

- **チーム**：ARES PROJECT（日本初の学生火星探査ローバー開発チーム。東北大・慶應中心）。ユーザは **自律班**。
- **最終目標**：**European Rover Challenge (ERC) 2026**（2026/9/4–6、ポーランド・クラクフ AGH 大学、現地フォーミュラ）での Navigation: Traverse を、**GPS を使わない SLAM ベースの自律走行**で高得点する。設計から実装まで。
- **なぜ GPS-free か**：ERC の得点構造上、GNSS 不使用かつ管制での映像フィードバック不使用で満点になるため（§3.1、根拠は ERC 2026 Rules Rev.2）。
- **新規性の優先度は低い**。競技ルール・現行センサ構成に対して「確実に成立し・信頼でき・回帰しない」実装が最優先。

---

## 3. ERC 競技ルール（設計制約）— 出典：ERC 2026 Rules Rev.2（プロジェクト添付）

### 3.1 Navigation: Traverse 得点構造（Rev.2 §7.3.2.1, Table 3）
得点は独立 2 軸：**Traverse 点（50/80/100%）** と **Autonomy 点（0 or 100%）**。

| 管制での映像feed | GNSS | Traverse点 | 自律実装で得られるAutonomy点 |
|---|---|---|---|
| 使用 | – | 50% | 実装で100% |
| 使用しない | 使用 | 80% | 実装で100% |
| 使用しない | **不使用** | **100%** | 実装で100% |

- 自律は **方式を問わず「有=100% / 無=0%」**。
- 「映像feed使用 / GNSS使用」とは **管制局の画面でそれを見て操作すること**。機体上で完結し管制へ送っていなければ自律満点。
- ⇒ **設計目標＝GNSSなし・管制映像なし・機体上完結の自律**（Traverse 100% + Autonomy 100%）。
- Traverse は 300 点満点（Navigation Task 600 点のうち）。

### 3.2 タスク進行と時間
- 準備 15 分（**実機フルアクセス・走行可・ソフトアップロード可**）→ 実行 20 分。
- 審判が定めた **4 点を任意順で到達**し、**5 点目（finish）を最後**に、発進点へ復帰。
- 移動が必要な場合、直前の到達済み waypoint か発進点へ戻せるが手動介入ペナルティ。
- 技術レポートに全センサ・使用モード・運用方法の記載が必要（審判が事前相談可）。

### 3.3 座標系・データム・配布物（Q&A 回答で確認済み）
- 座標系：**直交 X, Y（Z も提供）**。
- 原点 (0,0)：**発進地点の一つ**。`<要確認: どの発進点が原点か・軸/方位基準>`（8月確定）。
- 配布：**PDF**。全ランドマーク・waypoint（Traverse 用）・Deep-Sampling 位置の座標が提供される。
- 詳細（Mars Yard 3D モデル＋ドローン写真＋全点座標）は **Update Report #3（8月）** で配布。

### 3.4 ArUco / 障害物 / YOLO
- **ArUco マーカーを航法に使ってよい**（§7.3.2.1.4 g）。ID↔座標対応が配布される見込み（`<要確認: 配布PDFに各ランドマークのArUco IDフィールドが必ず付くか>`）。
- 3D モデルに **写らない地物（岩・溝・バンプ）が多い** と明記 → 実地の障害物回避は必須（モデル頼み不可）。
- **ERC Traverse に物体拾いは無い** → **YOLO 物体検出は Traverse では不要**（計算予算を SLAM に集中）。ArUco 検知は必要（§6.5）。
- 発進時、初期位置・向きは指定地点集合から抽選（向きは限定）。⇒ **初期姿勢は概ね既知**（定位の初期化に使える）。

### 3.5 8月まで確定できない項目（勝手に埋めない）
- 主催 3D モデルの形式・密度・視点（点群かメッシュか／地上か上空か）→ 自作マップへ寄せるか決まる。
- 配布座標のフレーム規約（原点・軸・方位）。
- 準備エリアが本番コースをどこまで覆うか。
- 各ランドマークの ArUco ID 明示の有無。
> これらは実データ到着時にピンで留める。設計は「これらに依存しすぎない」ように作る。

---

## 4. ハードウェア / ソフトウェア環境

- **LiDAR**：Livox **Mid-360**（360°×59° FoV、~20万点/s、内蔵 IMU=ICM-40609、静的 IP、100BASE-TX）。**mini-PC の LAN ポートへ LAN ケーブル直結**。
  - 既定 IP は `192.168.1.1XX`（XX=シリアル下2桁）。ホスト側 NIC を同セグメントに設定要。`<要確認: 実機のIP・ホストNIC設定>`
- **カメラ**：Intel RealSense **D435i**（RGB-D。ArUco 検出＋距離推定に使用）。
- **機体 IMU / ヘディング**：9 軸センサ（地磁気ヘディング＋IMU、UART 経由）。URC では自己位置に使用。
- **機上計算機**：GMKtec M8 mini-PC（AMD Ryzen 5 PRO 6650H, DDR5 16GB, 512GB）。**GPU 非依存前提**（統合 GPU のみ）。
- **OS / スタック**：Ubuntu **22.04**、ROS 2 **Humble**、C++ / Python、Nav2 / behavior tree、シミュレーション＝Unity。
- **AI 実装ツール**：Claude Code（Claude Pro に含まれる）。VSCode 拡張として使用予定。

---

## 5. 現行システム（URC 構成）＝ 実装の出発点

現行ワークスペースは `ares_nav2` を中核に、UART センサ入力・RealSense/Livox/ArUco/YOLO 認識・`cmd_vel`→MCU 変換が組み合わさっている。**Livox は現状 2D `/scan` に落として障害物回避にしか使っておらず、自己位置は `robot_localization`（IMU＋GPS）が担い、`map` は GPS 基準**。

### 5.1 主要パッケージ
| パッケージ | 役割 |
|---|---|
| `ares_nav2` | Nav2 起動、GPS waypoint 管理、ArUco/YOLO 接近フェーズ制御（ミッションオーケストレータ） |
| `ares_sensor` | UART 生データ → IMU/GPS/TF 変換 |
| `uart_control` | UART 受信 / 送信ブリッジ |
| `rover_controller` | `/cmd_vel` → `uart_command` 変換 |
| `realsense_from_lib` (/`_libC`) | RealSense RGB/Depth/CameraInfo 配信 |
| `aruco_opencv` (+`_msgs`) | ArUco 検出、`/aruco/id` と `aruco_marker` TF |
| `YOLO_detection_v2` | YOLO 物体検出→TF・簡易追従（**ERCでは不使用**） |
| `livox_ros_driver2` | Livox Mid-360 ドライバ |
| `livox_to_pointcloud2` | Livox 独自形式 → `PointCloud2` |
| `pointcloud_to_laserscan` | 3D 点群 → `/scan`（Nav2 costmap 用） |
| `rviz_*`, `log_maker` | 可視化・ログ補助 |

### 5.2 データフロー要点
- UART → `imu_node`(`/imu/data`) / `gps_node`(`/gps/fix`)。
- `robot_localization`：`navsat_transform` + `ekf_odom`(`/odometry/local`) + `ekf_map`(`/odometry/global`)。`map` は GPS 基準。
- `gps_waypoint_follower`：`config/waypoints.yaml`（緯度経度）を読み、最初の `gps/fix` を基準点にローカル平面へ変換し Nav2 へ。到達後 ArUco / YOLO 接近フェーズへ分岐。
- Livox：`/livox/lidar` → `livox_to_pointcloud2`(`/converted_pointcloud2`) → `pointcloud_to_laserscan`(`/scan`) → Nav2 costmap。
- 制御：Nav2 `/cmd_vel` → `rover_controller` → `/uart_command` → `serial_publisher` → MCU。

### 5.3 TF の現状（重要）
- `sensor_tf_node` が `map → odom` と `base_link` 配下の静的 TF を publish。
- `odom → base_link` は `robot_localization` が publish。
- ⇒ ERC 化では **この `map`/`odom` の権威を SLAM に移す**（§6）。

> 詳細は `doc/node_topic_graph_README.md`, `doc/ares_nav2_README.md`, `doc/ares_sensor_README.md` を参照。

---

## 6. 目標システム（ERC GPS-free）確定設計

### 6.1 全体アーキテクチャ
```
[事前] 主催3Dモデル/座標  → datum(原点・向き)・waypointゴール・粗い事前地図の定義
[準備15分] Mid-360で自作LiDAR点群マップ(FAST-LIO2 mapping) → .pcd 保存
[本番] Mid-360+IMU → LiDAR定位(既知/自作マップ上, FAST_LIO_LOCALIZATION 相当) → map→odom→base
        ArUco(既知座標) → aruco_map_anchor → datum↔map をゲート補正
        live 3D点群 → costmap(障害物回避)   ※当面 pointcloud_to_laserscan 流用可
        Nav2(共有framework) + 新規Cartesian waypoint follower → 4点+finish 到達・復帰
```
> 基本方針：**Nav2 フレームワークと共有 I/O は再利用し、ERC の定位・config・統括は新規 ERC パッケージで供給する**（既存 `ares_nav2` / `robot_localization` は改変しない）。Livox を「2D scan 専用」から「3D LIO の主センサ」へ格上げ。

### 6.2 SLAM 選定と根拠
- **バックボーン＝ FAST-LIO2（ROS2 版）**。tightly-coupled iterated EKF、特徴抽出不要、ikd-tree で高速、CPU のみで軽量、Livox CustomMsg（点ごと timestamp）で motion undistortion。Mid-360 内蔵 IMU と相性が良い。
- **大域整合＝事前マップ＋定位（§6.3）＋ ArUco 絶対アンカー（§6.4）**。
- **代替と却下理由**（naive gluing を避けるため単一バックボーンに統一）：
  - Point-LIO：高レート/機敏動作向け。低速ローバーには過剰。
  - LIO-SAM：GTSAM ループ閉じは魅力だが回転式 LiDAR 前提で Livox 適応の手間。
  - GLIM / hdl_graph_slam：グラフ最適化＋ループ閉じ内蔵だが設定が重い。将来の選択肢。
  - KISS-ICP：LiDAR のみ・堅牢。**フォールバック基準線**として保持。
  - **TagSLAM：不採用**。ROS2 版は Rolling/Jazzy 以降で **Humble 非対応**、AprilTag 前提（ERC は ArUco）、屋外・マーカー疎でカメラ単独 fiducial は脆い。「fiducial を factor として使う」発想は §6.4 で活かす。
- **禁止事項**：2 つの SLAM を並走させ出力を混ぜる／`map→odom` を二重に publish する等の継ぎ接ぎ。

### 6.3 事前マップ → 定位
- 準備 15 分（または事前訪問が許されれば事前）に Mid-360 で走り、FAST-LIO2 の **mapping** で点群地図を作り **.pcd 保存**（`pcd_save_en` + `/map_save` サービス）。
- 本番は **既知マップ上でローカライズ**（FAST_LIO_LOCALIZATION 相当）してドリフト蓄積を断つ。
- 主催 3D モデルは **datum・waypoint・粗地図** の定義に使い、**照合用の定位マップは Mid-360 由来の点群**を基本にする（モデルが上空/メッシュだと地上 LiDAR と不整合になり得るため）。`<要確認: 8月配布データの形式>`

### 6.4 ArUco 大域アンカー（補正の式と安全弁）
定位が `map` 基準姿勢 `T_map→base` を出し、RealSense が ID=k を検出すると `aruco_opencv` が `T_cam→marker` を出す。外部パラメータ `T_base→cam` を用い、推定上のマーカー位置は
```
p_marker^map(推定) = T_map→base · T_base→cam · p_marker^cam
```
既知真値 `p_k^datum`（配布表）との差
```
Δ = p_k^datum − p_marker^map(推定)
```
を「蓄積ドリフト＋フレーム不整合」とみなす。

- **採用（A）**：`datum→map`（＝`map→odom` の大域補正）という **1 本の剛体変換をゆっくり更新**。LiDAR 定位は局所の滑らかな姿勢の権威として残し、ArUco は疎な大域アンカーとして **フレームを1つだけ**直す。複数同時検出なら SE(2)/Procrustes フィット、単一なら並進主体＋（近距離・正面時のみ）弱く方位。
- **不採用（B）**：ポーズグラフに landmark factor として厳密統合。原理的だが重く、5 点規模には過剰。まず (A)。

安全弁（必ず実装）：
- 単眼 ArUco の **平面ターゲット姿勢曖昧性**（遠距離・浅角で悪化）→ **並進補正を主**、方位は近く正面時のみ。D435i の **depth で距離を安定化**。
- **姿勢を不連続にスナップさせない**（Nav2 costmap/経路が壊れる）→ `datum→map` をローパス更新 or 低レート絶対観測として滑らかに。
- **妥当性ゲート**：既知 ID のみ／再投影誤差・見かけサイズ・最大距離で足切り。誤対応は大誤差を注入。
- **外部パラメータ校正**（`base↔Mid-360`, `base↔D435i`）が補正精度の上限。校正手順を早期に用意。`<要確認: 実測 extrinsics>`

### 6.5 ArUco 検知ノードの起動設計
YOLO を外して余裕があり、OpenCV の ArUco 検出は軽いので **常時オン（スロットル）** が素直。関心を分離：
- `aruco_detector`（常時オン・5–10Hz throttle）：D435i RGB を購読、検出 ID と `T_cam→marker` を publish。
- `aruco_map_anchor`（ゲート付き・補正専任）：既知 ID かつ品質ゲート通過分だけ受け、§6.4(A) の `datum→map` 更新。
- 現行の `/aruco/enabled`（waypoint 接近時のみ有効化）ゲートは **ERC では外して常時検出** に。最終接近フェーズが要る場面は既存接近ノードを必要時のみ追加。
- lifecycle ノード化も可能だが、5 点規模なら「常時検出＋補正側ゲート」の単純構成が壊れにくく、後輩にも読める。

### 6.6 再利用 / 新規 / 不使用 一覧（同一ワークスペース＋新規 ERC パッケージ）
- **再利用（フォークしない）**：Nav2 フレームワーク、共有 I/O（`uart_control`/`rover_controller`/各ドライバ）、ArUco 検出（`aruco_opencv`）。
- **新規 ERC パッケージで実装**：FAST-LIO2 による定位（`map→odom→base` を単一権威で供給）、`aruco_map_anchor`（§6.5）、事前マップ保存/読込、ERC 用 costmap/Nav2 config、Cartesian waypoint follower、ERC bringup launch。
- **URC 側（触らない・ERC では起動しない）**：`robot_localization`(GPS)/`navsat_transform`、`gps_waypoint_follower`、YOLO 接近。

---

## 7. 機能要件（ERC Traverse）

1. GNSS を使わず、機体上で完結する自律走行で 4 waypoint＋finish を到達し発進点へ復帰する。
2. Mid-360＋IMU による 6-DoF 定位を `map→odom→base` として単一権威で提供する。
3. 主催配布の直交座標 (X,Y,Z) を waypoint ゴールとして扱える（緯度経度依存を排除）。
4. 既知座標 ArUco を検出し、`datum↔map` を滑らかに補正してドリフトを抑える。
5. live 3D 点群から costmap を構成し、モデルに無い障害物（岩・溝・バンプ）を回避する。
6. 準備 15 分内に事前マップ生成 or マップ読込と初期化ができる。
7. 管制へ映像/GNSS を送らずに走行できる（自律満点条件）。ただし **操作者支援用の位置・状態の可視化**は機体上処理で提供してよい（ルール上 blind teleop は位置推定のみ可）。
8. URC 構成を壊さず、ERC 構成を並行で起動切替できる。

---

## 8. 実装フェーズ・ロードマップ

- **Phase 0（合宿・最優先／後悔しない範囲）**：`livox_ros_driver2` + FAST-LIO2 **mapping** を立て、3D スキャン/.pcd 保存まで。ROS2（node/topic/tf/rosbag）学習も兼ねる。
- **Phase 1**：定位モード（既知/自作マップ上でのローカライズ）を立て、初期化と収束を確認。
- **Phase 2**：`map→odom` の権威を（新規 ERC パッケージの）SLAM 定位に置き、Nav2 を GPS 無しで動かす。**新規 Cartesian waypoint follower** を用意（URC の `gps_waypoint_follower` は流用せず別実装）。
- **Phase 3**：`aruco_map_anchor`（§6.4/6.5）で大域補正。costmap を live 点群ベースに整理。屋外/砂地でのロバスト性。
- **Phase 4**：Traverse 全体（4点+finish）統合、Unity シム整合、回帰試験、運用手順書・操作者可視化。

---

## 9. ローバー自律走行 実装の一般手順（bring-up 順）

> SLAM/Nav2 は下流ほどエラーが増幅する。**下から順に、各段で rosbag 検証**しながら積み上げる。

1. **各センサ単体起動と健全性確認**：Mid-360 点群 `/livox/lidar` と IMU `/livox/imu`、D435i、機体 IMU のトピック・レートを確認。
2. **TF ツリー & 外部パラメータ校正**：`base_link ↔ 各センサ` を正しく。**SLAM は extrinsics に極めて敏感**。`extrinsic_T`/`extrinsic_R`（LiDAR の IMU body 系での姿勢）を設定。
3. **時刻同期**：Mid-360 と IMU の同期を確保。IMU 加速度の単位（g→m/s²）に注意（§11 の standard-unit 版ドライバ推奨）。
4. **Odometry/SLAM（FAST-LIO2 mapping）**：rosbag で map/odom 品質を確認（"Failed to find match for field 'time'" が出たら CustomMsg 未使用）。
5. **定位モード（prior map 上）**：収束・datum 整合を確認。
6. **大域アンカー（ArUco）**：補正が不連続を起こさないか、ゲートが効くか確認。
7. **costmap / 障害物回避**：live 点群から生成、モデルに無い地物を避けられるか。
8. **Nav2 プランニング**：直交 waypoint への到達。
9. **ミッション統括**：waypoint follower（改修）で 4点+finish・復帰。
10. **シム(Unity)＋実地検証・回帰試験・操作者可視化**。各段で URC スタックを壊していないか確認。

---

## 10. リポジトリ / git ターゲット

- 作業対象：現行 自律走行ワークスペース（`ares_nav2` 他、`doc/` に各パッケージ解説）。
- `origin`（push 先・個人）：`<要記入: 個人リポジトリURL>`
- `upstream`（共有・PR 先）：`<要記入: 大元リポジトリURL>`（他班＝電装等も使用。**直 push 禁止**）
- ブランチ：`feat/slam-fastlio` から開始。共有へは PR。
- push / PR は §1.3 に従い、実行前にユーザ確認。

---

## 11. ビルド・実行コマンド（要点。実行前に各リポジトリ README で最新を確認）

> 参考実装：`livox_ros_driver2`（Livox-SDK 公式）、`FAST_LIO_ROS2`（Ericsii、ROS2/Humble 対応）、`FAST_LIO_LOCALIZATION`（再定位）。**バージョン・ブランチは着手時に最新確認**（`<要確認>`）。

**Livox 前提**
- Livox-SDK2 を導入し、Mid-360 の IP / ホスト NIC を設定（既定 `192.168.1.1XX`）。
- ドライバは新しい `livox_ros_driver2`（Mid-360 対応）を使用。**旧 `livox_ros2_driver` は Mid-360 非対応**。
- FAST-LIO 用途では **CustomMsg（`msg_MID360_launch.py`）** を使う（点ごと timestamp が motion undistortion に必須）。
- IMU 加速度の単位問題（g→m/s²）を避けるため、standard-unit 対応版ドライバの利用を検討（例：Ericsii の `livox_ros_driver2` standard-unit ブランチ）。

**ビルド（例）**
```bash
# livox_ros_driver2 を先に build & source してから FAST-LIO を build
cd <ros2_ws>/src
git clone https://github.com/Ericsii/FAST_LIO_ROS2.git --recursive   # ブランチ ros2
cd ..
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

**mapping 実行（例）**
```bash
# 端末A: Livox ドライバ（Mid-360, CustomMsg）
ros2 launch livox_ros_driver2 msg_MID360_launch.py
# 端末B: FAST-LIO2（Mid-360 用 config を用意して指定）
ros2 launch fast_lio mapping.launch.py config_file:=<mid360.yaml を作成して指定>
```
- `config/*.yaml` で `lid_topic` / `imu_topic` / `extrinsic_T` / `extrinsic_R` を Mid-360 用に設定。extrinsics が既知なら `extrinsic_est_en: false`。
- 地図保存：`pcd_save.pcd_save_en: true` と `map_file_path` を設定し、`/map_save` サービスを呼ぶ。

**現行ワークスペースの launch（URC 系。ERC は同一ワークスペースの新規パッケージに別 bringup launch を用意する）**
- `ares_nav2/controller_bringup.launch.py`（センサ・UART・RealSense・ArUco・Livox・制御の統合 bringup）
- `ares_nav2/navigation_sim.launch.py`（`robot_localization`＋Nav2 本番系）
- `ares_nav2/main.launch.py`（waypoint follower＋ArUco/YOLO 接近）

---

## 12. 参照すべきリポジトリ内ドキュメント（読む順）
1. `doc/node_topic_graph_README.md`（ノード/トピック/TF の全体像）
2. `doc/ares_nav2_README.md`（ミッションオーケストレータ）
3. `doc/ares_sensor_README.md`（UART→IMU/GPS/TF）
4. `doc/uart_control_README.md`, `doc/rover_controller_README.md`（制御系）
5. `doc/realsense_from_lib_README.md`, `doc/aruco_opencv_README.md`（認識系）
6. `doc/livox_ros_driver2_README.md`, `doc/livox_to_pointcloud2_README.md`, `doc/pointcloud_to_laserscan_README.md`（LiDAR 系）

---

## 13. `<要確認>` / `<要記入>` 一覧（着手前にユーザへ）
- git：`origin` / `upstream` の実 URL、共有リポジトリの default ブランチ名。
- ハード：Mid-360 の実 IP・ホスト NIC 設定、`base↔Mid-360` / `base↔D435i` の外部パラメータ実測値。
- SLAM：使用する FAST-LIO2 / livox ドライバ / localization リポジトリの確定ブランチ・バージョン。
- ERC（8月確定）：3D モデル形式・密度・視点、配布座標のフレーム規約（原点/軸/方位）、ArUco ID の明示有無、準備エリアの被覆範囲。
- 現行コード：`waypoints.yaml` の現行書式、`dual_ekf_navsat` の IMU remap、`make_direction.py` のトピック名不一致（`camera/camera/color/image_raw`）等の既知の注意点。

---

_最終更新：設計チャットでの確定内容を反映（Phase 0 = FAST-LIO2 mapping を承認済み。ERC 本番設計 = 事前マップ→定位 ＋ ArUco 大域アンカー ＋ Nav2 流用）。以降の設計変更はこのファイルに追記する。_