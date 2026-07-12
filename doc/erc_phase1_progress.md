# ERC Phase 1 進捗ログ — datum 整合 (1a) ＋ 事前マップ定位 (1b)

**日付:** 2026-07-11  **ブランチ:** `feat/slam-fastlio`  **担当:** 自律班 (setomitsumasa) + Claude Code

Phase 0（Mid-360 + FAST-LIO2 で `.pcd` 保存）の続き。Phase 1 は **「事前マップ上で自己位置を決める」** ところまで。すべて既存 `ares_erc_bringup` に追加し、**URC 構成・Ericsii FAST-LIO は不変更**（CLAUDE.md §1.4 / §6.6）。

---

## Milestone 1a — datum ↔ map 整合と 4 waypoint の配置

ERC の waypoint は **発進点を原点 (0,0) とする相対直交座標（datum）**で与えられる（GNSS ではない）。FAST-LIO の `map` 原点＝発進点なので、datum→map は **並進0・yaw θ だけ**の静的変換。

- `config/waypoints_erc.yaml` — datum 相対の 4点＋finish、`datum.yaw_offset_deg`。
- `scripts/erc_waypoints.py` — 静的TF `map→datum`(yaw)、`/erc/waypoint_markers`(RViz)、`/erc/waypoint_poses`(Phase2 Nav2 用)。
- `launch/place_waypoints.launch.py` — **ハード不要**のオフライン配置デモ（保存 `.pcd`＋4点＋RViz）。
- **検証済み:** 実機スキャンマップ上に指定4点が正しく表示されることをユーザ確認（`ros2 launch ares_erc_bringup erc_waypoints.launch.py`）。

## Milestone 1b — 事前マップ上リローカライズ（`map_anchor`）

本番20分は mapping のみだとドリフトが蓄積する。→ **保存 `.pcd` に毎周期アンカー**して大域一貫な姿勢を保つ（CLAUDE.md §6.3「FAST_LIO_LOCALIZATION 相当」）。

**方式＝軽量な自作アンカーノード**（liangheming 一式採用や NDT パッケージ横付けは、ビルド/統合コストが重く却下）。検証済み Ericsii FAST-LIO を局所オドメトリの権威として残し、その上に大域補正を1ノード足す。

- `src/map_anchor.cpp`（C++/PCL GICP）— 事前 `.pcd` を target に、live `/cloud_registered`(camera_init系) を短時間累積して source に、低レート GICP。**fitness ゲート＋ジャンプ棄却＋ローパス**で `map→camera_init` を滑らかに補正して常時 broadcast。`/erc/localization_fitness` も publish。
- `config/localization.yaml` — voxel/GICP/更新レート/ゲート/ローパス/初期姿勢。
- `launch/localize.launch.py` — `mapping_mid360`(rviz off) ＋ `map_anchor` ＋ `erc_waypoints` ＋ 事前マップ表示 ＋ RViz(Fixed=`map`)。
- `launch/localize_demo.launch.py` — **ハード不要**の定位可視化デモ。事前マップを疑似 live(`/cloud_registered`)として再生し、わざと 0.6m/0.15rad ずらして GICP が引き戻す様子を RViz で見せる。
- `rviz/erc_localize.rviz` — 事前マップ(灰・固定) vs live 点群(赤) の重なりで定位を目視確認。

**TF 権威は単一**（§6.2 二重 publish 禁止を満たす）:

| TF | publisher |
|---|---|
| `map → camera_init`(odom) | **map_anchor** |
| `camera_init → body` | FAST-LIO |
| `map → datum`(yaw) | erc_waypoints |

→ チェーン `map → camera_init(odom) → body`、waypoint は `map` に固定。

---

## 検証（ハード無しで end-to-end 確認済み）

事前マップ自身を `/cloud_registered` として流し込み（＝FAST-LIO が原点でマップ形状を報告する状況を合成）、**わざと初期 x=0.3m のオフセット**を与えて GICP が原点へ引き戻すかを確認:

| 時刻 | `map→camera_init` の x | fitness |
|---|---|---|
| 初期 | 0.30 m（意図的オフセット） | — |
| ~6s | 0.038 m | ~1e-9 |
| ~18s | 0.000 m（収束） | ~1e-10 |

→ マップ読込→累積→GICP→ゲート→ローパス→TF常時publish の**全経路が動作**。棄却・非収束の警告なし。実機は `/cloud_registered` を供給するだけで同一インターフェース。

### 実機 live 検証（2026-07-11 実施・成功）

- `localize_demo.launch.py`（合成 live）で、赤い点群が灰色マップへスライドして重なる様子をユーザが目視確認。
- その後 **Mid-360 を実接続**（`eno1` = 192.168.1.50/24、`ping 192.168.1.164` 応答）し `localize.launch.py` を実行。**live スキャン(赤)が事前マップ(灰)に重なり、定位成立をユーザ確認** → **Phase 1 完了**。

---

## 現在の状態と次のステップ

- **状態:** `feat/slam-fastlio` にローカルで完成、build 済み、**オフライン＋実機 live 検証済み**（未 commit）。
- **使い方:** `ros2 launch ares_erc_bringup localize.launch.py` → RViz(Fixed=`map`) で事前マップ(灰)と live(赤)が重なれば定位成功。詳細は `ares_erc_bringup/README.md`「Phase 1b」。
- **要点（live 起動時）:** GICP は局所最適化なので、**発進姿勢＝マッピング開始姿勢**に LiDAR を置いてから起動する（ズレ 1m/15°以内で引き込む）。発進点が違う場合は `init_xyz`/`init_yaw`。棄却連発なら `fitness_max` を緩める。
- **次 (Phase 2 / Milestone 2a):** GICP 定位を継続しつつ、live LiDAR からロボットの **大域オドメトリ（`map` フレーム）を出力＆表示**（`map→body` を合成した `nav_msgs/Odometry` ＋ 軌跡 `Path`）。Nav2 が消費する定位出力の土台。
- **次 (Phase 2 / Milestone 2b):** `map` を Nav2 の global frame に接続し、`/erc/waypoint_poses` を `NavigateToPose` へ順次送る Cartesian waypoint follower を新規実装（URC の `gps_waypoint_follower` は流用せず `makePoseStamped` パターンのみ再利用）。live 点群 → costmap。
- **次 (Phase 3):** `map_anchor` の「遅い剛体補正」機構を土台に、ArUco 既知座標の大域アンカー（§6.4）を追加。
