# ERC Phase 0 進捗ログ — Livox Mid-360 + FAST-LIO2 mapping

**日付:** 2026-07-11  **ブランチ:** `feat/slam-fastlio`  **担当:** 自律班 (setomitsumasa) + Claude Code

ERC GPS-free 自律走行の Phase 0（Mid-360 + FAST-LIO2 で 3D 点群地図を作り `.pcd` 保存）の立ち上げ。**URC 構成は一切変更せず**、ERC 用に並存させた（CLAUDE.md §1.4）。

---

## 1. やったこと

### git 安全体制（§1.3）
- `origin` = 個人 `github.com/setomitsumasa/src` のみ push 先。
- 共有リポ **`upstream`(先輩 karisora/src) と `urc2026`(チーム aresproject-jp) は push を二重に無効化**:
  - `.claude/settings.json` の `permissions.deny` で force push と両リポへの直 push を拒否。
  - `git remote set-url --push … DISABLED-push-to-origin-only`（git レベルでも push 不可、fetch は可）。
- 作業ブランチ `feat/slam-fastlio` を作成。

### 新規パッケージ `ares_erc_bringup`
ERC 用 launch/config を集約。中身:
- `launch/mapping_mid360.launch.py` — Livox ドライバ + FAST-LIO2 (+ RViz) を一括起動。
- `config/mid360_mapping.yaml` — FAST-LIO2 パラメータ（topic・extrinsic・pcd 保存）。
- `config/MID360_config_erc.json` — ドライバ設定（extrinsic ゼロ。理由は §3）。

### FAST-LIO2 (`FAST_LIO_ROS2`)
- `Ericsii/FAST_LIO_ROS2`（ros2 ブランチ, Humble）を clone → 既存流儀に合わせ **vendored 化**（nested `.git` 除去、`livox_ros_driver2` と同じ扱い）。
- 選定理由: CPU のみ・Mid-360 ネイティブ（CustomMsg + 内蔵IMU tight coupling）で ERC の GNSS-free 自律に最適（CLAUDE.md §6.2）。

---

## 2. 実機で検証できたこと（すべて OK）

| 段階 | 結果 |
|---|---|
| ネットワーク | `eno1` = `192.168.1.50/24`、Mid-360 `192.168.1.164` に ping 成功 |
| Livox ドライバ | `/livox/lidar` **10 Hz**(CustomMsg)、`/livox/imu` **~200 Hz**、`Init lds lidar success!` |
| IMU 健全性 | `linear_acceleration.z ≈ 9.78 m/s²`（正常。g 単位バグではない） |
| FAST-LIO2 | `/Odometry` **10 Hz**、`IMU Initial Done`、静止時ドリフト無し |
| .pcd 保存 | 静止スキャンで **8m × 4m × 2.5m の部屋**を取得（685k点、23MB）を確認 |

---

## 3. 見つけて直した不具合（3件）

1. **`.pcd` が保存されない** — vendored FAST-LIO が地図蓄積コード(`pcl_wait_save`)を **コメントアウト**していて、`pcd_save_en:true` でも無言で空保存。→ 該当ブロックを再有効化（`// [ARES]`）。Ctrl-C で `FAST_LIO_ROS2/PCD/scans.pcd` に全地図が保存されるように。

2. **RViz で円錐に見える** — Fixed Frame が `body`（動くセンサ視点）だった。→ **`camera_init`**（世界固定）にすると部屋が正しく蓄積表示される。保存 pcd 自体は正常だった。

3. **動かすと猛烈にドリフト（発散）** — `MID360_config.json` の rover 搭載用 extrinsic `pitch/yaw = 180/180` を、ドライバが **点群だけに適用し IMU には適用しない**ため LiDAR↔IMU が 180° 食い違い、回転運動で発散。IMU 故障ではない。→ ERC 用 `MID360_config_erc.json`（extrinsic ゼロ、標準構成）を作り launch を差し替え。URC 設定は不変更。**実機での動作再検証は LiDAR 冷却後にユーザが実施予定。**

---

## 4. 現在の状態と次のステップ

- **状態:** `feat/slam-fastlio` にローカルで完成。build/検証済み。§3-3 の発散修正のみ実機再検証待ち。
- **使い方:** `ros2 launch ares_erc_bringup mapping_mid360.launch.py` → 静止10秒 → ゆっくり移動 → Ctrl-C で `scans.pcd` 保存。詳細は `ares_erc_bringup/README.md`。
- **フォールバック:** 動かしてまだ発散する場合、`mid360_mapping.yaml` の `extrinsic_est_en: false` を試す。
- **次 (Phase 1〜):** 既知/自作マップ上でのローカライズ（FAST_LIO_LOCALIZATION 相当）、`map→odom` 権威を SLAM に移行、直交 waypoint 化（CLAUDE.md §8）。
- **要確認（後日）:** `base_link↔各センサ` の実測 extrinsic（Phase 2）、ERC 8月配布データ（座標系・ArUco ID 等）。
