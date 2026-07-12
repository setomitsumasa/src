# ERC 新環境セットアップ手順書 — スキャン → 事前マップ → live 定位 ＋ オドメトリ表示

**対象:** ERC 自律走行スタック（`ares_erc_bringup`）を **新しい場所** で一から動かす手順。
**必要ハード:** Livox Mid-360 ＋ LAN ケーブル ＋ mini PC のみ（D435i・治具・ローバー実機は不要）。
**得られるもの:** その場の 3D 点群を事前マップ化し、その上で **live 点群を重ね合わせ（定位）＋ ロボットのオドメトリ（`map` フレーム）を表示**（＝ Phase 2-a まで）。

> URC(GNSS) 構成・Ericsii FAST-LIO は不変更。すべて `ares_erc_bringup` 内で完結（CLAUDE.md §1.4 / §6.6）。
> 各コマンドの詳細・トラブル対処は `ares_erc_bringup/README.md` を参照。ここは「新環境で通す最短経路」。

---

## 事前マップの保存場所（最重要・先に頭に入れる）

**正規の事前マップ = `ares_erc_bringup/maps/prior_map.pcd`**

- 定位・オドメトリの全 launch（`localize` / `odometry` / `place_waypoints` / `localize_demo`）は、
  引数なしだと **この `maps/prior_map.pcd` を読む**。
- **自作マップ**：スキャン結果 `FAST_LIO_ROS2/PCD/scans.pcd` を、確認後にここへコピー（=昇格）。
- **外部から与えられたマップ**：その `.pcd` を `maps/prior_map.pcd` として置くだけで定位可能
  （条件：PCL が読める `.pcd`／原点 (0,0,0)=発進点／+Z=重力上向き。原点・向きが違う場合は
  `config/localization.yaml` の `init_xyz`/`init_yaw` に発進姿勢を入れる）。
- `.pcd` は **git 管理外**（大きい・場所依存）。詳細は `ares_erc_bringup/maps/README.md`。

複数環境を持つ場合は `maps/lab_room.pcd`, `maps/marsyard.pcd` のように名前付きで保存し、
`pcd:=$HOME/real_ws/src/ares_erc_bringup/maps/<名前>.pcd` で選ぶ（または active へ cp）。

---

## Step 1 — ネットワーク（新 PC / 新 NIC のとき一度だけ）

Mid-360 は Ethernet 直結。ホスト有線 NIC を LiDAR と同セグメントにする。

```bash
# NIC を静的 IP に（ifname は自分の有線ポート名：eno1 / enp2s0 等）
nmcli con add type ethernet ifname eno1 con-name livox \
  ipv4.method manual ipv4.addresses 192.168.1.50/24
nmcli con up livox

ping -c 2 192.168.1.164          # Mid-360 に応答が返れば OK
```
- 既定は host `192.168.1.50` / lidar `192.168.1.164`（`config/MID360_config_erc.json`）。
- LiDAR の実 IP が違う場合（本体シールの下2桁 XX → `192.168.1.1XX`）は
  `MID360_config_erc.json` の `lidar_configs[0].ip` を直して再ビルド。
- 通らないと後で `Init lds lidar fail!` になる。

## Step 2 — ビルド

```bash
cd ~/real_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select fast_lio ares_erc_bringup --symlink-install
source install/setup.bash
```

## Step 3 — 環境をスキャンして 3D マップを作る

```bash
ros2 launch ares_erc_bringup mapping_mid360.launch.py     # RViz 同時起動
```
- RViz は **Fixed Frame = `camera_init`**（`body` にすると視点が動いて点群がにじむ）。
- **発進させたい地点・向きから開始**する（その姿勢が事前マップの原点＝datum 原点になる）。
- ゆっくり歩いて/走らせて周囲を一周。**開始地点に戻る**とループが閉じてズレが減る。
- 動かした瞬間に点群が猛烈に飛ぶ場合 → LiDAR↔IMU の extrinsic 問題（README「drifts violently」参照。
  本パッケージは `MID360_config_erc.json` で対策済み。再発時は `mid360_mapping.yaml` の
  `extrinsic_est_en: false` にフォールバック）。
- 十分スキャンできたら **Ctrl-C** → `FAST_LIO_ROS2/PCD/scans.pcd` に保存される。

## Step 4 — マップを確認して正規の事前マップへ昇格

```bash
# スキャンが妥当か確認（任意）
pcl_viewer ~/real_ws/src/FAST_LIO_ROS2/PCD/scans.pcd

# 良ければ active な事前マップへコピー（＝以降 launch が既定で読む）
cp ~/real_ws/src/FAST_LIO_ROS2/PCD/scans.pcd \
   ~/real_ws/src/ares_erc_bringup/maps/prior_map.pcd
```
> **外部マップを使う場合はここだけ差し替え**：`cp /path/to/given_map.pcd .../maps/prior_map.pcd`。
> Step 3 を飛ばして Step 6 へ進める。

## Step 5 —（任意）新環境の datum / waypoint を設定

Phase 2-a の定位・オドメトリ自体は waypoint 不要だが、RViz に目標点を出したい場合：

`ares_erc_bringup/config/waypoints_erc.yaml` を編集（**発進点を原点 (0,0) とする相対座標 [m]**、
x=前, y=左）。`datum.yaw_offset_deg` は datum 軸を発進方位からどれだけ回すか（既定 0）。

## Step 6 — live 定位（事前マップに live 点群を重ねる）

```bash
ros2 launch ares_erc_bringup localize.launch.py
```
- **重要**：GICP は局所最適化。**LiDAR を「スキャン開始時と同じ位置・向き」に置いてから**起動
  （ズレ 1m / 15°以内なら引き込む）。違う場所からなら `config/localization.yaml` の
  `init_xyz`/`init_yaw` を発進姿勢に設定。
- RViz（Fixed Frame=`map`）：**灰=事前マップ（固定）／赤=live スキャン**。
  静止で赤が灰に重なれば定位成立。`ros2 topic echo /erc/localization_fitness` が小さく安定。
- 赤が大きくズレて動かない＝発進姿勢ズレ、`fitness … rejected` 連発＝`fitness_max` を 0.3→0.6 に緩める。

## Step 7 — live オドメトリ表示（Phase 2-a 本体）

```bash
ros2 launch ares_erc_bringup odometry.launch.py
```
- Step 6 のスタック一式に、**大域オドメトリ**を足したもの。RViz（Fixed Frame=`map`）で
  灰=事前マップ／赤=live／**黄=姿勢矢印（`/erc/odometry`）／シアン=軌跡（`/erc/trajectory`）**。
- LiDAR を動かすと、矢印と軌跡が事前マップ上を追従する。
- 数値確認：
  ```bash
  ros2 topic echo /erc/odometry --once    # header.frame_id=map, child_frame_id=body
  ros2 topic hz   /erc/odometry           # 約 30 Hz
  ```

---

## 各段の確認ポイント

| Step | コマンド | 正しい状態 |
|---|---|---|
| 1 ネット | `ping 192.168.1.164` | 応答あり（0% loss） |
| 3 スキャン | RViz `/cloud_registered`、`ros2 topic hz /livox/lidar` | 部屋の形が二重にならず溜まる／約10Hz |
| 4 保存 | `ls -la maps/prior_map.pcd` | サイズが妥当（数十〜数百 MB） |
| 6 定位 | RViz 灰 vs 赤、`/erc/localization_fitness` | 赤が灰に重なる／fitness 小さく安定 |
| 7 オドメトリ | RViz 黄矢印・シアン軌跡、`/erc/odometry` | 動きに追従、frame_id=map / 30Hz |

## TF 権威（二重 publish しない・§6.2）

| TF | publisher |
|---|---|
| `map → camera_init`(odom) | `map_anchor`（GICP 大域補正） |
| `camera_init → body` | FAST-LIO（局所オドメトリ） |
| `map → datum`(yaw) | `erc_waypoints.py`（静的） |

`erc_odometry.py` は TF を **購読するだけ**（`map→body` を合成して `/erc/odometry` 化）。TF は出さない。

## コマンド早見表

```bash
# 毎回
cd ~/real_ws && source install/setup.bash

# 新環境を作る
ros2 launch ares_erc_bringup mapping_mid360.launch.py          # スキャン → Ctrl-C
cp ~/real_ws/src/FAST_LIO_ROS2/PCD/scans.pcd \
   ~/real_ws/src/ares_erc_bringup/maps/prior_map.pcd           # 昇格

# 既存 / 外部マップで動かす
ros2 launch ares_erc_bringup localize.launch.py               # 定位（重ね合わせ）
ros2 launch ares_erc_bringup odometry.launch.py               # 定位＋オドメトリ（Phase 2-a）
#   別マップ: pcd:=$HOME/real_ws/src/ares_erc_bringup/maps/<名前>.pcd

# ハード無しで動作確認
ros2 launch ares_erc_bringup place_waypoints.launch.py         # 保存マップ＋waypoint 表示
ros2 launch ares_erc_bringup localize_demo.launch.py           # 定位の合成デモ
```

---

## 次のステップ（Phase 3）

事前マップ／datum は「発進点原点・相対座標」で自己完結しているが、長時間ではドリフトが残る。
次は **ArUco タグによるマップ補正**：既知座標の ArUco を検出して `datum↔map` を滑らかに補正し、
大域一貫性を上げる（CLAUDE.md §6.4 / §6.5）。`map_anchor` の「遅い剛体補正」機構を土台に拡張する。
