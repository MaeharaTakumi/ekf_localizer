# ekf_localizer

NDT の位置推定（`ndt_pose`）と車輪オドメトリを融合する 8 状態 EKF です。
処理の流れは [docs/ekf_flow.md](docs/ekf_flow.md) を参照してください。

- 状態：`[x, y, z, roll, pitch, yaw, v, omega]`（`base_frame_id` の姿勢と、前進速度・ヨー角速度）
- 観測：NDT の LiDAR 姿勢（取付 `base → LiDAR` は静的 TF から読む）、オドメトリの `[v, omega]`（`use_odom: true`）
- **観測の遅れ**：NDT は `ndt_delay`、オドメトリは `odom_delay` 秒だけ stamp を戻して入れます
  （Gazebo の diff_drive_controller は twist を 0.2 s の移動平均で出すので `odom_delay` は 0.09 s）。
- **巻き戻し（ルックバック）**：NDT の解は点群の取得時刻の観測として、遅れて届きます。
  NDT が届いたら取得時刻の直前の状態に戻し、それ以降のオドメトリを適用し直します（`LaggedEkf`）。
- **観測は届いた時点で更新**し、**予測は `predict_rate` の周期**で現在時刻まで行って配信します。

## IO

| 種類 | トピック（既定） | 型 | 内容 |
|---|---|---|---|
| 入力 | `ndt_pose`（`ndt_pose_topic`） | geometry_msgs/PoseWithCovarianceStamped | NDT の解（map → LiDAR、stamp は点群の取得時刻。`frame_id` は見ない） |
| 入力 | `odom`（`odom_topic`） | nav_msgs/Odometry | `twist.linear.x`, `twist.angular.z` を v, omega として使う |
| 入力 | `initialpose`（`initialpose_topic`） | geometry_msgs/PoseWithCovarianceStamped | 受け取ると EKF をリセットし、次の NDT で初期化し直す |
| 出力 | `ekf_pose` | geometry_msgs/PoseWithCovarianceStamped | map → LiDAR（共分散は観測モデルで伝播） |
| 出力 | `ekf_odom` | nav_msgs/Odometry | map → base と v, omega |
| 出力 | TF | | `map_frame_id` → `base_frame_id`（`publish_tf`） |

購読するトピック名はパラメータで変えられます（リマップでも変えられます）。

## パラメータ

値は [param/ekf.yaml](param/ekf.yaml) のもの。

### 全体

| 名前 | 値 | 説明 |
|---|---|---|
| `use_odom` | true | オドメトリを観測に使うか。false なら NDT だけ（v, omega も NDT の姿勢の変化から推定）。コードの既定は false |
| `predict_rate` | 50.0 | 予測して `ekf_pose`・`ekf_odom`・TF を配信する周期 [Hz] |
| `history_length` | 1.0 | 巻き戻しのために観測を保持する長さ [s]。NDT の遅れ（計算時間＋通信）より長くする |
| `max_predict_dt` | 0.2 | 1 回の予測で進める時間の上限 [s]。下記 |
| `publish_tf` | true | `map_frame_id` → `base_frame_id` の TF を配信するか |

`max_predict_dt`：前の状態からの間隔がこれを超えると、動かすのはこの時間分だけにして、時刻は最後まで進めます
（共分散もこの時間分しか増やしません）。オドメトリ（50 Hz）や NDT（約 10 Hz）が届いている間は効きません。
観測が途切れたときに、古い v, omega のまま大きく外挿しないための上限です。sim time の `/clock` が飛んだときも同様です。
途中を刻んで積分するのではなく、超えた分の動きは捨てます。

### トピック・フレーム

| 名前 | 値 | 説明 |
|---|---|---|
| `ndt_pose_topic` | `ndt_pose` | NDT の解のトピック |
| `odom_topic` | `odom` | オドメトリのトピック |
| `initialpose_topic` | `initialpose` | 初期姿勢（リセット）のトピック |
| `map_frame_id` | `map` | 地図のフレーム。出力（TF・`ekf_pose`・`ekf_odom`）の `frame_id`。`ndt_pose` はこのフレームの値として扱う |
| `base_frame_id` | `base_link` | 状態と TF の基準のフレーム |
| `lidar_frame_id` | `velodyne` | 点群のフレーム（NDT が推定する姿勢のフレーム）。取付 `base → LiDAR` を TF から読む |

### ノイズ・初期共分散

| 名前 | 値 | 説明 |
|---|---|---|
| `Q` | [0.05, 0.05, 0.005, 0.05, 0.05, 0.05, 0.05, 0.01] | プロセスノイズ（スペクトル密度）[x, y, z, roll, pitch, yaw, v, omega]。単位 [m²/s], [rad²/s], [m²/s³], [rad²/s³]。v, omega は速度がどれだけ急に変わりうるか |
| `P_init` | [0.2, 0.2, 0.2, 0.01, 0.01, 0.01, 0.01, 0.01] | 初期化したときの共分散（分散、base）[x, y, z, roll, pitch, yaw, v, omega] |
| `R_ndt` | [0.01, 0.01, 0.01, 0.03, 0.03, 0.001] | NDT の観測ノイズ（分散、LiDAR）[x, y, z, roll, pitch, yaw]。`ndt_pose` の共分散は使わない |
| `R_odom` | [0.0025, 0.005] | オドメトリの観測ノイズ（分散）[v, omega] [m²/s²], [rad²/s²] |

### NDT

| 名前 | 値 | 説明 |
|---|---|---|
| `ndt_delay` | 0.0 | NDT の遅れ [s]。点群の取得時刻が stamp より前の分を、stamp から引いて補う。`[0, history_length)` の範囲 |
| `gate_horizontal` | 11.34 | NDT の [x, y, yaw] のゲート（χ²(3) の 99%） |
| `gate_1d` | 6.63 | NDT の z, roll, pitch それぞれのゲート（χ²(1) の 99%） |
| `lockout_count` | 10 | z, roll, pitch がこの回数続けて棄却されたら、その成分を観測値で再初期化する（0 以下で無効） |

### オドメトリ（`use_odom: true` のみ）

| 名前 | 値 | 説明 |
|---|---|---|
| `odom_covariance_source` | `param` | 観測ノイズの出どころ。`param`：`R_odom` / `message`：`twist.covariance[0]`・`[35]`（正でなければ `R_odom`） |
| `gate_odom` | 1.0e9 | オドメトリのゲート（χ²(2) の 99% は 9.21。1.0e9 は実質無効） |
| `odom_timeout_init` | 0.2 | 初期化のとき、この時間以内のオドメトリがあれば v, omega をその値から始める [s] |
| `odom_delay` | 0.09 | オドメトリの遅れ [s]。stamp から引いて補う。`[0, history_length)` の範囲 |

## テスト

```bash
colcon build --packages-select ekf_localizer
colcon test --packages-select ekf_localizer --event-handlers console_direct+
```

`test_lagged_ekf` は、遅れて届いた観測を巻き戻して入れ直した結果が、時刻順にそのまま処理した結果と一致することを確認します。
