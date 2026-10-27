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
| 出力 | TF | | `map_frame_id` → `base_frame_id`（常に配信）。`predict_rate` の周期で、最新の状態を現在時刻まで予測した値。NDT の初期値と MATLAB はこの TF を点群の stamp の時刻で引く（tf2 が前後の値を補間する）。MATLAB は最新の値を今の姿勢にも使う。`tf_future_steps` が 1 以上なら、周期の刻みの stamp で今より先の刻みも送る（下記） |
| 入力 | `velodyne_points`（`points_topic`） | sensor_msgs/PointCloud2 | `time_source: receipt` のときだけ購読し、受信した時刻を NDT の観測の時刻にする |

購読するトピック名はパラメータで変えられます（リマップでも変えられます）。

## パラメータ

値は [param/ekf.yaml](param/ekf.yaml) のもの。

### 全体

| 名前 | 値 | 説明 |
|---|---|---|
| `use_odom` | true | オドメトリを観測に使うか。false なら NDT だけ（v, omega も NDT の姿勢の変化から推定）。コードの既定は false |
| `time_source` | `stamp` | 観測の時刻。`stamp`：メッセージの stamp（遅れて届いた NDT は巻き戻して入れ直す）/ `receipt`：受信した時刻。オドメトリはこのノードが受信した時刻、NDT はその点群（`points_topic`）を受信した時刻（NDT のノードに入っていたころの `time_source: wall` と同じ。NDT は計算の分遅れて届くので、点群の時刻まで巻き戻して入れる） |
| `predict_rate` | 50.0 | 最新の状態を現在時刻まで予測して TF・`ekf_pose`・`ekf_odom` を配信する周期 [Hz]。オドメトリと同じ 50 Hz にしている（点群は stamp から約 15 ms 後に届くので、NDT・MATLAB が点群の stamp より後の TF を待つのは最大 5 ms ほど。Gazebo の `/clock` は 10 ms 刻みなので 100 Hz より速くしても出る数は増えない） |
| `tf_future_steps` | 0 | TF を `predict_rate` の周期の刻み（stamp が周期の整数倍）で送るとき、今より後の刻みを何個送るか。0 なら今の時刻の 1 個だけ（刻みにしない）。下記 |
| `tf_past_steps` | 0 | `tf_future_steps` が 1 以上のとき、今以前の刻みを何個送り直すか。0 なら送らない。`history_length` より短くする |
| `history_length` | 1.0 | 巻き戻しのために観測を保持する長さ [s]。NDT の遅れ（計算時間＋通信）より長くする |
| `max_predict_dt` | 0.2 | 1 回の予測で進める時間の上限 [s]。下記 |

TF は、届いた観測をすべて入れた最新の状態を現在時刻まで予測した値です。一度出した値は書き直しません。
点群の時刻の NDT は約 50 ms 遅れて届き、その時刻まで巻き戻して入れるので、NDT が届いた直後の TF は
その分だけ跳ねます。点群の stamp で引いた TF は、その点群の NDT を入れる前の予測値（事前推定値）です。
観測が `max_predict_dt` より長く途切れると、それ以上は進みません（下記）。

`tf_future_steps`・`tf_past_steps`（既定 0）：今の TF は stamp が今の時刻なので、点群の stamp より後の TF が届くまで
（点群の stamp から平均約 10 ms、最大約 25 ms）NDT は待ち、MATLAB は前の組で代える。`tf_future_steps` を 1 以上にすると、
TF の stamp を周期 T（1 / `predict_rate`）の刻み k·T（整数のナノ秒）にそろえ、毎周期、今以前の最後の刻み k0 について
k0 − `tf_past_steps` + 1 〜 k0 + `tf_future_steps` の刻みの TF を 1 つの `/tf` メッセージで送る。値は
`LaggedEkf::at`（その刻み以前の観測をすべて入れた状態をその刻みまで予測した値。未来の刻みは最新の状態の予測）。
同じ刻みは周期ごとに新しい値で送り直し、購読側（tf2・MATLAB の `ros2tf`）では後から届いた値で置き換わる。
`tf_future_steps: 2` なら、点群が届いた時点で点群の stamp を挟む TF が揃っている。`tf_past_steps: 1` は、過ぎた刻みを
1 回送り直して、その間に届いたオドメトリを入れる（今の時刻の TF と同じ新しさになる）。2 以上にすると、遅れて届いた
NDT の更新も過去の刻みに入り、点群の stamp で引く値が、引く時刻によって事前推定値とその点群の NDT を入れた値の混ざったものになる。
最新を引く側（MATLAB の今の姿勢、RViz の視点）には (`tf_future_steps` − 1)·T 〜 `tf_future_steps`·T 先の予測が渡る。
`ekf_pose`・`ekf_odom` は今まで通り今の時刻の 1 個。初期化より前と履歴の起点より前の刻みは送らない。

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
| `R_ndt` | [0.01, 0.01, 0.01, 0.03, 0.03, 0.001] | NDT の観測ノイズ（分散、LiDAR）[x, y, z, roll, pitch, yaw]。`ndt_pose` の共分散は使わない。更新では取付で base の姿勢の共分散（相関込み）に直して使う |
| `R_odom` | [0.0025, 0.005] | オドメトリの観測ノイズ（分散）[v, omega] [m²/s²], [rad²/s²] |

### NDT

| 名前 | 値 | 説明 |
|---|---|---|
| `ndt_delay` | 0.0 | NDT の遅れ [s]。点群の取得時刻が stamp より前の分を、stamp から引いて補う。`[0, history_length)` の範囲 |
| `gate_ndt` | 1.0e5 | NDT のゲート（マハラノビス距離の 2 乗の上限。1.0e5 は実質無効。χ²(6) の 99% は 16.81 で、コードの既定）。NDT の解は base の姿勢に直して 6 成分を 1 回で更新するので、ゲートを効かせると、どれか 1 成分が大きくずれても観測全体を使わない |

### オドメトリ（`use_odom: true` のみ）

| 名前 | 値 | 説明 |
|---|---|---|
| `odom_covariance_source` | `param` | 観測ノイズの出どころ。`param`：`R_odom` / `message`：`twist.covariance[0]`・`[35]`（正でなければ `R_odom`） |
| `gate_odom` | 1.0e5 | オドメトリのゲート（1.0e5 は実質無効。χ²(2) の 99% は 9.21 で、コードの既定） |
| `odom_timeout_init` | 0.2 | 初期化のとき、この時間以内のオドメトリがあれば v, omega をその値から始める [s] |
| `odom_delay` | 0.0 | オドメトリの遅れ [s]。stamp（`time_source: receipt` なら受信した時刻）から引いて補う。`[0, history_length)` の範囲 |

## テスト

```bash
colcon build --packages-select ekf_localizer
colcon test --packages-select ekf_localizer --event-handlers console_direct+
```

`test_lagged_ekf` は、遅れて届いた観測を巻き戻して入れ直した結果が、時刻順にそのまま処理した結果と一致することを確認します。
