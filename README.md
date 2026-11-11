# ekf_localizer

NDT の位置推定（`ndt_pose`）と車輪オドメトリを融合する 11 状態 EKF です。
処理の流れは [docs/ekf_flow.md](docs/ekf_flow.md) を参照してください。

- 状態：`[x, y, z, roll, pitch, yaw, v_xy, v_z, omega_roll, omega_pitch, omega_yaw]`（`base_frame_id` の姿勢と、水平面の前向きの速さ・
  地図の z の速度・roll, pitch, yaw の変化率）。運動は各軸が独立（[docs/ekf_flow.md](docs/ekf_flow.md) の「運動モデル」）。
  `v_z`・`omega_roll`・`omega_pitch` は `Q`・`P_init` を 0 にすると 0 のまま動かず、足す前の 8 状態と同じになる（Gazebo の環境ファイル `gazebo.yaml` は 3 つとも 0。
  コードの既定は 2026-11-09 から実機 blue の値で、`omega_roll`・`omega_pitch` の `Q` 0.1・`v_z` 0）
- 観測：NDT の LiDAR 姿勢（取付 `base → LiDAR` は静的 TF から読む）、オドメトリの `[v_xy, omega_yaw]`（`use_odom: true`）
- **観測の遅れ**：NDT は `ndt_delay`、オドメトリは `odom_delay` 秒だけ stamp を戻して入れます
  （既定はどちらも 0。Gazebo の diff_drive_controller が twist を 0.2 s の移動平均で出していたころは `odom_delay` 0.09 s。
  2026-10-12 に移動平均を 1 にしてからは 0）。
- **巻き戻し（ルックバック）**：NDT の解は点群の取得時刻の観測として、遅れて届きます。
  NDT が届いたら取得時刻の直前の状態に戻し、それ以降のオドメトリを適用し直します（`LaggedEkf`）。
- **観測は届いた時点で更新**し、**予測は `predict_rate` の周期**で現在時刻まで行って配信します。

## IO

| 種類 | トピック（既定） | 型 | 内容 |
|---|---|---|---|
| 入力 | `ndt_pose` | geometry_msgs/PoseWithCovarianceStamped | NDT の解（map → LiDAR、stamp は点群の取得時刻。`frame_id` は見ない） |
| 入力 | `odom` | nav_msgs/Odometry | `twist.linear.x`, `twist.angular.z` に `odom_scale` を掛けて v_xy, omega_yaw の観測に使う |
| 入力 | `initialpose` | geometry_msgs/PoseWithCovarianceStamped | 受け取ると EKF をリセットし、次の NDT で初期化し直す |
| 出力 | `ekf_pose` | geometry_msgs/PoseWithCovarianceStamped | map → LiDAR（共分散は観測モデルで伝播） |
| 出力 | `ekf_odom` | nav_msgs/Odometry | map → base と v_xy（`twist.linear.x`）, omega_yaw（`twist.angular.z`）。v_z・omega_roll・omega_pitch は地図の z とオイラー角の変化率で、base 座標の twist と合わないので出さない |
| 出力 | TF | | `map_frame_id` → `base_frame_id`（常に配信）。`predict_rate` の周期で、最新の状態を現在時刻まで予測した値。NDT の初期値と MATLAB はこの TF を点群の stamp の時刻で引く（tf2 が前後の値を補間する）。MATLAB は最新の値を今の姿勢にも使う。`tf_future_steps` が 1 以上なら、周期の刻みの stamp で今より先の刻みも送る（下記） |

購読するトピック名はパラメータで変えられます（リマップでも変えられます）。

## パラメータ

値はコードの既定値（`src/ekf_params.cpp` の `declareEkfParameters`）。このパッケージにはパラメータファイルを置かない
（2026-11-11 に `param/ekf.yaml` をやめた）。環境ごとの値は [localization_bringup](https://github.com/MaeharaTakumi/localization_bringup) の
`param/<env>/ekf_localizer.yaml` に書き、書いていない項目はこの既定値になる。トピック名を変えるときは remap する
（2026-11-11 に `ndt_pose_topic`・`odom_topic`・`initialpose_topic` をやめた。localization_bringup の launch は `odom` を `/wheelchair/odom` にする）。

### 全体

| 名前 | 値 | 説明 |
|---|---|---|
| `use_odom` | true | オドメトリを観測に使うか。false なら NDT だけ（v, omega も NDT の姿勢の変化から推定） |
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
| `map_frame_id` | `map` | 地図のフレーム。出力（TF・`ekf_pose`・`ekf_odom`）の `frame_id`。`ndt_pose` はこのフレームの値として扱う |
| `base_frame_id` | `base_link` | 状態と TF の基準のフレーム |
| `lidar_frame_id` | `velodyne` | 点群のフレーム（NDT が推定する姿勢のフレーム）。取付 `base → LiDAR` を TF から読む |

### ノイズ・初期共分散

| 名前 | 値 | 説明 |
|---|---|---|
| `Q` | [0.05, 0.05, 0.005, 0.05, 0.05, 0.05, 0.05, 0, 0.1, 0.1, 10] | プロセスノイズ（スペクトル密度）[x, y, z, roll, pitch, yaw, v_xy, v_z, omega_roll, omega_pitch, omega_yaw] の 11 要素。単位 [m²/s], [rad²/s], [m²/s³], [rad²/s³]。速度は、速度がどれだけ急に変わりうるか。要素数が違うと WARN を出してコードの既定値になる（8 要素の古い設定も） |
| `P_init` | [0.2, 0.2, 0.2, 0.01, 0.01, 0.01, 0.01, 0, 0.01, 0.01, 0.01] | 初期化したときの共分散（分散、base）。並びは `Q` と同じ |
| `R_ndt` | [2.08e-4, 1.55e-4, 9.61e-5, 5.89e-5, 6.32e-5, 1.23e-5] | NDT の観測ノイズ（分散、LiDAR）[x, y, z, roll, pitch, yaw]。`ndt_pose` の共分散は使わない。更新では取付で base の姿勢の共分散（相関込み）に直して使う |
| `R_odom` | [0.00608, 0.000992] | オドメトリの観測ノイズ（分散）のうち、速さ・加速度によらない分 [v, omega] [m²/s²], [rad²/s²]。正の値 |

### NDT

| 名前 | 値 | 説明 |
|---|---|---|
| `ndt_delay` | 0.0 | NDT の遅れ [s]。点群の取得時刻が stamp より前の分を、stamp から引いて補う。`[0, history_length)` の範囲 |
| `gate_ndt` | 1.0e5 | NDT のゲート（マハラノビス距離の 2 乗の上限。1.0e5 は実質無効。χ²(6) の 99% は 16.81。`NdtModel::Params` の既定は 16.81 で、テストで使う）。NDT の解は base の姿勢に直して 6 成分を 1 回で更新するので、ゲートを効かせると、どれか 1 成分が大きくずれても観測全体を使わない |

### オドメトリ（`use_odom: true` のみ）

| 名前 | 値 | 説明 |
|---|---|---|
| `odom_noise_alpha` | [0, 0, 0, 0] | オドメトリの観測ノイズの速さに応じた分の係数 [v²→R_v, ω²→R_v, v²→R_omega, ω²→R_omega]。R_v = R_odom[0] + a[0] v² + a[1] ω² + (τ[0] dv/dt)²、R_omega = R_odom[1] + a[2] v² + a[3] ω² + (τ[1] dω/dt)² |
| `accel_noise_tau` | [0, 0] | オドメトリの観測ノイズの加速度に応じた分の時定数 [τ_v, τ_omega] [s]。加速度は直前のメッセージとの差 |
| `gate_odom` | 1.0e5 | オドメトリのゲート（1.0e5 は実質無効。χ²(2) の 99% は 9.21。`OdomModel::Params` の既定は 9.21 で、テストで使う） |
| `odom_scale` | [1.0, 1.0] | オドメトリの v, omega に掛ける倍率 [v, omega]（実効的な車輪間隔・半径が設定と違う分の補正。Gazebo は [1.0, 0.922]）。観測ノイズはこの倍率を掛けた値から計算する。2026-11-11 に localization_bringup の `odom_relay` から移した |
| `odom_stamp_mode` | `stamp` | オドメトリの観測の時刻の決め方。`stamp`：メッセージの stamp / `receipt`：このノードが受け取った時刻 / `auto`：\|受け取った時刻 − stamp\| が `odom_stamp_auto_threshold` 以上なら受け取った時刻、小さければ stamp（1 つずつ判断し、10 s ごとに使った数をログに出す）。sim time で `/clock` を受け取る前は stamp。NDT の観測は常に stamp（受信した時刻を使うときは NDT の `stamp_mode` が `/ndt_pose` の stamp に点群の受信時刻を入れる） |
| `odom_stamp_auto_threshold` | 0.1 | `auto` の閾値 [s] |
| `odom_timeout_init` | 0.2 | 初期化のとき、この時間以内のオドメトリがあれば v, omega をその値から始める [s] |
| `odom_delay` | 0.0 | オドメトリの遅れ [s]。stamp から引いて補う。`[0, history_length)` の範囲 |

## テスト

```bash
colcon build --packages-select ekf_localizer
colcon test --packages-select ekf_localizer --event-handlers console_direct+
```

`test_lagged_ekf` は、遅れて届いた観測を巻き戻して入れ直した結果が、時刻順にそのまま処理した結果と一致することを確認します。
