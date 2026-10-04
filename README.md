# ekf_localizer

NDT の位置推定（[lidar_localization](https://github.com/MaeharaTakumi/lidar_localization) の `/ndt_pose`）と
車輪オドメトリを融合する 8 状態 EKF です。
環境ごとの起動は [localization_bringup](https://github.com/MaeharaTakumi/localization_bringup) を使います。

- 状態：`[x, y, z, roll, pitch, yaw, v, omega]`（`base_frame_id` の姿勢と、前進速度・ヨー角速度）
- 観測：NDT の LiDAR 姿勢（取付 `base → LiDAR` は静的 TF から読む）、オドメトリの `[v, omega]`
- **巻き戻し（ルックバック）**：NDT の解は点群の取得時刻の観測として、遅れて届きます。
  NDT が届いたら取得時刻の直前の状態に戻し、それ以降のオドメトリを適用し直します（`LaggedEkf`）。
- **観測は届いた時点で更新**し、**予測は `predict_rate` の周期**で現在時刻まで行って配信します。

## IO

| 種類 | トピック | 型 | 内容 |
|---|---|---|---|
| 入力 | `ndt_pose` | geometry_msgs/PoseWithCovarianceStamped | NDT の解（map → LiDAR、stamp は点群の取得時刻） |
| 入力 | `odom` | nav_msgs/Odometry | `twist.linear.x`, `twist.angular.z` を v, omega として使う |
| 入力 | `initialpose` | geometry_msgs/PoseWithCovarianceStamped | 受け取ると EKF をリセットし、次の NDT で初期化し直す |
| 出力 | `ekf_pose` | geometry_msgs/PoseWithCovarianceStamped | map → LiDAR（共分散は観測モデルで伝播） |
| 出力 | `ekf_odom` | nav_msgs/Odometry | map → base と v, omega |
| 出力 | TF | | `map_frame_id` → `base_frame_id`（`publish_tf`） |

## パラメータ

[param/ekf.yaml](param/ekf.yaml) を参照してください。主なもの：

| 名前 | 既定値 | 説明 |
|---|---|---|
| `ekf_model` | `ndt_odom` | `ndt_only`：NDT のみ / `ndt_odom`：NDT ＋ オドメトリ |
| `predict_rate` | 50.0 | 予測して配信する周期 [Hz] |
| `history_length` | 1.0 | 巻き戻しのために保持する長さ [s]。NDT の遅れ（計算時間＋通信）より長くする |
| `base_frame_id` / `lidar_frame_id` | `base_link` / `velodyne` | 状態の基準フレームと点群のフレーム |

## テスト

```bash
colcon build --packages-select ekf_localizer
colcon test --packages-select ekf_localizer --event-handlers console_direct+
```

`test_lagged_ekf` は、遅れて届いた観測を巻き戻して入れ直した結果が、時刻順にそのまま処理した結果と一致することを確認します。
