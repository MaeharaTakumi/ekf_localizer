#ifndef EKF_LOCALIZER__EKF_PARAMS_HPP_
#define EKF_LOCALIZER__EKF_PARAMS_HPP_

#include <Eigen/Core>

#include <string>

#include "rclcpp/rclcpp.hpp"

#include "ekf_localizer/ekf_core.hpp"
#include "ekf_localizer/ndt_model.hpp"
#include "ekf_localizer/odom_model.hpp"

namespace ekf_localizer
{

/// EKF ノードのパラメータ一式。
/// コードの既定値はこの構造体（と EkfCore・NdtModel・OdomModel の Params）の初期値だけで、
/// declareEkfParameters はここから宣言する。設定ファイルは localization_bringup の param/<env>/ekf_localizer.yaml
struct EkfParams
{
  /// オドメトリを観測に使うか
  bool use_odom{true};
  /// 最新の状態を現在時刻まで予測して TF・ekf_pose・ekf_odom を配信する周期 [Hz]
  double publish_rate{50.0};
  /// TF を publish_rate の周期の刻み（stamp が周期の整数倍）で送るとき、今より後の刻みを何個送るか。
  /// 0 なら今の時刻の 1 個だけ（刻みにしない）
  int tf_future_steps{0};
  /// tf_future_steps が 1 以上のとき、今以前の刻みを何個送り直すか（0 なら送らない）
  int tf_past_steps{0};
  /// 巻き戻しに備えて観測と状態を保持する長さ [s]（NDT の遅れより長くする）
  double history_length{1.0};
  std::string map_frame_id{"map"};
  std::string base_frame_id{"base_link"};
  /// NDT が推定する LiDAR のフレーム（取付 base → LiDAR は TF から読む）
  std::string lidar_frame_id{"velodyne"};
  /// NDT の遅れ [s]。点群の取得時刻が stamp より前の分を、stamp から引いて補う
  double ndt_delay{0.0};
  /// オドメトリの遅れ [s]。twist が実際の動きより遅れる分（移動平均など）を、stamp から引いて補う
  double odom_delay{0.0};
  /// オドメトリの v, omega に掛ける倍率 [v, omega]（実効的な車輪間隔・半径が設定と違う分の補正）
  Eigen::Vector2d odom_scale{1.0, 1.0};
  /// オドメトリの観測の時刻の決め方："stamp"（メッセージの stamp）、"receipt"（このノードが受け取った時刻）、
  /// "auto"（|受け取った時刻 − stamp| が odom_stamp_auto_threshold 以上なら受け取った時刻、小さければ stamp）
  std::string odom_stamp_mode{"stamp"};
  double odom_stamp_auto_threshold{0.1};

  EkfCore::Params core;
  /// 取付 base → LiDAR は TF から読んで createFilter() で入れる
  NdtModel::Params ndt;
  OdomModel::Params odom;
};

/// EKF のパラメータを宣言する（コンストラクタから呼ぶ）。既定値は EkfParams の初期値
void declareEkfParameters(rclcpp::Node & node);
/// パラメータを読み込む。不正な値は WARN を出して既定値に戻す。名前を変えた古いパラメータが渡されていれば WARN を出す
EkfParams loadEkfParams(rclcpp::Node & node);
/// 読み込んだ値を INFO で出す
void logEkfParams(const rclcpp::Logger & logger, const EkfParams & params);

}  // namespace ekf_localizer

#endif  // EKF_LOCALIZER__EKF_PARAMS_HPP_
