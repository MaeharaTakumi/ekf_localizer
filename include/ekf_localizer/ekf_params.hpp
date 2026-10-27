#ifndef EKF_LOCALIZER__EKF_PARAMS_HPP_
#define EKF_LOCALIZER__EKF_PARAMS_HPP_

#include <Eigen/Geometry>

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "ekf_localizer/ekf_core.hpp"
#include "ekf_localizer/lagged_ekf.hpp"
#include "ekf_localizer/ndt_model.hpp"
#include "ekf_localizer/odom_model.hpp"

namespace ekf_localizer
{

/// EKF ノードのパラメータ一式（param/ekf.yaml）
struct EkfConfig
{
  /// オドメトリを観測に使うか
  bool use_odom{false};
  /// 購読するトピック
  std::string ndt_pose_topic{"ndt_pose"};
  std::string odom_topic{"odom"};
  std::string initialpose_topic{"initialpose"};
  /// 観測の時刻："stamp"（メッセージの stamp。遅れて届いた観測は巻き戻して入れ直す）or
  /// "receipt"（このノードが受信した時刻。届いた順に処理され、巻き戻しは起きない）
  std::string time_source{"stamp"};
  /// time_source: "receipt" で購読する点群（NDT の観測の時刻に、その点群を受信した時刻を使う）
  std::string points_topic{"velodyne_points"};
  /// 最新の状態を現在時刻まで予測して TF・ekf_pose・ekf_odom を配信する周期 [Hz]
  double predict_rate{50.0};
  /// TF を predict_rate の周期の刻み（stamp が周期の整数倍）で送るとき、今より後の刻みを何個送るか。
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
  /// オドメトリ観測ノイズの出どころ："param"（R_odom）or "message"（twist.covariance）
  std::string odom_covariance_source{"param"};
  /// NDT の遅れ [s]。点群の取得時刻が stamp より前の分を、stamp から引いて補う
  double ndt_delay{0.0};
  /// オドメトリの遅れ [s]。twist が実際の動きより遅れる分（移動平均など）を、stamp から引いて補う
  double odom_delay{0.0};

  EkfCore::Params core;
  /// 取付 base → LiDAR は TF から読んで createFilter() で入れる
  NdtModel::Params ndt;
  OdomModel::Params odom;
};

/// EKF 一式：履歴つきの EKF 本体と、観測モデル
struct EkfFilter
{
  std::shared_ptr<const NdtModel> ndt;
  /// use_odom: false なら null
  std::shared_ptr<const OdomModel> odom;
  std::unique_ptr<LaggedEkf> lagged;
};

/// EKF のパラメータを宣言する（コンストラクタから呼ぶ）
void declareEkfParameters(rclcpp::Node & node);
/// パラメータを読み込む。不正な値は WARN を出して既定値に戻す
EkfConfig loadEkfConfig(rclcpp::Node & node);
/// 読み込んだ値を INFO で出す
void logEkfConfig(const rclcpp::Logger & logger, const EkfConfig & cfg);
/// EKF 一式を作る（use_odom: true ならオドメトリの観測モデルも）。mount は base → LiDAR の取付変換
EkfFilter createFilter(const EkfConfig & cfg, const Eigen::Affine3d & mount);

}  // namespace ekf_localizer

#endif  // EKF_LOCALIZER__EKF_PARAMS_HPP_
