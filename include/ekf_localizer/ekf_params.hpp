#ifndef EKF_LOCALIZER__EKF_PARAMS_HPP_
#define EKF_LOCALIZER__EKF_PARAMS_HPP_

#include <Eigen/Geometry>

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "ekf_localizer/vehicle_ekf.hpp"
#include "ekf_localizer/vehicle_odom_ekf.hpp"

namespace ekf_localizer
{

/// EKF ノードのパラメータ一式（param/ekf.yaml）
struct EkfConfig
{
  /// オドメトリを観測に使うか（true：VehicleOdomEkf / false：VehicleEkf）
  bool use_odom{false};
  /// 購読するトピック
  std::string ndt_pose_topic{"ndt_pose"};
  std::string odom_topic{"odom"};
  std::string initialpose_topic{"initialpose"};
  /// 予測して配信する周期 [Hz]
  double predict_rate{50.0};
  /// 巻き戻しに備えて観測と状態を保持する長さ [s]（NDT の遅れより長くする）
  double history_length{1.0};
  /// map → base_frame_id の TF を配信するか
  bool publish_tf{true};
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

  /// 取付（o_x〜yaw_o）は TF から読んで createEkf() で入れる
  VehicleEkf::Params params;
  VehicleOdomEkf::OdomParams odom_params;
};

/// EKF のパラメータを宣言する（コンストラクタから呼ぶ）
void declareEkfParameters(rclcpp::Node & node);
/// パラメータを読み込む。不正な値は WARN を出して既定値に戻す
EkfConfig loadEkfConfig(rclcpp::Node & node);
/// 読み込んだ値を INFO で出す
void logEkfConfig(const rclcpp::Logger & logger, const EkfConfig & cfg);
/// use_odom に応じた EKF を作る。mount は base → LiDAR の取付変換
std::unique_ptr<VehicleEkf> createEkf(const EkfConfig & cfg, const Eigen::Affine3d & mount);

}  // namespace ekf_localizer

#endif  // EKF_LOCALIZER__EKF_PARAMS_HPP_
