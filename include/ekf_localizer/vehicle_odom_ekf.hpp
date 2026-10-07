#ifndef EKF_LOCALIZER__VEHICLE_ODOM_EKF_HPP_
#define EKF_LOCALIZER__VEHICLE_ODOM_EKF_HPP_

#include "ekf_localizer/vehicle_ekf.hpp"

namespace ekf_localizer
{

/// VehicleEkf にオドメトリの v, omega を**観測**として加えた EKF（docs/ekf_odom_plan.md）
///
///   状態・運動モデル・NDT 観測は VehicleEkf と同じ。差分は次の 2 点だけ。
///     観測 z_odom = [v, omega]（base_link 基準なのでレバーアームなし）
///     初期化 直前のオドメトリがあれば v, omega をその値から始める
///
///   ゲートで棄却したオドメトリは捨てる（再初期化しない）。
///   **符号の誤りはゲートでは検出できない**（docs/ekf_odom_plan.md 1.2 節）。
class VehicleOdomEkf : public VehicleEkf
{
public:
  struct OdomParams
  {
    /// オドメトリ観測ノイズ [v, omega] の分散 [m^2/s^2], [rad^2/s^2]
    Eigen::Vector2d r_odom{2.5e-3, 5.0e-3};
    /// ゲート（既定 chi2(2, 0.99)）
    double gate_odom{9.21};
    /// NDT で初期化する時刻からこの秒数以内のオドメトリがあれば、v, omega をその値で初期化する
    double odom_timeout_init{0.2};
  };

  std::unique_ptr<VehicleEkf> clone() const override
  {
    return std::make_unique<VehicleOdomEkf>(*this);
  }

  void setOdomParams(const OdomParams & p) {odom_prm_ = p;}
  const OdomParams & odomParams() const {return odom_prm_;}

  /// オドメトリ観測。stamp まで予測してから更新する。
  /// r は観測ノイズの分散（r_odom かメッセージの値。呼び出し側が選ぶ）。
  /// 未初期化でも値は記録し、次の initialize() で使う。
  /// 戻り値：更新したら true、未初期化・ゲート棄却なら false
  bool updateOdom(double v, double omega, const Eigen::Vector2d & r, double stamp);

  /// VehicleEkf::initialize() のあと、直前のオドメトリで v, omega を上書きする（1.4 節）
  void initialize(const Vector6d & z_lidar, double stamp) override;
  void reset() override;

  int odomRejectCount() const {return odom_reject_count_;}
  double lastOdomMahalanobis() const {return last_odom_d2_;}
  /// 直近の initialize() で v, omega をオドメトリから初期化したか
  bool initializedFromOdom() const {return initialized_from_odom_;}

private:
  OdomParams odom_prm_;
  int odom_reject_count_{0};
  double last_odom_d2_{0.0};
  bool initialized_from_odom_{false};

  // 直近に受け取ったオドメトリ（初期化用）
  bool has_last_odom_{false};
  double last_odom_stamp_{0.0};
  Eigen::Vector2d last_odom_z_{Eigen::Vector2d::Zero()};
  Eigen::Vector2d last_odom_r_{Eigen::Vector2d::Zero()};
};

}  // namespace ekf_localizer

#endif  // EKF_LOCALIZER__VEHICLE_ODOM_EKF_HPP_
