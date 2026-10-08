#include "ekf_localizer/vehicle_odom_ekf.hpp"

#include <cmath>

namespace ekf_localizer
{

bool VehicleOdomEkf::updateOdom(double v, double omega, const Eigen::Vector2d & r, double stamp)
{
  has_last_odom_ = true;
  last_odom_stamp_ = stamp;
  last_odom_z_ << v, omega;
  last_odom_r_ = r;

  if (!initialized_) {return false;}

  // h(x) = [v, omega]。H は v, omega の列だけが 1
  Eigen::Matrix<double, 2, kN> H = Eigen::Matrix<double, 2, kN>::Zero();
  H(0, kV) = 1.0;
  H(1, kOmega) = 1.0;
  const Eigen::Vector2d y = last_odom_z_ - Eigen::Vector2d(x_(kV), x_(kOmega));
  const Eigen::Matrix2d R = r.asDiagonal();
  if (!kalmanUpdate<2>(y, H, R, odom_prm_.gate_odom, last_odom_d2_)) {
    ++odom_reject_count_;
    return false;
  }
  odom_reject_count_ = 0;
  return true;
}

void VehicleOdomEkf::initialize(const Vector6d & z_lidar, double stamp)
{
  VehicleEkf::initialize(z_lidar, stamp);
  odom_reject_count_ = 0;
  last_odom_d2_ = 0.0;

  // 走行中に初期化し直したとき、v = 0 から出発して NDT に追いつくまでの過渡をなくす
  initialized_from_odom_ =
    has_last_odom_ && std::abs(stamp - last_odom_stamp_) <= odom_prm_.odom_timeout_init;
  if (initialized_from_odom_) {
    x_(kV) = last_odom_z_(0);
    x_(kOmega) = last_odom_z_(1);
    // 姿勢との相関は初期化直後なので 0 のまま
    P_(kV, kV) = last_odom_r_(0);
    P_(kOmega, kOmega) = last_odom_r_(1);
  }
}

void VehicleOdomEkf::reset()
{
  VehicleEkf::reset();
  odom_reject_count_ = 0;
  last_odom_d2_ = 0.0;
  initialized_from_odom_ = false;
  // 直近のオドメトリは残す（次の initialize() で使う）
}

}  // namespace ekf_localizer
