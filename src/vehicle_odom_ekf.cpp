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

  predictTo(stamp);

  // H = [0 | I2] なので S, K は P の右下ブロックと右 2 列だけで書ける
  const Eigen::Vector2d y = last_odom_z_ - x_.tail<2>();
  const Eigen::Matrix2d R = r.asDiagonal();
  const Eigen::Matrix2d S = P_.bottomRightCorner<2, 2>() + R;
  const Eigen::Matrix2d S_inv = S.inverse();

  const double d2 = y.dot(S_inv * y);
  last_odom_d2_ = d2;
  if (!std::isfinite(d2) || d2 > odom_prm_.gate_odom) {
    ++odom_reject_count_;
    return false;
  }
  odom_reject_count_ = 0;

  const Eigen::Matrix<double, kN, 2> K = P_.rightCols<2>() * S_inv;
  x_ += K * y;
  x_(kRoll) = normalizeAngle(x_(kRoll));
  x_(kPitch) = normalizeAngle(x_(kPitch));
  x_(kYaw) = normalizeAngle(x_(kYaw));
  Matrix8d KH = Matrix8d::Zero();   // K * H：H = [0 | I2] なので右 2 列が K
  KH.rightCols<2>() = K;
  const Matrix8d IKH = Matrix8d::Identity() - KH;
  P_ = IKH * P_ * IKH.transpose() + K * R * K.transpose();   // Joseph 形
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
    x_.tail<2>() = last_odom_z_;
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
