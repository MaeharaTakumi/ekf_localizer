#include "ekf_localizer/vehicle_odom_ekf.hpp"

#include <Eigen/LU>

#include <algorithm>
#include <cmath>

namespace ekf_localizer
{

void VehicleOdomEkf::setParams(const Params & p)
{
  VehicleEkf::setParams(p);
  applyOmegaScaleParams();
}

void VehicleOdomEkf::setOdomParams(const OdomParams & p)
{
  odom_prm_ = p;
  applyOmegaScaleParams();
}

void VehicleOdomEkf::applyOmegaScaleParams()
{
  prm_.q(kOmegaScale) = odom_prm_.omega_scale_q;
  prm_.p_init(kOmegaScale) = odom_prm_.omega_scale_var;
}

bool VehicleOdomEkf::updateOdom(double v, double omega, const Eigen::Vector2d & r, double stamp)
{
  has_last_odom_ = true;
  last_odom_stamp_ = stamp;
  last_odom_z_ << v, omega;
  last_odom_r_ = r;

  if (!initialized_) {return false;}

  predictTo(stamp);

  // h(x) = [v, s_omega · omega]。H は v, omega, s_omega の列だけが非 0
  const double s = x_(kOmegaScale);
  Eigen::Matrix<double, 2, kN> H = Eigen::Matrix<double, 2, kN>::Zero();
  H(0, kV) = 1.0;
  H(1, kOmega) = s;
  H(1, kOmegaScale) = x_(kOmega);
  const Eigen::Vector2d y = last_odom_z_ - Eigen::Vector2d(x_(kV), s * x_(kOmega));
  const Eigen::Matrix2d R = r.asDiagonal();
  const Eigen::Matrix<double, kN, 2> PHt = P_ * H.transpose();
  const Eigen::Matrix2d S = H * PHt + R;
  const Eigen::Matrix2d S_inv = S.inverse();

  const double d2 = y.dot(S_inv * y);
  last_odom_d2_ = d2;
  if (!std::isfinite(d2) || d2 > odom_prm_.gate_odom) {
    ++odom_reject_count_;
    return false;
  }
  odom_reject_count_ = 0;

  Eigen::Matrix<double, kN, 2> K = PHt * S_inv;
  // 旋回していないときは s_omega を動かさない（OdomParams::omega_scale_min_rate）。
  // 最適でないゲインになるが、Joseph 形なので共分散はそのゲインに対して正しい
  if (std::abs(x_(kOmega)) < odom_prm_.omega_scale_min_rate) {K.row(kOmegaScale).setZero();}
  x_ += K * y;
  x_(kRoll) = normalizeAngle(x_(kRoll));
  x_(kPitch) = normalizeAngle(x_(kPitch));
  x_(kYaw) = normalizeAngle(x_(kYaw));
  const StateMatrix IKH = StateMatrix::Identity() - K * H;
  P_ = IKH * P_ * IKH.transpose() + K * R * K.transpose();   // Joseph 形
  // 滑りの割合として不自然な値にならないようにする（旋回中の外れたオドメトリで暴れないため）
  x_(kOmegaScale) =
    std::clamp(x_(kOmegaScale), odom_prm_.omega_scale_min, odom_prm_.omega_scale_max);
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
    // s_omega は 1 から始めるので、omega はオドメトリの値そのもの
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
