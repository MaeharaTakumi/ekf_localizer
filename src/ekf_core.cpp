#include "ekf_localizer/ekf_core.hpp"

#include <algorithm>
#include <cmath>

namespace ekf_localizer
{

double normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

Eigen::Vector3d rpyFromRotation(const Eigen::Matrix3d & R)
{
  return Eigen::Vector3d(
    std::atan2(R(2, 1), R(2, 2)),
    std::asin(std::max(-1.0, std::min(1.0, -R(2, 0)))),
    std::atan2(R(1, 0), R(0, 0)));
}

Eigen::Matrix3d rotationFromRpy(double roll, double pitch, double yaw)
{
  return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
}

EkfCore::EkfCore()
: EkfCore(Params())
{
}

EkfCore::EkfCore(const Params & p)
: prm_(p)
{
}

// ---------------------------------------------------------------------------
// 運動モデル
// ---------------------------------------------------------------------------

Eigen::Affine3d EkfCore::poseFromState(const StateVector & x)
{
  Eigen::Affine3d T = Eigen::Affine3d::Identity();
  T.linear() = rotationFromRpy(x(kRoll), x(kPitch), x(kYaw));
  T.translation() = x.head<3>();
  return T;
}

void EkfCore::wrapAngles(StateVector & x)
{
  x(kRoll) = normalizeAngle(x(kRoll));
  x(kPitch) = normalizeAngle(x(kPitch));
  x(kYaw) = normalizeAngle(x(kYaw));
}

EkfCore::StateVector EkfCore::propagate(const StateVector & x, double dt)
{
  // 水平面を移動すると仮定し、車体 x 軸方向の速度 v と yaw の角速度 omega を中点法（RK2）で積分する：
  //   yaw_m = yaw + omega dt / 2（区間の中点の向き）
  //   x += v dt cos(yaw_m)、y += v dt sin(yaw_m)、yaw += omega dt
  // roll, pitch による v, omega の分解は考えない。z, roll, pitch は予測では動かさず、Q のノイズだけを与える。
  // v, omega はランダムウォーク
  const double v = x(kV);
  const double omega = x(kOmega);
  const double yaw_m = x(kYaw) + 0.5 * omega * dt;

  StateVector xn = x;
  xn(kX) += v * dt * std::cos(yaw_m);
  xn(kY) += v * dt * std::sin(yaw_m);
  xn(kYaw) = normalizeAngle(x(kYaw) + omega * dt);
  return xn;
}

EkfCore::StateMatrix EkfCore::transitionJacobian(const StateVector & x, double dt)
{
  const double v = x(kV);
  const double omega = x(kOmega);
  const double yaw_m = x(kYaw) + 0.5 * omega * dt;
  const double c = std::cos(yaw_m);
  const double s = std::sin(yaw_m);

  StateMatrix F = StateMatrix::Identity();
  F(kX, kYaw) = -v * dt * s;
  F(kY, kYaw) = v * dt * c;
  F(kX, kV) = dt * c;
  F(kY, kV) = dt * s;
  // d(yaw_m)/d(omega) = dt / 2
  F(kX, kOmega) = -0.5 * v * dt * dt * s;
  F(kY, kOmega) = 0.5 * v * dt * dt * c;
  F(kYaw, kOmega) = dt;
  return F;
}

// ---------------------------------------------------------------------------
// 初期化・予測
// ---------------------------------------------------------------------------

void EkfCore::initialize(const StateVector & x0, double stamp)
{
  x_ = x0;
  P_ = prm_.p_init.asDiagonal();
  std::fill(gates_.begin(), gates_.end(), GateState());
  t_last_ = stamp;
  initialized_ = true;
}

void EkfCore::reset()
{
  initialized_ = false;
  last_dt_ = 0.0;
  last_dt_clamped_ = false;
  std::fill(gates_.begin(), gates_.end(), GateState());
}

void EkfCore::predictTo(double stamp)
{
  if (!initialized_) {return;}

  double dt = stamp - t_last_;
  if (dt <= 0.0) {
    last_dt_ = 0.0;
    last_dt_clamped_ = false;
    return;
  }
  last_dt_clamped_ = (dt > prm_.max_predict_dt);
  if (last_dt_clamped_) {dt = prm_.max_predict_dt;}
  last_dt_ = dt;

  const StateMatrix F = transitionJacobian(x_, dt);
  x_ = propagate(x_, dt);

  P_ = F * P_ * F.transpose() + StateMatrix(prm_.q.asDiagonal()) * dt;

  t_last_ = stamp;
}

int EkfCore::addGates(int n)
{
  const int first = static_cast<int>(gates_.size());
  gates_.resize(gates_.size() + n);
  return first;
}

}  // namespace ekf_localizer
