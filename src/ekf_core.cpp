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
  // 各軸は独立。水平面を移動すると仮定し、水平面の前向きの速さ v_xy と yaw の角速度 omega_yaw を
  // 中点法（RK2）で積分する：
  //   yaw_m = yaw + omega_yaw dt / 2（区間の中点の向き）
  //   x += v_xy dt cos(yaw_m)、y += v_xy dt sin(yaw_m)、yaw += omega_yaw dt
  // z, roll, pitch はそれぞれ v_z, omega_roll, omega_pitch で積分する。速度は区間の中で一定なので、
  // 中点法でも厳密な積分と同じ：z += v_z dt、roll += omega_roll dt、pitch += omega_pitch dt
  // roll, pitch による v_xy, omega_yaw の分解は考えない。速度 5 つはランダムウォーク
  const double v_xy = x(kVxy);
  const double omega_yaw = x(kOmegaYaw);
  const double yaw_m = x(kYaw) + 0.5 * omega_yaw * dt;

  StateVector xn = x;
  xn(kX) += v_xy * dt * std::cos(yaw_m);
  xn(kY) += v_xy * dt * std::sin(yaw_m);
  xn(kZ) += x(kVz) * dt;
  // roll, pitch は更新のたびに wrapAngles で wrap する（ここで wrap すると角速度 0 でも丸めで値が変わる）
  xn(kRoll) += x(kOmegaRoll) * dt;
  xn(kPitch) += x(kOmegaPitch) * dt;
  xn(kYaw) = normalizeAngle(x(kYaw) + omega_yaw * dt);
  return xn;
}

EkfCore::StateMatrix EkfCore::transitionJacobian(const StateVector & x, double dt)
{
  const double v_xy = x(kVxy);
  const double omega_yaw = x(kOmegaYaw);
  const double yaw_m = x(kYaw) + 0.5 * omega_yaw * dt;
  const double c = std::cos(yaw_m);
  const double s = std::sin(yaw_m);

  StateMatrix F = StateMatrix::Identity();
  F(kX, kYaw) = -v_xy * dt * s;
  F(kY, kYaw) = v_xy * dt * c;
  F(kX, kVxy) = dt * c;
  F(kY, kVxy) = dt * s;
  // d(yaw_m)/d(omega_yaw) = dt / 2
  F(kX, kOmegaYaw) = -0.5 * v_xy * dt * dt * s;
  F(kY, kOmegaYaw) = 0.5 * v_xy * dt * dt * c;
  F(kYaw, kOmegaYaw) = dt;
  F(kZ, kVz) = dt;
  F(kRoll, kOmegaRoll) = dt;
  F(kPitch, kOmegaPitch) = dt;
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
