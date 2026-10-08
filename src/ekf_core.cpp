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

namespace
{

/// sinc(u) = sin(u)/u と、その導関数。u → 0 では級数で評価する
void sincAndDerivative(double u, double & k, double & dk)
{
  if (std::abs(u) < 1.0e-3) {
    const double u2 = u * u;
    k = 1.0 - u2 / 6.0 + u2 * u2 / 120.0;
    dk = -u / 3.0 + u * u2 / 30.0;
  } else {
    const double s = std::sin(u);
    const double c = std::cos(u);
    k = s / u;
    dk = (u * c - s) / (u * u);
  }
}

}  // namespace

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
  // 車体 x 軸方向の速度 v を map に回すと (v cos(pitch) [cos yaw, sin yaw], -v sin(pitch))。
  // 水平は円弧の厳密積分を半角公式で書いた形（pitch は区間内一定）：
  //   変位 = v cos(pitch) dt * sinc(alpha/2) * [cos(yaw + alpha/2), sin(yaw + alpha/2)]
  // alpha → 0 で自然にオイラー積分に一致するので分岐が要らない
  const double v = x(kV);
  const double omega = x(kOmega);
  const double u = 0.5 * omega * dt;
  double k, dk;
  sincAndDerivative(u, k, dk);
  const double beta = x(kYaw) + u;
  const double ct = std::cos(x(kPitch));
  const double st = std::sin(x(kPitch));

  StateVector xn = x;
  xn(kX) += v * ct * dt * k * std::cos(beta);
  xn(kY) += v * ct * dt * k * std::sin(beta);
  xn(kZ) -= v * st * dt;   // ZYX ではピッチ正が機首下げ。上り坂は pitch < 0 で z が増える
  // yaw の変化率は本来 omega cos(roll)/cos(pitch) だが omega で近似する（1/12 勾配で 0.35%）
  xn(kYaw) = normalizeAngle(x(kYaw) + omega * dt);
  // roll, pitch, v, omega はランダムウォーク
  return xn;
}

EkfCore::StateMatrix EkfCore::transitionJacobian(const StateVector & x, double dt)
{
  const double v = x(kV);
  const double omega = x(kOmega);
  const double u = 0.5 * omega * dt;
  double k, dk;
  sincAndDerivative(u, k, dk);
  const double beta = x(kYaw) + u;
  const double cb = std::cos(beta);
  const double sb = std::sin(beta);
  const double ct = std::cos(x(kPitch));
  const double st = std::sin(x(kPitch));
  const double a = v * ct * dt;   // 水平方向の移動量（sinc 補正前）

  StateMatrix F = StateMatrix::Identity();
  F(kX, kPitch) = -v * st * dt * k * cb;
  F(kY, kPitch) = -v * st * dt * k * sb;
  F(kX, kYaw) = -a * k * sb;
  F(kY, kYaw) = a * k * cb;
  F(kX, kV) = ct * dt * k * cb;
  F(kY, kV) = ct * dt * k * sb;
  // d/d(omega)：u = omega*dt/2 なので du/d(omega) = dt/2
  F(kX, kOmega) = 0.5 * a * dt * (dk * cb - k * sb);
  F(kY, kOmega) = 0.5 * a * dt * (dk * sb + k * cb);
  F(kZ, kPitch) = -v * ct * dt;
  F(kZ, kV) = -st * dt;
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
