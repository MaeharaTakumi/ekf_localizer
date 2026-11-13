#include "ekf_localizer/odom_model.hpp"

#include <cmath>

namespace ekf_localizer
{

OdomModel::OdomModel(const Params & params, EkfCore & core)
: params_(params), gate_index_(core.addGates(1))
{
}

MeasurementPtr OdomModel::measurement(
  double v, double omega, const Eigen::Vector2d & r, double t) const
{
  return makeMeasurement(t, Eigen::Vector2d(v, omega), r);
}

bool OdomModel::aidInitialization(EkfCore & core, const Measurement & m, double t_init) const
{
  // 走行中に初期化し直したとき、v = 0 から出発して NDT に追いつくまでの過渡をなくす
  if (std::abs(t_init - m.time) > params_.timeout_init) {return false;}
  core.mutableState()(EkfCore::kVxy) = m.z(0);
  core.mutableState()(EkfCore::kOmegaYaw) = m.z(1);
  // 姿勢との相関は初期化直後なので 0 のまま
  core.mutableCovariance()(EkfCore::kVxy, EkfCore::kVxy) = m.r(0);
  core.mutableCovariance()(EkfCore::kOmegaYaw, EkfCore::kOmegaYaw) = m.r(1);
  return true;
}

std::vector<StageResult> OdomModel::update(EkfCore & core, const Measurement & m) const
{
  // h(x) = [v_xy, omega_yaw]。H は v_xy, omega_yaw の列だけが 1
  using Matrix2xN = Eigen::Matrix<double, 2, EkfCore::kStateSize>;
  Matrix2xN H = Matrix2xN::Zero();
  H(0, EkfCore::kVxy) = 1.0;
  H(1, EkfCore::kOmegaYaw) = 1.0;
  const EkfCore::StateVector & x = core.state();
  const Eigen::Vector2d z = m.z;
  const Eigen::Vector2d y = z - Eigen::Vector2d(x(EkfCore::kVxy), x(EkfCore::kOmegaYaw));
  const Eigen::Matrix2d R = Eigen::Vector2d(m.r).asDiagonal();

  return {gatedUpdate<2>(core, "v/omega", gate_index_, y, H, R, params_.gate_threshold)};
}

OdomNoise::OdomNoise(const OdomModel::Params & params)
: params_(params)
{
}

Eigen::Vector2d OdomNoise::variance(double t, double v, double omega)
{
  const Eigen::Vector2d velocity(v, omega);
  if (has_prev_ && t > prev_time_) {
    accel_ = (velocity - prev_velocity_) / (t - prev_time_);
  }
  has_prev_ = true;
  prev_time_ = t;
  prev_velocity_ = velocity;

  const Eigen::Vector4d & a = params_.alpha;
  const Eigen::Vector2d & tau = params_.tau;
  return Eigen::Vector2d(
    params_.r(0) + a(0) * v * v + a(1) * omega * omega + std::pow(tau(0) * accel_(0), 2),
    params_.r(1) + a(2) * v * v + a(3) * omega * omega + std::pow(tau(1) * accel_(1), 2));
}

}  // namespace ekf_localizer
