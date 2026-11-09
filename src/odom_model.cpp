#include "ekf_localizer/odom_model.hpp"

#include <cmath>

namespace ekf_localizer
{

OdomModel::OdomModel(const Params & p, EkfCore & core)
: prm_(p), gate_(core.addGates(1))
{
}

MeasurementPtr OdomModel::measurement(
  double v, double omega, const Eigen::Vector2d & r, double stamp) const
{
  return makeMeasurement(stamp, Eigen::Vector2d(v, omega), r);
}

bool OdomModel::aidInitialization(EkfCore & core, const Measurement & m, double t_init) const
{
  // 走行中に初期化し直したとき、v = 0 から出発して NDT に追いつくまでの過渡をなくす
  if (std::abs(t_init - m.stamp) > prm_.timeout_init) {return false;}
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
  Eigen::Matrix<double, 2, EkfCore::kN> H = Eigen::Matrix<double, 2, EkfCore::kN>::Zero();
  H(0, EkfCore::kVxy) = 1.0;
  H(1, EkfCore::kOmegaYaw) = 1.0;
  const EkfCore::StateVector & x = core.state();
  const Eigen::Vector2d z = m.z;
  const Eigen::Vector2d y = z - Eigen::Vector2d(x(EkfCore::kVxy), x(EkfCore::kOmegaYaw));
  const Eigen::Matrix2d R = Eigen::Vector2d(m.r).asDiagonal();

  EkfCore::GateState & gate = core.gate(gate_);
  StageResult res;
  res.name = "v/omega";
  if (core.kalmanUpdate<2>(y, H, R, prm_.gate, gate.last_d2)) {
    gate.reject_count = 0;
    res.status = StageResult::Status::kUpdated;
  } else {
    ++gate.reject_count;
    res.status = StageResult::Status::kRejected;
  }
  res.d2 = gate.last_d2;
  res.reject_count = gate.reject_count;
  return {res};
}

OdomNoise::OdomNoise(const OdomModel::Params & p)
: prm_(p)
{
}

Eigen::Vector2d OdomNoise::update(double stamp, double v, double omega)
{
  const Eigen::Vector2d now(v, omega);
  if (has_prev_ && stamp > prev_stamp_) {
    accel_ = (now - prev_) / (stamp - prev_stamp_);
  }
  has_prev_ = true;
  prev_stamp_ = stamp;
  prev_ = now;

  const Eigen::Vector4d & a = prm_.alpha;
  return Eigen::Vector2d(
    prm_.r(0) + a(0) * v * v + a(1) * omega * omega + std::pow(prm_.tau(0) * accel_(0), 2),
    prm_.r(1) + a(2) * v * v + a(3) * omega * omega + std::pow(prm_.tau(1) * accel_(1), 2));
}

}  // namespace ekf_localizer
