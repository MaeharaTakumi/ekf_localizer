#include "ekf_localizer/ndt_model.hpp"

#include <cmath>

namespace ekf_localizer
{

namespace
{

/// 観測（LiDAR の [x, y, z, roll, pitch, yaw]）の成分が角度か
bool isAngleObs(int i) {return i >= 3;}

const char * const kStageNames[NdtModel::kNumStages] = {"x/y/yaw", "z", "roll", "pitch"};

}  // namespace

NdtModel::NdtModel(const Params & p, const Eigen::Affine3d & mount, EkfCore & core)
: prm_(p), mount_(mount), first_gate_(core.addGates(kNumStages))
{
}

MeasurementPtr NdtModel::measurement(const Vector6d & z_lidar, double stamp) const
{
  return makeMeasurement(stamp, z_lidar, prm_.r);
}

// ---------------------------------------------------------------------------
// 観測モデル
// ---------------------------------------------------------------------------

Eigen::Affine3d NdtModel::lidarPose(const EkfCore::StateVector & x) const
{
  // TF の map → base → LiDAR と厳密に一致する
  return EkfCore::poseFromState(x) * mount_;
}

NdtModel::Vector6d NdtModel::observe(const EkfCore::StateVector & x) const
{
  const Eigen::Affine3d T_ml = lidarPose(x);
  Vector6d h;
  h.head<3>() = T_ml.translation();
  h.tail<3>() = rpyFromRotation(T_ml.linear());
  return h;
}

NdtModel::Matrix6xN NdtModel::observationJacobian(const EkfCore::StateVector & x) const
{
  // RPY 抽出が取付回転を含むと解析式が長くなるので中心差分で求める。h は v, omega に依存しない
  constexpr double kStep = 1.0e-6;
  Matrix6xN H = Matrix6xN::Zero();
  for (int j = 0; j < 6; ++j) {
    EkfCore::StateVector xp = x;
    EkfCore::StateVector xm = x;
    xp(j) += kStep;
    xm(j) -= kStep;
    Vector6d d = observe(xp) - observe(xm);
    for (int i = 3; i < 6; ++i) {d(i) = normalizeAngle(d(i));}
    H.col(j) = d / (2.0 * kStep);
  }
  return H;
}

// ---------------------------------------------------------------------------
// 初期化
// ---------------------------------------------------------------------------

std::optional<EkfCore::StateVector> NdtModel::initialState(const Measurement & m) const
{
  // LiDAR 姿勢 → base 姿勢（T_B = T_L · T_BL^-1、厳密）
  const Vector6d z = m.z;
  Eigen::Affine3d T_ml = Eigen::Affine3d::Identity();
  T_ml.linear() = rotationFromRpy(z(3), z(4), z(5));
  T_ml.translation() = z.head<3>();
  const Eigen::Affine3d T_mb = T_ml * mount_.inverse();

  EkfCore::StateVector x0 = EkfCore::StateVector::Zero();
  x0.head<3>() = T_mb.translation();
  x0.segment<3>(EkfCore::kRoll) = rpyFromRotation(T_mb.linear());
  return x0;
}

// ---------------------------------------------------------------------------
// 更新
// ---------------------------------------------------------------------------

template<int M>
bool NdtModel::componentUpdate(
  EkfCore & core, const Vector6d & z, const Vector6d & r, const int (&idx)[M], double gate,
  double & d2_out) const
{
  const Vector6d h = observe(core.state());
  const Matrix6xN H_full = observationJacobian(core.state());

  Eigen::Matrix<double, M, 1> y;
  Eigen::Matrix<double, M, EkfCore::kN> H;
  Eigen::Matrix<double, M, M> R = Eigen::Matrix<double, M, M>::Zero();
  for (int i = 0; i < M; ++i) {
    y(i) = z(idx[i]) - h(idx[i]);
    if (isAngleObs(idx[i])) {y(i) = normalizeAngle(y(i));}
    H.row(i) = H_full.row(idx[i]);
    R(i, i) = r(idx[i]);
  }
  return core.kalmanUpdate<M>(y, H, R, gate, d2_out);
}

StageResult NdtModel::scalarUpdate(
  EkfCore & core, const Vector6d & z, const Vector6d & r, Stage stage, int obs_idx) const
{
  EkfCore::GateState & gate = core.gate(first_gate_ + stage);
  StageResult res;
  res.name = kStageNames[stage];

  const int idx[1] = {obs_idx};
  const bool accepted = componentUpdate<1>(core, z, r, idx, prm_.gate_1d, gate.last_d2);
  res.d2 = gate.last_d2;
  if (accepted) {
    gate.reject_count = 0;
    res.status = StageResult::Status::kUpdated;
    return res;
  }

  ++gate.reject_count;
  res.reject_count = gate.reject_count;
  if (prm_.lockout_count <= 0 || gate.reject_count < prm_.lockout_count) {
    res.status = StageResult::Status::kRejected;
    return res;
  }

  // ロックアウト：真値が急に変わって古い値に張り付いたとみなし、観測値から出直す。
  // LiDAR の z, roll, pitch（観測の添字 2, 3, 4）には base の同じ添字の状態がほぼ 1 対 1 で効くので、
  // その状態だけを動かして観測に合わせる（ニュートン法で数回）。他成分との相関は捨てる
  EkfCore::StateVector & x = core.mutableState();
  EkfCore::StateMatrix & P = core.mutableCovariance();
  const int s = obs_idx;
  double h_ii = 1.0;
  for (int iter = 0; iter < 3; ++iter) {
    double y = z(obs_idx) - observe(x)(obs_idx);
    if (isAngleObs(obs_idx)) {y = normalizeAngle(y);}
    h_ii = observationJacobian(x)(obs_idx, s);
    if (std::abs(h_ii) < 1.0e-3) {break;}   // 取付が極端で 1 対 1 でない場合は諦める
    x(s) += y / h_ii;
    EkfCore::wrapAngles(x);
  }
  P.row(s).setZero();
  P.col(s).setZero();
  P(s, s) = 2.0 * r(obs_idx) / (h_ii * h_ii);
  gate.reject_count = 0;
  res.reject_count = 0;
  res.status = StageResult::Status::kReinitialized;
  res.reinitialized_to = z(obs_idx);
  return res;
}

std::vector<StageResult> NdtModel::update(EkfCore & core, const Measurement & m) const
{
  const Vector6d z = m.z;
  const Vector6d r = m.r;
  std::vector<StageResult> res(kNumStages);

  // ---- 水平系 [x_L, y_L, yaw_L] ----
  EkfCore::GateState & gate = core.gate(first_gate_ + kHorizontal);
  const int horizontal[3] = {0, 1, 5};
  StageResult & h = res[kHorizontal];
  h.name = kStageNames[kHorizontal];
  if (componentUpdate<3>(core, z, r, horizontal, prm_.gate_horizontal, gate.last_d2)) {
    gate.reject_count = 0;
    h.status = StageResult::Status::kUpdated;
  } else {
    // 水平系は再初期化しない（誤収束した解に飛び移る危険があるため）
    ++gate.reject_count;
    h.status = StageResult::Status::kRejected;
  }
  h.d2 = gate.last_d2;
  h.reject_count = gate.reject_count;

  // ---- z, roll, pitch：それぞれ独立に判定する（水平系の結果に依存しない）----
  res[kStageZ] = scalarUpdate(core, z, r, kStageZ, 2);
  res[kStageRoll] = scalarUpdate(core, z, r, kStageRoll, 3);
  res[kStagePitch] = scalarUpdate(core, z, r, kStagePitch, 4);
  return res;
}

}  // namespace ekf_localizer
