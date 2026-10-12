#include "ekf_localizer/ndt_model.hpp"

#include <cmath>

namespace ekf_localizer
{

namespace
{

/// 姿勢の行列 → [x, y, z, roll, pitch, yaw]
NdtModel::Vector6d poseVector(const Eigen::Affine3d & T)
{
  NdtModel::Vector6d v;
  v.head<3>() = T.translation();
  v.tail<3>() = rpyFromRotation(T.linear());
  return v;
}

/// [x, y, z, roll, pitch, yaw] → 姿勢の行列
Eigen::Affine3d poseMatrix(const NdtModel::Vector6d & v)
{
  Eigen::Affine3d T = Eigen::Affine3d::Identity();
  T.linear() = rotationFromRpy(v(3), v(4), v(5));
  T.translation() = v.head<3>();
  return T;
}

}  // namespace

NdtModel::NdtModel(const Params & p, const Eigen::Affine3d & mount, EkfCore & core)
: prm_(p), mount_(mount), gate_(core.addGates(1))
{
}

MeasurementPtr NdtModel::measurement(const Vector6d & z_lidar, double stamp) const
{
  return makeMeasurement(stamp, z_lidar, prm_.r);
}

// ---------------------------------------------------------------------------
// LiDAR の姿勢 ⇔ base の姿勢
// ---------------------------------------------------------------------------

NdtModel::Vector6d NdtModel::basePoseFromLidar(const Vector6d & z_lidar) const
{
  // T_B = T_L · T_BL⁻¹（厳密）
  return poseVector(poseMatrix(z_lidar) * mount_.inverse());
}

NdtModel::Matrix6d NdtModel::baseCovariance(const Vector6d & z_lidar, const Vector6d & r_lidar) const
{
  // z_L → z_B のヤコビアン J を中心差分で求め、J diag(r) Jᵀ にする（1 次の伝播）。
  // 位置の誤差はそのまま、姿勢の誤差は取付のレバーアーム分だけ base の位置に回るので、位置と姿勢に相関が出る
  constexpr double kStep = 1.0e-6;
  Matrix6d J;
  for (int j = 0; j < 6; ++j) {
    Vector6d zp = z_lidar;
    Vector6d zm = z_lidar;
    zp(j) += kStep;
    zm(j) -= kStep;
    Vector6d d = basePoseFromLidar(zp) - basePoseFromLidar(zm);
    for (int i = 3; i < 6; ++i) {d(i) = normalizeAngle(d(i));}
    J.col(j) = d / (2.0 * kStep);
  }
  const Matrix6d R = J * r_lidar.asDiagonal() * J.transpose();
  return 0.5 * (R + R.transpose());
}

Eigen::Affine3d NdtModel::lidarPose(const EkfCore::StateVector & x) const
{
  // TF の map → base → LiDAR と厳密に一致する
  return EkfCore::poseFromState(x) * mount_;
}

NdtModel::Matrix6xN NdtModel::lidarPoseJacobian(const EkfCore::StateVector & x) const
{
  // RPY 抽出が取付回転を含むと解析式が長くなるので中心差分で求める。LiDAR の姿勢は v, omega に依存しない
  constexpr double kStep = 1.0e-6;
  Matrix6xN Jx = Matrix6xN::Zero();
  for (int j = 0; j < 6; ++j) {
    EkfCore::StateVector xp = x;
    EkfCore::StateVector xm = x;
    xp(j) += kStep;
    xm(j) -= kStep;
    Vector6d d = poseVector(lidarPose(xp)) - poseVector(lidarPose(xm));
    for (int i = 3; i < 6; ++i) {d(i) = normalizeAngle(d(i));}
    Jx.col(j) = d / (2.0 * kStep);
  }
  return Jx;
}

// ---------------------------------------------------------------------------
// 初期化・更新
// ---------------------------------------------------------------------------

std::optional<EkfCore::StateVector> NdtModel::initialState(const Measurement & m) const
{
  EkfCore::StateVector x0 = EkfCore::StateVector::Zero();
  x0.head<6>() = basePoseFromLidar(m.z);
  return x0;
}

std::vector<StageResult> NdtModel::update(EkfCore & core, const Measurement & m) const
{
  const Vector6d z_lidar = m.z;
  const Vector6d r_lidar = m.r;

  // base の姿勢の観測：残差 y = z_B - x[0:6]、H = [I 0]、R = J R_ndt Jᵀ
  Vector6d y = basePoseFromLidar(z_lidar) - core.state().head<6>();
  for (int i = 3; i < 6; ++i) {y(i) = normalizeAngle(y(i));}
  Matrix6xN H = Matrix6xN::Zero();
  H.leftCols<6>().setIdentity();
  const Matrix6d R = baseCovariance(z_lidar, r_lidar);

  EkfCore::GateState & gate = core.gate(gate_);
  StageResult res;
  res.name = "pose";
  if (core.kalmanUpdate<6>(y, H, R, prm_.gate, gate.last_d2)) {
    gate.reject_count = 0;
    res.status = StageResult::Status::kUpdated;
  } else {
    // 再初期化はしない（間違って収束した NDT の解に飛び移る危険があるため）
    ++gate.reject_count;
    res.status = StageResult::Status::kRejected;
  }
  res.d2 = gate.last_d2;
  res.reject_count = gate.reject_count;
  return {res};
}

}  // namespace ekf_localizer
