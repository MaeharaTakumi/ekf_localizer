#include "ekf_localizer/ndt_model.hpp"

#include <cmath>

namespace ekf_localizer
{

namespace
{

/// 姿勢を返す関数 f の、引数 p の先頭 6 成分に対するヤコビアン（中心差分。角度の差は wrap する）。
/// 残りの列は 0
template<int N, class F>
Eigen::Matrix<double, 6, N> poseJacobian(const Eigen::Matrix<double, N, 1> & p, F f)
{
  constexpr double kStep = 1.0e-6;
  Eigen::Matrix<double, 6, N> J = Eigen::Matrix<double, 6, N>::Zero();
  for (int j = 0; j < 6; ++j) {
    Eigen::Matrix<double, N, 1> pp = p;
    Eigen::Matrix<double, N, 1> pm = p;
    pp(j) += kStep;
    pm(j) -= kStep;
    Vector6d d = f(pp) - f(pm);
    wrapPoseAngles(d);
    J.col(j) = d / (2.0 * kStep);
  }
  return J;
}

}  // namespace

NdtModel::NdtModel(const Params & params, const Eigen::Affine3d & mount, EkfCore & core)
: params_(params), mount_(mount), gate_index_(core.addGates(1))
{
}

MeasurementPtr NdtModel::measurement(const Vector6d & z_lidar, double t) const
{
  return makeMeasurement(t, z_lidar, params_.r);
}

// ---------------------------------------------------------------------------
// LiDAR の姿勢 ⇔ base の姿勢
// ---------------------------------------------------------------------------

NdtModel::Vector6d NdtModel::basePoseFromLidar(const Vector6d & z_lidar) const
{
  // T_B = T_L · T_BL⁻¹（厳密）
  return poseToVector(poseFromVector(z_lidar) * mount_.inverse());
}

NdtModel::Matrix6d NdtModel::baseCovariance(const Vector6d & z_lidar, const Vector6d & r_lidar) const
{
  // z_L → z_B のヤコビアン J を中心差分で求め、J diag(r) Jᵀ にする（1 次の伝播）。
  // 位置の誤差はそのまま、姿勢の誤差は取付のレバーアーム分だけ base の位置に回るので、位置と姿勢に相関が出る
  const Matrix6d J = poseJacobian(
    z_lidar, [this](const Vector6d & z) {return basePoseFromLidar(z);});
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
  return poseJacobian(
    x, [this](const EkfCore::StateVector & xs) {return poseToVector(lidarPose(xs));});
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
  wrapPoseAngles(y);
  Matrix6xN H = Matrix6xN::Zero();
  H.leftCols<6>().setIdentity();
  const Matrix6d R = baseCovariance(z_lidar, r_lidar);

  // 棄却しても再初期化はしない（間違って収束した NDT の解に飛び移る危険があるため）
  return {gatedUpdate<6>(core, "pose", gate_index_, y, H, R, params_.gate_threshold)};
}

}  // namespace ekf_localizer
