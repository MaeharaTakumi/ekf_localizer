// ekf_localizer の観測モデルのテストで共用する部品（ROS 非依存）
#ifndef EKF_LOCALIZER__TEST_HELPERS_HPP_
#define EKF_LOCALIZER__TEST_HELPERS_HPP_

#include <Eigen/Eigenvalues>

#include <cmath>
#include <memory>
#include <vector>

#include "ekf_localizer/ekf_core.hpp"
#include "ekf_localizer/ndt_model.hpp"
#include "ekf_localizer/observation_model.hpp"
#include "ekf_localizer/odom_model.hpp"

namespace test_helpers
{

using ekf_localizer::applyMeasurement;
using ekf_localizer::ApplyResult;
using ekf_localizer::EkfCore;
using ekf_localizer::NdtModel;
using ekf_localizer::normalizeAngle;
using ekf_localizer::OdomModel;
using VS = EkfCore::StateVector;
using MS = EkfCore::StateMatrix;
using V6 = NdtModel::Vector6d;
using SR = ekf_localizer::StageResult::Status;

inline Eigen::Matrix3d rotRpy(double r, double p, double y)
{
  return (Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX())).toRotationMatrix();
}

inline Eigen::Affine3d makePose(double x, double y, double z, double r, double p, double yaw)
{
  Eigen::Affine3d T = Eigen::Affine3d::Identity();
  T.linear() = rotRpy(r, p, yaw);
  T.translation() = Eigen::Vector3d(x, y, z);
  return T;
}

/// テスト用のパラメータ一式（取付 base → LiDAR も含む）
struct TestParams
{
  EkfCore::Params core;
  NdtModel::Params ndt;
  OdomModel::Params odom;
  double o_x{0.0}, o_y{0.0}, o_z{0.0}, roll_o{0.0}, pitch_o{0.0}, yaw_o{0.0};

  Eigen::Affine3d mount() const {return makePose(o_x, o_y, o_z, roll_o, pitch_o, yaw_o);}
};

inline TestParams P0()
{
  TestParams p;
  p.odom.r = Eigen::Vector2d(2.5e-3, 5.0e-3);  // シミュレーションのオドメトリのノイズ（σ 0.05 / 0.07）に合わせる
  p.o_x = -0.2; p.o_y = 0.05; p.o_z = 1.2; p.yaw_o = -0.034; p.roll_o = 0.01; p.pitch_o = 0.02;
  // v_z, omega_roll, omega_pitch は Q・P_init とも 0（0 のまま動かない）
  p.core.q << 3e-2, 3e-2, 3e-2, 5e-2, 5e-2, 5e-2, 0.05, 0.0, 0.0, 0.0, 0.1;
  p.ndt.r << 1e-2, 1e-2, 1e-2, 1e-3, 1e-3, 5e-3;
  p.core.p_init << 2 * p.ndt.r, 1.0, 0.0, 0.0, 0.0, 1.0;
  // ゲートは χ² の 99%（ノードの既定 1e5 は実質無効なので、ゲートを確かめるテストのために効かせる）
  p.ndt.gate_threshold = 16.81;
  p.odom.gate_threshold = 9.21;
  return p;
}

/// EKF 本体と観測モデルの組（odom が null なら NDT だけ）
struct Rig
{
  EkfCore core;
  std::shared_ptr<NdtModel> ndt;
  std::shared_ptr<OdomModel> odom;

  /// NDT の解を適用する（未初期化なら初期化、初期化済みなら予測して更新）
  ApplyResult addNdt(const V6 & z, double t) {return applyMeasurement(core, ndt->measurement(z, t));}
  /// オドメトリを適用する（未初期化なら覚えるだけ、初期化済みなら予測して更新）
  ApplyResult addOdom(double v, double w, const Eigen::Vector2d & r, double t)
  {
    return applyMeasurement(core, odom->measurement(v, w, r, t));
  }
  /// 予測してから NDT で更新する（初期化済みのとき）。段ごとの結果を返す
  std::vector<ekf_localizer::StageResult> updateNdt(const V6 & z, double t)
  {
    return addNdt(z, t).stages;
  }
};

inline Rig makeRig(const TestParams & p, bool use_odom)
{
  Rig rig{EkfCore(p.core), nullptr, nullptr};
  rig.ndt = std::make_shared<NdtModel>(p.ndt, p.mount(), rig.core);
  if (use_odom) {rig.odom = std::make_shared<OdomModel>(p.odom, rig.core);}
  return rig;
}

inline bool psdOK(const Eigen::MatrixXd & P)
{
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(P);
  return P.allFinite() && (P - P.transpose()).cwiseAbs().maxCoeff() < 1e-9 &&
         es.eigenvalues().minCoeff() > -1e-12;
}

/// base の真値から LiDAR 観測を作る（水平面）
inline V6 lidarObs(const TestParams & p, double xb, double yb, double psib)
{
  const double c = std::cos(psib), s = std::sin(psib);
  V6 z;
  z << xb + p.o_x * c - p.o_y * s, yb + p.o_x * s + p.o_y * c, p.o_z, p.roll_o, p.pitch_o,
    normalizeAngle(psib + p.yaw_o);
  return z;
}

/// 回転行列 → [roll, pitch, yaw]（ZYX、tf2 の getRPY と同じ）
inline Eigen::Vector3d rpyOf(const Eigen::Matrix3d & R)
{
  return Eigen::Vector3d(
    std::atan2(R(2, 1), R(2, 2)), std::asin(-R(2, 0)), std::atan2(R(1, 0), R(0, 0)));
}

/// 姿勢 → [x, y, z, roll, pitch, yaw]
inline V6 poseVec(const Eigen::Affine3d & T)
{
  V6 z;
  z << T.translation(), rpyOf(T.linear());
  return z;
}

/// base の 6 自由度の真値から LiDAR 観測を作る（T_L = T_B · T_BL）
inline V6 lidarObs3d(const NdtModel & ndt, const Eigen::Affine3d & T_mb)
{
  return poseVec(T_mb * ndt.mountTransform());
}

/// 推定した LiDAR の姿勢
inline Eigen::Affine3d lidarPose(const Rig & rig) {return rig.ndt->lidarPose(rig.core.state());}

}  // namespace test_helpers

#endif  // EKF_LOCALIZER__TEST_HELPERS_HPP_
