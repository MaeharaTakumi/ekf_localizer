#ifndef EKF_LOCALIZER__NDT_MODEL_HPP_
#define EKF_LOCALIZER__NDT_MODEL_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <optional>
#include <vector>

#include "ekf_localizer/ekf_core.hpp"
#include "ekf_localizer/observation_model.hpp"
#include "ekf_localizer/pose_utils.hpp"

namespace ekf_localizer
{

/// NDT の解（LiDAR の姿勢 map → LiDAR）の観測モデル
///
///   観測   NDT の LiDAR の姿勢 z_L を、取付で base の姿勢 z_B に直したもの（T_B = T_L · T_BL⁻¹、厳密）
///   h(x)   base の [x, y, z, roll, pitch, yaw]（状態の先頭 6 成分。H = [I 0]）
///   R      LiDAR の観測ノイズ R_ndt（対角）を z_L → z_B のヤコビアン J で伝播した J R_ndt Jᵀ（相関込みの 6x6）。
///          LiDAR は base から離れている（高さ約 1.1 m）ので、LiDAR の roll・pitch の誤差が base の x, y に回る
///   更新   6 成分をまとめて 1 回。ゲートは gate_threshold
///   初期化 z_B（v, omega は 0）
///   観測（Measurement）には LiDAR の z_L と R_ndt をそのまま持ち、更新のたびに z_B と R に直す
class NdtModel : public ObservationModel
{
public:
  using Vector6d = ekf_localizer::Vector6d;
  using Matrix6d = Eigen::Matrix<double, 6, 6>;
  using Matrix6xN = Eigen::Matrix<double, 6, EkfCore::kStateSize>;

  struct Params
  {
    /// 観測ノイズ（LiDAR の [x, y, z, roll, pitch, yaw]）[m^2], [rad^2]
    Vector6d r = (Vector6d() << 2.08e-4, 1.55e-4, 9.61e-5, 5.89e-5, 6.32e-5, 1.23e-5).finished();
    /// 6 成分のゲート（マハラノビス距離の 2 乗の上限）。1e5 は実質無効（χ²(6) の 99% は 16.81）
    double gate_threshold{1.0e5};
  };

  /// mount は取付 base → LiDAR。core にゲートを 1 つ確保する
  NdtModel(const Params & params, const Eigen::Affine3d & mount, EkfCore & core);

  /// NDT の解（LiDAR の [x, y, z, roll, pitch, yaw]、時刻 t_z）を観測にする。観測ノイズは Params::r
  MeasurementPtr measurement(const Vector6d & z_lidar, double t) const;

  const char * name() const override {return "NDT";}
  std::optional<EkfCore::StateVector> initialState(const Measurement & m) const override;
  std::vector<StageResult> update(EkfCore & core, const Measurement & m) const override;

  const Params & params() const {return params_;}
  /// 取付 base → LiDAR（T_BL）
  const Eigen::Affine3d & mountTransform() const {return mount_;}
  /// LiDAR の姿勢 [x, y, z, roll, pitch, yaw] → base の姿勢（T_L · T_BL⁻¹）
  Vector6d basePoseFromLidar(const Vector6d & z_lidar) const;
  /// base の姿勢の観測ノイズ J diag(r_lidar) Jᵀ（J は z_L → z_B のヤコビアン、中心差分）
  Matrix6d baseCovariance(const Vector6d & z_lidar, const Vector6d & r_lidar) const;
  /// 状態 x から求めた LiDAR の姿勢 map → LiDAR（T_B · T_BL）
  Eigen::Affine3d lidarPose(const EkfCore::StateVector & x) const;
  /// LiDAR の姿勢 [x, y, z, roll, pitch, yaw] の状態に対するヤコビアン（中心差分。/ekf_pose の共分散に使う）
  Matrix6xN lidarPoseJacobian(const EkfCore::StateVector & x) const;
  /// このモデルのゲートの状態
  const EkfCore::GateState & gateState(const EkfCore & core) const {return core.gateState(gate_index_);}

private:
  Params params_;
  Eigen::Affine3d mount_;
  /// core の中のゲートの番号
  int gate_index_;
};

}  // namespace ekf_localizer

#endif  // EKF_LOCALIZER__NDT_MODEL_HPP_
