#ifndef EKF_LOCALIZER__NDT_MODEL_HPP_
#define EKF_LOCALIZER__NDT_MODEL_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <optional>
#include <vector>

#include "ekf_localizer/ekf_core.hpp"
#include "ekf_localizer/observation_model.hpp"

namespace ekf_localizer
{

/// NDT の解（LiDAR の姿勢 map → LiDAR）の観測モデル
///
///   観測 z = LiDAR の [x, y, z, roll, pitch, yaw]
///   h(x)   T_B(x) · T_BL を [x, y, z, roll, pitch, yaw] にしたもの（T_BL は取付 base → LiDAR）
///   更新   [x, y, yaw] → z → roll → pitch の 4 段で、それぞれ独立にゲート判定する。
///          z, roll, pitch は lockout_count 回続けて棄却したら、その成分を観測値に合わせて再初期化する
///   初期化 T_B = T_L · T_BL⁻¹（v, omega は 0）
class NdtModel : public ObservationModel
{
public:
  using Vector6d = Eigen::Matrix<double, 6, 1>;
  using Matrix6xN = Eigen::Matrix<double, 6, EkfCore::kN>;

  struct Params
  {
    /// 観測ノイズ（LiDAR の [x, y, z, roll, pitch, yaw]）[m^2], [rad^2]
    Vector6d r = (Vector6d() << 1.0e-4, 1.0e-4, 1.0e-4, 1.0e-6, 1.0e-6, 1.0e-6).finished();
    /// [x, y, yaw] のゲート（既定 chi2(3, 0.99)）
    double gate_horizontal{11.34};
    /// z, roll, pitch それぞれのゲート（既定 chi2(1, 0.99)）
    double gate_1d{6.63};
    /// z, roll, pitch が連続でこの回数棄却したら観測値で再初期化する（0 以下で無効）
    int lockout_count{10};
  };

  /// 段の番号（StageResult の並び、ゲートの並び）
  enum Stage {kHorizontal = 0, kStageZ, kStageRoll, kStagePitch, kNumStages};

  /// mount は取付 base → LiDAR。core にゲートを 4 つ確保する
  NdtModel(const Params & p, const Eigen::Affine3d & mount, EkfCore & core);

  /// NDT の解（LiDAR の [x, y, z, roll, pitch, yaw]、t_z）を観測にする。観測ノイズは Params::r
  MeasurementPtr measurement(const Vector6d & z_lidar, double stamp) const;

  const char * name() const override {return "NDT";}
  std::optional<EkfCore::StateVector> initialState(const Measurement & m) const override;
  std::vector<StageResult> update(EkfCore & core, const Measurement & m) const override;

  const Params & params() const {return prm_;}
  /// 取付 base → LiDAR（T_BL）
  const Eigen::Affine3d & mountTransform() const {return mount_;}
  /// 状態 x から求めた LiDAR の姿勢 map → LiDAR（T_B · T_BL）
  Eigen::Affine3d lidarPose(const EkfCore::StateVector & x) const;
  /// h(x) = LiDAR の [x, y, z, roll, pitch, yaw]
  Vector6d observe(const EkfCore::StateVector & x) const;
  /// h のヤコビアン（中心差分。角度成分は差を wrap する）
  Matrix6xN observationJacobian(const EkfCore::StateVector & x) const;
  /// 段 s のゲートの状態
  const EkfCore::GateState & gate(const EkfCore & core, Stage s) const
  {
    return core.gate(first_gate_ + s);
  }

private:
  /// z の成分 idx（LiDAR の並び）を使った更新。ゲートを通れば更新して true
  template<int M>
  bool componentUpdate(
    EkfCore & core, const Vector6d & z, const Vector6d & r, const int (&idx)[M], double gate,
    double & d2_out) const;
  /// z, roll, pitch の 1 次元更新（ゲート・ロックアウト再初期化つき）
  StageResult scalarUpdate(
    EkfCore & core, const Vector6d & z, const Vector6d & r, Stage stage, int obs_idx) const;

  Params prm_;
  Eigen::Affine3d mount_;
  int first_gate_;
};

}  // namespace ekf_localizer

#endif  // EKF_LOCALIZER__NDT_MODEL_HPP_
