#ifndef EKF_LOCALIZER__ODOM_MODEL_HPP_
#define EKF_LOCALIZER__ODOM_MODEL_HPP_

#include <Eigen/Core>

#include <vector>

#include "ekf_localizer/ekf_core.hpp"
#include "ekf_localizer/observation_model.hpp"

namespace ekf_localizer
{

/// 車輪オドメトリ（前進速度・ヨー角速度）の観測モデル
///
///   観測 z = [v_o, omega_o]（base 座標なので取付の変換はない）
///   h(x)   [v, omega]
///   更新   1 段。ゲートで棄却したら捨てる（再初期化しない）。**符号の誤りはゲートでは検出できない**
///   初期化 自分では初期化できない。最新の値を覚えておき、NDT で初期化した時刻から
///          odom_timeout_init 以内なら、v, omega をその値から始める
class OdomModel : public ObservationModel
{
public:
  struct Params
  {
    /// 観測ノイズ [v, omega] の分散 [m^2/s^2], [rad^2/s^2]
    Eigen::Vector2d r{2.5e-3, 5.0e-3};
    /// ゲート（既定 chi2(2, 0.99)）
    double gate{9.21};
    /// 初期化の時刻からこの秒数以内のオドメトリがあれば、v, omega をその値で初期化する
    double timeout_init{0.2};
  };

  /// core にゲートを 1 つ確保する
  OdomModel(const Params & p, EkfCore & core);

  /// [v, omega]（t_z）を観測にする。r は観測ノイズの分散（Params::r かメッセージの値）
  MeasurementPtr measurement(double v, double omega, const Eigen::Vector2d & r, double stamp) const;

  const char * name() const override {return "Odometry";}
  bool aidsInitialization() const override {return true;}
  bool aidInitialization(EkfCore & core, const Measurement & m, double t_init) const override;
  std::vector<StageResult> update(EkfCore & core, const Measurement & m) const override;

  const Params & params() const {return prm_;}
  const EkfCore::GateState & gate(const EkfCore & core) const {return core.gate(gate_);}

private:
  Params prm_;
  int gate_;
};

}  // namespace ekf_localizer

#endif  // EKF_LOCALIZER__ODOM_MODEL_HPP_
