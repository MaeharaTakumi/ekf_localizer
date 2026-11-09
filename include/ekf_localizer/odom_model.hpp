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
    /// 観測ノイズ [v, omega] の分散 [m^2/s^2], [rad^2/s^2]（速さ・加速度によらない分）
    Eigen::Vector2d r{6.08e-3, 9.92e-4};
    /// 速さに応じた分の係数 [v^2 -> R_v, omega^2 -> R_v, v^2 -> R_omega, omega^2 -> R_omega]
    Eigen::Vector4d alpha{Eigen::Vector4d::Zero()};
    /// 加速度に応じた分の時定数 [tau_v, tau_omega] [s]
    Eigen::Vector2d tau{Eigen::Vector2d::Zero()};
    /// ゲート（既定 chi2(2, 0.99)）
    double gate{9.21};
    /// 初期化の時刻からこの秒数以内のオドメトリがあれば、v, omega をその値で初期化する
    double timeout_init{0.2};
  };

  /// core にゲートを 1 つ確保する
  OdomModel(const Params & p, EkfCore & core);

  /// [v, omega]（t_z）を観測にする。r は観測ノイズの分散（OdomNoise::update の値）
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

/// オドメトリの観測ノイズ（分散）を、速さと加速度から計算する
///
///   R_v     = r[0] + alpha[0] v^2 + alpha[1] omega^2 + (tau[0] a)^2
///   R_omega = r[1] + alpha[2] v^2 + alpha[3] omega^2 + (tau[1] alpha_z)^2
///   a = dv/dt、alpha_z = domega/dt は直前のメッセージとの差。stamp が進まなければ前の値のまま。最初は 0
class OdomNoise
{
public:
  explicit OdomNoise(const OdomModel::Params & p);

  /// 時刻 stamp の v, omega から [R_v, R_omega] を計算する
  Eigen::Vector2d update(double stamp, double v, double omega);

private:
  OdomModel::Params prm_;
  bool has_prev_{false};
  double prev_stamp_{0.0};
  Eigen::Vector2d prev_{Eigen::Vector2d::Zero()};
  Eigen::Vector2d accel_{Eigen::Vector2d::Zero()};
};

}  // namespace ekf_localizer

#endif  // EKF_LOCALIZER__ODOM_MODEL_HPP_
