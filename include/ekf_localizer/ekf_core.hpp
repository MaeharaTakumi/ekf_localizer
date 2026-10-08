#ifndef EKF_LOCALIZER__EKF_CORE_HPP_
#define EKF_LOCALIZER__EKF_CORE_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>

#include <cmath>
#include <memory>
#include <vector>

namespace ekf_localizer
{

struct Measurement;

/// 角度を (-pi, pi] に正規化する
double normalizeAngle(double angle);
/// 回転行列 → [roll, pitch, yaw]（tf2 の getRPY と同じ ZYX 規約）
Eigen::Vector3d rpyFromRotation(const Eigen::Matrix3d & R);
/// [roll, pitch, yaw] → 回転行列（ZYX）
Eigen::Matrix3d rotationFromRpy(double roll, double pitch, double yaw);

/// EKF 本体（センサを知らない部分）
///
///   状態 x = [x, y, z, roll, pitch, yaw, v, omega]^T
///        … map → base の 6 自由度姿勢（ZYX）と、車体の前進速度・ヨー角速度
///   運動 車体 x 軸方向にだけ進む。水平は円弧の厳密積分（速度 v cos(pitch)）、z は -v sin(pitch)。
///        roll, pitch, v, omega はランダムウォーク
///   更新 一般形の kalmanUpdate だけを持つ。観測ごとの y, H, R, ゲートは観測モデルが組み立てる
///
///   履歴（LaggedEkf）の各時点でコピーして持つので、巻き戻しで一緒に戻すべき値もここに置く。
///     ゲートの状態  観測モデルが addGates() で確保した、連続棄却回数と直近の d²
///     初期化用の観測 初期化のときに使う観測（オドメトリの v, omega など）の最新の 1 つ
///
/// ROS に依存しない。時刻はすべて double（秒）。
class EkfCore
{
public:
  static constexpr int kN = 8;
  using StateVector = Eigen::Matrix<double, kN, 1>;
  using StateMatrix = Eigen::Matrix<double, kN, kN>;

  /// 状態の添字
  enum Index {kX = 0, kY, kZ, kRoll, kPitch, kYaw, kV, kOmega};

  struct Params
  {
    /// プロセスノイズ（スペクトル密度、状態と同じ並び）
    /// [x, y, z, roll, pitch, yaw] [m^2/s], [rad^2/s]、[v, omega] [m^2/s^3], [rad^2/s^3]
    StateVector q =
      (StateVector() << 1.0e-3, 1.0e-3, 2.0e-3, 5.0e-3, 5.0e-3, 1.0e-3, 0.05, 0.2).finished();
    /// 初期共分散の対角（分散、状態と同じ並び）
    StateVector p_init =
      (StateVector() << 2.0e-4, 2.0e-4, 2.0e-4, 2.0e-6, 2.0e-6, 2.0e-6, 1.0, 1.0).finished();
    /// 1 回の予測で進める dt の上限 [s]
    double max_predict_dt{0.2};
  };

  /// ゲート 1 つ分の状態（観測モデルの更新の段ごとに 1 つ）
  struct GateState
  {
    int reject_count{0};
    double last_d2{0.0};
  };

  EkfCore();
  explicit EkfCore(const Params & p);

  const Params & params() const {return prm_;}

  /// x0 の位置・姿勢・速度で初期化する。P = diag(p_init)、t_x = stamp、ゲートの状態は 0 に戻す
  void initialize(const StateVector & x0, double stamp);
  /// 未初期化に戻す（初期化用の観測は残し、次の初期化で使う）
  void reset();
  bool initialized() const {return initialized_;}

  /// t_x から stamp まで予測する。未初期化か dt <= 0 なら何もしない。
  /// dt が max_predict_dt を超える場合は予測だけ切り詰め、t_x は stamp まで進める。
  void predictTo(double stamp);

  /// 一般形の KF 更新。y = z - h(x) は角度成分を wrap して渡す。
  /// d² = yᵀ S⁻¹ y が gate 以下なら更新して true、棄却なら状態を変えず false
  template<int M>
  bool kalmanUpdate(
    const Eigen::Matrix<double, M, 1> & y, const Eigen::Matrix<double, M, kN> & H,
    const Eigen::Matrix<double, M, M> & R, double gate, double & d2_out);

  const StateVector & state() const {return x_;}
  const StateMatrix & covariance() const {return P_;}
  /// 観測モデル用（ロックアウトの再初期化など、一般形の更新で書けない変更）
  StateVector & mutableState() {return x_;}
  StateMatrix & mutableCovariance() {return P_;}

  /// 推定した base の姿勢 map → base（状態そのもの）
  Eigen::Affine3d basePose() const {return poseFromState(x_);}
  double velocity() const {return x_(kV);}
  double angularVelocity() const {return x_(kOmega);}
  /// 状態が表している時刻 t_x [s]（最後に予測・初期化した時刻）
  double lastStamp() const {return t_last_;}
  double lastDt() const {return last_dt_;}
  bool lastDtClamped() const {return last_dt_clamped_;}

  /// ゲートを n 個確保し、先頭の番号を返す（観測モデルの作成時に呼ぶ）
  int addGates(int n);
  GateState & gate(int i) {return gates_.at(i);}
  const GateState & gate(int i) const {return gates_.at(i);}

  /// 初期化用の観測
  void setInitAid(std::shared_ptr<const Measurement> m) {init_aid_ = std::move(m);}
  const std::shared_ptr<const Measurement> & initAid() const {return init_aid_;}

  /// 運動モデル f(x, dt)
  static StateVector propagate(const StateVector & x, double dt);
  /// 運動モデルのヤコビアン F = df/dx
  static StateMatrix transitionJacobian(const StateVector & x, double dt);
  /// 状態の先頭 6 成分を map → base の変換にする
  static Eigen::Affine3d poseFromState(const StateVector & x);
  /// 状態の角度成分（roll, pitch, yaw）を wrap する
  static void wrapAngles(StateVector & x);

private:
  StateVector x_{StateVector::Zero()};
  StateMatrix P_{StateMatrix::Identity()};
  double t_last_{0.0};
  double last_dt_{0.0};
  bool last_dt_clamped_{false};
  bool initialized_{false};
  std::vector<GateState> gates_;
  std::shared_ptr<const Measurement> init_aid_;
  Params prm_;
};

template<int M>
bool EkfCore::kalmanUpdate(
  const Eigen::Matrix<double, M, 1> & y, const Eigen::Matrix<double, M, kN> & H,
  const Eigen::Matrix<double, M, M> & R, double gate, double & d2_out)
{
  const Eigen::Matrix<double, M, M> S = H * P_ * H.transpose() + R;
  const Eigen::Matrix<double, M, M> S_inv = S.inverse();
  const double d2 = y.dot(S_inv * y);
  d2_out = d2;
  if (!std::isfinite(d2) || d2 > gate) {return false;}

  const Eigen::Matrix<double, kN, M> K = P_ * H.transpose() * S_inv;
  x_ += K * y;
  wrapAngles(x_);
  const StateMatrix IKH = StateMatrix::Identity() - K * H;
  P_ = IKH * P_ * IKH.transpose() + K * R * K.transpose();   // Joseph 形
  return true;
}

}  // namespace ekf_localizer

#endif  // EKF_LOCALIZER__EKF_CORE_HPP_
