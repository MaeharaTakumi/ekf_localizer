#ifndef EKF_LOCALIZER__VEHICLE_EKF_HPP_
#define EKF_LOCALIZER__VEHICLE_EKF_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <memory>

namespace ekf_localizer
{

/// 角度を (-pi, pi] に正規化する
double normalizeAngle(double angle);

/// base_link 基準の 8 状態 EKF（docs/ekf_design.md 5.3 節、docs/ekf_3d_plan.md）
///
///   状態 x = [x_b, y_b, z_b, roll_b, pitch_b, yaw_b, v, omega]^T
///        … map → base_link の 6 自由度姿勢（ZYX）と、車体の前進速度・ヨー角速度
///   運動 車体 x 軸方向にだけ進む。水平は円弧の厳密積分（速度 v cos(pitch)）、z は -v sin(pitch)。
///        roll, pitch, v, omega はランダムウォーク
///   観測 NDT の LiDAR 姿勢 h(x) = T_MB(x) · T_BL。取付オフセットは roll, pitch, yaw 全部で回す
///        更新は [x, y, yaw] → z → roll → pitch の逐次で、それぞれ独立にゲート判定する
///
///   **オドメトリは一切使わない。** v, omega は NDT 姿勢列から推定する（観測に加えるのは VehicleOdomEkf）。
///
/// ROS に依存しない。時刻はすべて double（秒）で扱う。
class VehicleEkf
{
public:
  static constexpr int kN = 8;
  using StateVector = Eigen::Matrix<double, kN, 1>;
  using StateMatrix = Eigen::Matrix<double, kN, kN>;
  using Vector6d = Eigen::Matrix<double, 6, 1>;
  using Matrix6xN = Eigen::Matrix<double, 6, kN>;

  /// 状態の添字
  enum Index {kX = 0, kY, kZ, kRoll, kPitch, kYaw, kV, kOmega};

  struct Params
  {
    /// プロセスノイズ（スペクトル密度、状態と同じ並び）
    /// [x, y, z, roll, pitch, yaw] [m^2/s], [rad^2/s]、[v, omega] [m^2/s^3], [rad^2/s^3]
    StateVector q =
      (StateVector() << 1.0e-3, 1.0e-3, 2.0e-3, 5.0e-3, 5.0e-3, 1.0e-3, 0.05, 0.2).finished();
    /// NDT 観測ノイズ（LiDAR 座標の [x, y, z, roll, pitch, yaw]）[m^2], [rad^2]
    Vector6d r_ndt =
      (Vector6d() << 1.0e-4, 1.0e-4, 1.0e-4, 1.0e-6, 1.0e-6, 1.0e-6).finished();

    /// base_link → LiDAR の取付並進 [m]
    double o_x{0.0};
    double o_y{0.0};
    double o_z{0.0};
    /// base_link → LiDAR の取付回転 [rad]
    double roll_o{0.0};
    double pitch_o{0.0};
    double yaw_o{0.0};

    /// 水平系 [x, y, yaw] のゲート（既定 chi2(3, 0.99)）
    double gate_horizontal{11.34};
    /// z, roll, pitch それぞれのゲート（既定 chi2(1, 0.99)）
    double gate_1d{6.63};
    /// z, roll, pitch が連続でこの回数棄却したら観測値で再初期化する（0 以下で無効）
    int lockout_count{10};
    /// 初期共分散の対角（分散、状態と同じ並び）
    StateVector p_init =
      (StateVector() << 2.0e-4, 2.0e-4, 2.0e-4, 2.0e-6, 2.0e-6, 2.0e-6, 1.0, 1.0).finished();
    /// 1 回の予測で進める dt の上限 [s]
    double max_predict_dt{0.2};
  };

  /// z, roll, pitch の 1 次元更新の結果
  enum class ScalarResult {kUpdated, kRejected, kReinitialized};

  /// update() の結果。各成分の判定を個別に返す
  struct UpdateResult
  {
    bool horizontal_accepted{false};
    ScalarResult z{ScalarResult::kRejected};
    ScalarResult roll{ScalarResult::kRejected};
    ScalarResult pitch{ScalarResult::kRejected};
  };

  /// z, roll, pitch の成分ごとのゲート状態
  struct ScalarGate
  {
    int reject_count{0};
    double last_d2{0.0};
  };

  virtual ~VehicleEkf() = default;

  /// 状態・ゲートの状態ごと複製する（LaggedEkf の履歴用）
  virtual std::unique_ptr<VehicleEkf> clone() const {return std::make_unique<VehicleEkf>(*this);}

  virtual void setParams(const Params & p) {prm_ = p;}
  const Params & params() const {return prm_;}

  /// 最初の NDT 観測（LiDAR の [x, y, z, roll, pitch, yaw]）で初期化する
  virtual void initialize(const Vector6d & z_lidar, double stamp);
  bool initialized() const {return initialized_;}
  virtual void reset();

  /// t_last_ から stamp まで予測する。dt <= 0 なら何もしない。
  /// dt が max_predict_dt を超える場合は予測だけ切り詰め、t_last_ は stamp まで進める。
  void predictTo(double stamp);
  /// NDT 観測（LiDAR の [x, y, z, roll, pitch, yaw]）。[x, y, yaw] と z, roll, pitch を独立に判定・更新する
  UpdateResult update(const Vector6d & z_lidar);

  const StateVector & state() const {return x_;}
  const StateMatrix & covariance() const {return P_;}
  const ScalarGate & zGate() const {return gate_z_;}
  const ScalarGate & rollGate() const {return gate_roll_;}
  const ScalarGate & pitchGate() const {return gate_pitch_;}

  /// 推定した LiDAR 姿勢 map → LiDAR（= basePose() * T_BL。NDT の init_guess 用）
  Eigen::Affine3d lidarPose() const;
  /// 推定した base_link 姿勢 map → base_link（状態そのもの）
  Eigen::Affine3d basePose() const;
  /// base_link → LiDAR の取付変換 T_BL
  Eigen::Affine3d mountTransform() const;

  double velocity() const {return x_(kV);}
  double angularVelocity() const {return x_(kOmega);}
  /// 状態が表している時刻 t_last [s]（最後に予測・初期化した時刻）
  double lastStamp() const {return t_last_;}
  double lastDt() const {return last_dt_;}
  bool lastDtClamped() const {return last_dt_clamped_;}
  int horizontalRejectCount() const {return reject_count_;}
  double lastHorizontalMahalanobis() const {return last_d2_;}

  /// 観測モデル h(x) = LiDAR の [x, y, z, roll, pitch, yaw]（テスト用に公開）
  Vector6d observe(const StateVector & x) const;
  /// h のヤコビアン（中心差分。角度成分は差を wrap する）（テスト用に公開）
  Matrix6xN observationJacobian(const StateVector & x) const;
  /// 運動モデル f(x, dt)（テスト用に公開）
  static StateVector propagate(const StateVector & x, double dt);
  /// 運動モデルのヤコビアン F = df/dx（テスト用に公開）
  static StateMatrix transitionJacobian(const StateVector & x, double dt);
  /// 状態の先頭 6 成分を map → base_link の変換にする
  static Eigen::Affine3d poseFromState(const StateVector & x);

protected:
  /// 観測 z_lidar の成分 idx（LiDAR の並び）を使った逐次更新。
  /// ゲートを通れば更新して true、棄却なら状態を変えず false
  template<int M>
  bool sequentialUpdate(
    const Vector6d & z_lidar, const int (&idx)[M], double gate, double & d2_out);
  /// z, roll, pitch の 1 次元更新（ゲート・ロックアウト再初期化つき）
  ScalarResult scalarUpdate(const Vector6d & z_lidar, int obs_idx, ScalarGate & gate);

  StateVector x_{StateVector::Zero()};
  StateMatrix P_{StateMatrix::Identity()};
  ScalarGate gate_z_;
  ScalarGate gate_roll_;
  ScalarGate gate_pitch_;
  double t_last_{0.0};
  double last_dt_{0.0};
  bool last_dt_clamped_{false};
  bool initialized_{false};
  int reject_count_{0};
  double last_d2_{0.0};
  Params prm_;
};

}  // namespace ekf_localizer

#endif  // EKF_LOCALIZER__VEHICLE_EKF_HPP_
