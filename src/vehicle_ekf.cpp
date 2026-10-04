#include "ekf_localizer/vehicle_ekf.hpp"

#include <Eigen/LU>

#include <algorithm>
#include <cmath>

namespace ekf_localizer
{

double normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

namespace
{

/// sinc(u) = sin(u)/u と、その導関数。u → 0 では級数で評価する
void sincAndDerivative(double u, double & k, double & dk)
{
  if (std::abs(u) < 1.0e-3) {
    const double u2 = u * u;
    k = 1.0 - u2 / 6.0 + u2 * u2 / 120.0;
    dk = -u / 3.0 + u * u2 / 30.0;
  } else {
    const double s = std::sin(u);
    const double c = std::cos(u);
    k = s / u;
    dk = (u * c - s) / (u * u);
  }
}

/// 回転行列 → [roll, pitch, yaw]（tf2 の getRPY と同じ ZYX 規約）
Eigen::Vector3d rpyFromRotation(const Eigen::Matrix3d & R)
{
  return Eigen::Vector3d(
    std::atan2(R(2, 1), R(2, 2)),
    std::asin(std::max(-1.0, std::min(1.0, -R(2, 0)))),
    std::atan2(R(1, 0), R(0, 0)));
}

/// [roll, pitch, yaw] → 回転行列（ZYX）
Eigen::Matrix3d rotationFromRpy(double roll, double pitch, double yaw)
{
  return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
}

/// 状態の角度成分（roll, pitch, yaw）を wrap する
void wrapStateAngles(VehicleEkf::Vector8d & x)
{
  x(VehicleEkf::kRoll) = normalizeAngle(x(VehicleEkf::kRoll));
  x(VehicleEkf::kPitch) = normalizeAngle(x(VehicleEkf::kPitch));
  x(VehicleEkf::kYaw) = normalizeAngle(x(VehicleEkf::kYaw));
}

/// 観測（LiDAR の [x, y, z, roll, pitch, yaw]）の成分が角度か
bool isAngleObs(int i) {return i >= 3;}

}  // namespace

// ---------------------------------------------------------------------------
// モデル
// ---------------------------------------------------------------------------

Eigen::Affine3d VehicleEkf::poseFromState(const Vector8d & x)
{
  Eigen::Affine3d T = Eigen::Affine3d::Identity();
  T.linear() = rotationFromRpy(x(kRoll), x(kPitch), x(kYaw));
  T.translation() = x.head<3>();
  return T;
}

VehicleEkf::Vector8d VehicleEkf::propagate(const Vector8d & x, double dt)
{
  // 車体 x 軸方向の速度 v を map に回すと (v cos(pitch) [cos yaw, sin yaw], -v sin(pitch))。
  // 水平は円弧の厳密積分を半角公式で書いた形（pitch は区間内一定）：
  //   変位 = v cos(pitch) dt * sinc(alpha/2) * [cos(yaw + alpha/2), sin(yaw + alpha/2)]
  // alpha → 0 で自然にオイラー積分に一致するので分岐が要らない
  const double v = x(kV);
  const double omega = x(kOmega);
  const double u = 0.5 * omega * dt;
  double k, dk;
  sincAndDerivative(u, k, dk);
  const double beta = x(kYaw) + u;
  const double ct = std::cos(x(kPitch));
  const double st = std::sin(x(kPitch));

  Vector8d xn = x;
  xn(kX) += v * ct * dt * k * std::cos(beta);
  xn(kY) += v * ct * dt * k * std::sin(beta);
  xn(kZ) -= v * st * dt;   // ZYX ではピッチ正が機首下げ。上り坂は pitch < 0 で z が増える
  // yaw の変化率は本来 omega cos(roll)/cos(pitch) だが omega で近似する（1/12 勾配で 0.35%）
  xn(kYaw) = normalizeAngle(x(kYaw) + omega * dt);
  // roll, pitch, v, omega はランダムウォーク
  return xn;
}

VehicleEkf::Matrix8d VehicleEkf::transitionJacobian(const Vector8d & x, double dt)
{
  const double v = x(kV);
  const double omega = x(kOmega);
  const double u = 0.5 * omega * dt;
  double k, dk;
  sincAndDerivative(u, k, dk);
  const double beta = x(kYaw) + u;
  const double cb = std::cos(beta);
  const double sb = std::sin(beta);
  const double ct = std::cos(x(kPitch));
  const double st = std::sin(x(kPitch));
  const double a = v * ct * dt;   // 水平方向の移動量（sinc 補正前）

  Matrix8d F = Matrix8d::Identity();
  F(kX, kPitch) = -v * st * dt * k * cb;
  F(kY, kPitch) = -v * st * dt * k * sb;
  F(kX, kYaw) = -a * k * sb;
  F(kY, kYaw) = a * k * cb;
  F(kX, kV) = ct * dt * k * cb;
  F(kY, kV) = ct * dt * k * sb;
  // d/d(omega)：u = omega*dt/2 なので du/d(omega) = dt/2
  F(kX, kOmega) = 0.5 * a * dt * (dk * cb - k * sb);
  F(kY, kOmega) = 0.5 * a * dt * (dk * sb + k * cb);
  F(kZ, kPitch) = -v * ct * dt;
  F(kZ, kV) = -st * dt;
  F(kYaw, kOmega) = dt;
  return F;
}

VehicleEkf::Vector6d VehicleEkf::observe(const Vector8d & x) const
{
  // T_ML = T_MB · T_BL
  const Eigen::Affine3d T_ml = poseFromState(x) * mountTransform();
  Vector6d h;
  h.head<3>() = T_ml.translation();
  h.tail<3>() = rpyFromRotation(T_ml.linear());
  return h;
}

VehicleEkf::Matrix6x8d VehicleEkf::observationJacobian(const Vector8d & x) const
{
  // RPY 抽出が取付回転を含むと解析式が長くなるので中心差分で求める。h は v, omega に依存しない
  constexpr double kStep = 1.0e-6;
  Matrix6x8d H = Matrix6x8d::Zero();
  for (int j = 0; j < 6; ++j) {
    Vector8d xp = x;
    Vector8d xm = x;
    xp(j) += kStep;
    xm(j) -= kStep;
    Vector6d d = observe(xp) - observe(xm);
    for (int i = 3; i < 6; ++i) {d(i) = normalizeAngle(d(i));}
    H.col(j) = d / (2.0 * kStep);
  }
  return H;
}

// ---------------------------------------------------------------------------
// 初期化・予測
// ---------------------------------------------------------------------------

void VehicleEkf::initialize(const Vector6d & z, double stamp)
{
  // LiDAR 姿勢 → base_link 姿勢（T_MB = T_ML · T_BL^-1、厳密）
  Eigen::Affine3d T_ml = Eigen::Affine3d::Identity();
  T_ml.linear() = rotationFromRpy(z(3), z(4), z(5));
  T_ml.translation() = z.head<3>();
  const Eigen::Affine3d T_mb = T_ml * mountTransform().inverse();

  x_.setZero();
  x_.head<3>() = T_mb.translation();
  x_.segment<3>(kRoll) = rpyFromRotation(T_mb.linear());

  P_ = prm_.p_init.asDiagonal();

  gate_z_ = ScalarGate();
  gate_roll_ = ScalarGate();
  gate_pitch_ = ScalarGate();
  t_last_ = stamp;
  reject_count_ = 0;
  last_d2_ = 0.0;
  initialized_ = true;
}

void VehicleEkf::reset()
{
  initialized_ = false;
  reject_count_ = 0;
  last_d2_ = 0.0;
  last_dt_ = 0.0;
  last_dt_clamped_ = false;
  gate_z_ = ScalarGate();
  gate_roll_ = ScalarGate();
  gate_pitch_ = ScalarGate();
}

void VehicleEkf::predictTo(double stamp)
{
  if (!initialized_) {return;}

  double dt = stamp - t_last_;
  if (dt <= 0.0) {
    last_dt_ = 0.0;
    last_dt_clamped_ = false;
    return;
  }
  last_dt_clamped_ = (dt > prm_.max_predict_dt);
  if (last_dt_clamped_) {dt = prm_.max_predict_dt;}
  last_dt_ = dt;

  const Matrix8d F = transitionJacobian(x_, dt);
  x_ = propagate(x_, dt);

  P_ = F * P_ * F.transpose() + Matrix8d(prm_.q.asDiagonal()) * dt;

  t_last_ = stamp;
}

// ---------------------------------------------------------------------------
// 更新
// ---------------------------------------------------------------------------

template<int M>
bool VehicleEkf::sequentialUpdate(
  const Vector6d & z, const int (&idx)[M], double gate, double & d2_out)
{
  const Vector6d h = observe(x_);
  const Matrix6x8d H_full = observationJacobian(x_);

  Eigen::Matrix<double, M, 1> y;
  Eigen::Matrix<double, M, kN> H;
  Eigen::Matrix<double, M, M> R = Eigen::Matrix<double, M, M>::Zero();
  for (int i = 0; i < M; ++i) {
    y(i) = z(idx[i]) - h(idx[i]);
    if (isAngleObs(idx[i])) {y(i) = normalizeAngle(y(i));}
    H.row(i) = H_full.row(idx[i]);
    R(i, i) = prm_.r_ndt(idx[i]);
  }

  const Eigen::Matrix<double, M, M> S = H * P_ * H.transpose() + R;
  const Eigen::Matrix<double, M, M> S_inv = S.inverse();
  const double d2 = y.dot(S_inv * y);
  d2_out = d2;
  if (!std::isfinite(d2) || d2 > gate) {return false;}

  const Eigen::Matrix<double, kN, M> K = P_ * H.transpose() * S_inv;
  x_ += K * y;
  wrapStateAngles(x_);
  const Matrix8d IKH = Matrix8d::Identity() - K * H;
  P_ = IKH * P_ * IKH.transpose() + K * R * K.transpose();   // Joseph 形
  return true;
}

VehicleEkf::ScalarResult VehicleEkf::scalarUpdate(
  const Vector6d & z, int obs_idx, ScalarGate & gate)
{
  const int idx[1] = {obs_idx};
  double d2 = 0.0;
  const bool accepted = sequentialUpdate<1>(z, idx, prm_.gate_1d, d2);
  gate.last_d2 = d2;
  if (accepted) {
    gate.reject_count = 0;
    return ScalarResult::kUpdated;
  }

  ++gate.reject_count;
  if (prm_.lockout_count <= 0 || gate.reject_count < prm_.lockout_count) {
    return ScalarResult::kRejected;
  }

  // ロックアウト：真値が急に変わって古い値に張り付いたとみなし、観測値から出直す。
  // LiDAR の z, roll, pitch（観測の添字 2, 3, 4）には base の同じ添字の状態がほぼ 1 対 1 で効くので、
  // その状態だけを動かして観測に合わせる（ニュートン法で数回）。他成分との相関は捨てる
  const int s = obs_idx;
  const double r = prm_.r_ndt(obs_idx);
  double h_ii = 1.0;
  for (int iter = 0; iter < 3; ++iter) {
    double y = z(obs_idx) - observe(x_)(obs_idx);
    if (isAngleObs(obs_idx)) {y = normalizeAngle(y);}
    h_ii = observationJacobian(x_)(obs_idx, s);
    if (std::abs(h_ii) < 1.0e-3) {break;}   // 取付が極端で 1 対 1 でない場合は諦める
    x_(s) += y / h_ii;
    wrapStateAngles(x_);
  }
  P_.row(s).setZero();
  P_.col(s).setZero();
  P_(s, s) = 2.0 * r / (h_ii * h_ii);
  gate.reject_count = 0;
  return ScalarResult::kReinitialized;
}

VehicleEkf::UpdateResult VehicleEkf::update(const Vector6d & z)
{
  UpdateResult res;
  if (!initialized_) {return res;}

  // ---- 水平系 [x_L, y_L, yaw_L] ----
  const int horizontal[3] = {0, 1, 5};
  if (sequentialUpdate<3>(z, horizontal, prm_.gate_horizontal, last_d2_)) {
    reject_count_ = 0;
    res.horizontal_accepted = true;
  } else {
    // 水平系は再初期化しない（誤収束した解に飛び移る危険があるため）。呼び出し側で WARN
    ++reject_count_;
  }

  // ---- z, roll, pitch：それぞれ独立に判定する（水平系の結果に依存しない）----
  res.z = scalarUpdate(z, 2, gate_z_);
  res.roll = scalarUpdate(z, 3, gate_roll_);
  res.pitch = scalarUpdate(z, 4, gate_pitch_);
  return res;
}

// ---------------------------------------------------------------------------
// 出力
// ---------------------------------------------------------------------------

Eigen::Affine3d VehicleEkf::mountTransform() const
{
  Eigen::Affine3d T = Eigen::Affine3d::Identity();
  T.linear() = rotationFromRpy(prm_.roll_o, prm_.pitch_o, prm_.yaw_o);
  T.translation() = Eigen::Vector3d(prm_.o_x, prm_.o_y, prm_.o_z);
  return T;
}

Eigen::Affine3d VehicleEkf::basePose() const
{
  return poseFromState(x_);
}

Eigen::Affine3d VehicleEkf::lidarPose() const
{
  // TF の map → base_link → LiDAR と厳密に一致する
  return basePose() * mountTransform();
}

}  // namespace ekf_localizer
