// VehicleEkf / VehicleOdomEkf の単体テスト（ROS 非依存）
//   colcon build --packages-select ekf_localizer
//   colcon test --packages-select ekf_localizer --event-handlers console_direct+
#include <gtest/gtest.h>

#include <Eigen/Eigenvalues>

#include <cmath>
#include <functional>
#include <random>
#include <vector>

#include "ekf_localizer/vehicle_ekf.hpp"
#include "ekf_localizer/vehicle_odom_ekf.hpp"

using ekf_localizer::normalizeAngle;
using ekf_localizer::VehicleEkf;
using ekf_localizer::VehicleOdomEkf;
using VS = VehicleEkf::StateVector;
using MS = VehicleEkf::StateMatrix;
using V6 = VehicleEkf::Vector6d;
using SR = VehicleEkf::ScalarResult;

namespace
{

VehicleEkf::Params P0()
{
  VehicleEkf::Params p;
  p.o_x = -0.2; p.o_y = 0.05; p.o_z = 1.2; p.yaw_o = -0.034; p.roll_o = 0.01; p.pitch_o = 0.02;
  p.q << 3e-2, 3e-2, 3e-2, 5e-2, 5e-2, 5e-2, 0.05, 0.1;
  p.r_ndt << 1e-2, 1e-2, 1e-2, 1e-3, 1e-3, 5e-3;
  p.p_init << 2 * p.r_ndt, 1.0, 1.0;
  return p;
}

bool psdOK(const Eigen::MatrixXd & P)
{
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(P);
  return P.allFinite() && (P - P.transpose()).cwiseAbs().maxCoeff() < 1e-9 &&
         es.eigenvalues().minCoeff() > -1e-12;
}

/// base の真値から LiDAR 観測を作る（水平面）
V6 lidarObs(const VehicleEkf::Params & p, double xb, double yb, double psib)
{
  const double c = std::cos(psib), s = std::sin(psib);
  V6 z;
  z << xb + p.o_x * c - p.o_y * s, yb + p.o_x * s + p.o_y * c, p.o_z, p.roll_o, p.pitch_o,
    normalizeAngle(psib + p.yaw_o);
  return z;
}

Eigen::Matrix3d rotRpy(double r, double p, double y)
{
  return (Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX())).toRotationMatrix();
}

/// 回転行列 → [roll, pitch, yaw]（ZYX、tf2 の getRPY と同じ）
Eigen::Vector3d rpyOf(const Eigen::Matrix3d & R)
{
  return Eigen::Vector3d(
    std::atan2(R(2, 1), R(2, 2)), std::asin(-R(2, 0)), std::atan2(R(1, 0), R(0, 0)));
}

/// 姿勢 → [x, y, z, roll, pitch, yaw]
V6 poseVec(const Eigen::Affine3d & T)
{
  V6 z;
  z << T.translation(), rpyOf(T.linear());
  return z;
}

/// base の 6 自由度の真値から LiDAR 観測を作る（T_ML = T_MB · T_BL）
V6 lidarObs3d(const VehicleEkf & ekf, const Eigen::Affine3d & T_mb)
{
  return poseVec(T_mb * ekf.mountTransform());
}

Eigen::Affine3d makePose(double x, double y, double z, double r, double p, double yaw)
{
  Eigen::Affine3d T = Eigen::Affine3d::Identity();
  T.linear() = rotRpy(r, p, yaw);
  T.translation() = Eigen::Vector3d(x, y, z);
  return T;
}

// ---------------------------------------------------------------------------
// 走行シミュレーション：25 ms 刻み。オドメトリ 20 Hz（奇数ステップ）、NDT 10 Hz（4 ステップごと）
// ---------------------------------------------------------------------------
struct Truth
{
  double x{0.0}, y{0.0}, psi{0.3}, v{0.0}, w{0.0};
  void advance(double dt)
  {
    const double u = 0.5 * w * dt;
    const double k = std::abs(u) < 1e-9 ? 1.0 : std::sin(u) / u;
    x += v * dt * k * std::cos(psi + u);
    y += v * dt * k * std::sin(psi + u);
    psi = normalizeAngle(psi + w * dt);
  }
};

struct SimConfig
{
  double duration{20.0};
  /// 時刻 → 真の (v, w)
  std::function<void(double, double &, double &)> motion =
    [](double, double & v, double & w) {v = 1.0; w = 0.0;};
  /// 時刻 → NDT を出すか
  std::function<bool(double)> ndt_on = [](double) {return true;};
  /// 時刻 → オドメトリを出すか
  std::function<bool(double)> odom_on = [](double) {return true;};
  /// オドメトリの加工（バイアス・符号誤りなど）
  std::function<void(double, double &, double &)> odom_corrupt = [](double, double &, double &) {};
  double ndt_pos_sigma{0.1};
  double ndt_yaw_sigma{0.07};
  double odom_v_sigma{0.05};
  double odom_w_sigma{0.07};
  unsigned seed{7};
};

struct Sample
{
  double t, err_pos, v_est, w_est, v_true, w_true, err_yaw;
};

struct SimResult
{
  std::vector<Sample> samples;   // NDT かどうかに関係なく 25 ms ごと
  int odom_updates{0};
  int odom_rejects{0};
  bool psd_always{true};
};

/// odom_ekf が null なら ndt_only として走らせる（オドメトリを渡さない）
SimResult simulate(VehicleEkf & ekf, VehicleOdomEkf * odom_ekf, const SimConfig & cfg)
{
  const auto p = ekf.params();
  std::mt19937 rng(cfg.seed);
  std::normal_distribution<double> n01(0.0, 1.0);
  const Eigen::Vector2d r_odom =
    odom_ekf ? odom_ekf->odomParams().r_odom : Eigen::Vector2d(2.5e-3, 5.0e-3);

  Truth s;
  SimResult res;
  const double DT = 0.025;
  double t = 1000.0;
  const int n = static_cast<int>(cfg.duration / DT);
  for (int k = 0; k < n; ++k) {
    const double tr = k * DT;   // 経過時間
    cfg.motion(tr, s.v, s.w);
    s.advance(DT);
    t += DT;

    if (odom_ekf && (k % 2 == 1) && cfg.odom_on(tr)) {
      double v = s.v + cfg.odom_v_sigma * n01(rng);
      double w = s.w + cfg.odom_w_sigma * n01(rng);
      cfg.odom_corrupt(tr, v, w);
      const bool was_init = ekf.initialized();
      const bool ok = odom_ekf->updateOdom(v, w, r_odom, t);
      if (was_init) {
        ++res.odom_updates;
        if (!ok) {++res.odom_rejects;}
      }
    }
    if (k % 4 == 0 && cfg.ndt_on(tr)) {
      V6 z = lidarObs(p, s.x, s.y, s.psi);
      z(0) += cfg.ndt_pos_sigma * n01(rng);
      z(1) += cfg.ndt_pos_sigma * n01(rng);
      z(5) = normalizeAngle(z(5) + cfg.ndt_yaw_sigma * n01(rng));
      if (!ekf.initialized()) {
        ekf.initialize(z, t);
      } else {
        ekf.predictTo(t);
        ekf.update(z);
      }
    }
    if (!ekf.initialized()) {continue;}
    ekf.predictTo(t);
    res.psd_always &= psdOK(ekf.covariance());
    const auto b = ekf.basePose().translation();
    res.samples.push_back(
      {tr, std::hypot(b.x() - s.x, b.y() - s.y), ekf.velocity(), ekf.angularVelocity(), s.v, s.w,
        normalizeAngle(ekf.state()(VehicleEkf::kYaw) - s.psi)});
  }
  return res;
}

/// [t0, t1) のサンプルに f を適用した値の平均
double meanOver(
  const SimResult & r, double t0, double t1, const std::function<double(const Sample &)> & f)
{
  double sum = 0.0;
  int n = 0;
  for (const auto & s : r.samples) {
    if (s.t >= t0 && s.t < t1) {sum += f(s); ++n;}
  }
  return n > 0 ? sum / n : NAN;
}

double rmsOver(
  const SimResult & r, double t0, double t1, const std::function<double(const Sample &)> & f)
{
  return std::sqrt(meanOver(r, t0, t1, [&](const Sample & s) {const double e = f(s); return e * e;}));
}

VehicleOdomEkf makeOdomEkf(const VehicleEkf::Params & p = P0())
{
  VehicleOdomEkf ekf;
  ekf.setParams(p);
  return ekf;
}

}  // namespace

// ===========================================================================
// VehicleEkf（NDT のみ、8 状態）
// ===========================================================================

TEST(VehicleEkf, ArcIntegrationIsExact)
{
  using E = VehicleEkf;
  VS x = VS::Zero();
  x(E::kX) = 1.0; x(E::kY) = 2.0; x(E::kYaw) = 0.3; x(E::kV) = 1.2; x(E::kOmega) = 0.8;
  for (double dt : {0.05, 0.2, 1.0, 3.0}) {
    const VS xn = VehicleEkf::propagate(x, dt);
    const double R = x(E::kV) / x(E::kOmega);
    const double ex = x(E::kX) + R * (std::sin(x(E::kYaw) + x(E::kOmega) * dt) - std::sin(x(E::kYaw)));
    const double ey = x(E::kY) - R * (std::cos(x(E::kYaw) + x(E::kOmega) * dt) - std::cos(x(E::kYaw)));
    EXPECT_LT(std::hypot(xn(E::kX) - ex, xn(E::kY) - ey), 1e-12) << "dt=" << dt;
    EXPECT_DOUBLE_EQ(xn(E::kZ), x(E::kZ)) << "pitch = 0 なら z は動かない";
  }
  VS a = VS::Zero();
  a(E::kYaw) = 0.3; a(E::kV) = 1.0; a(E::kOmega) = 1e-9;
  VS b = a;
  b(E::kOmega) = 0.0;
  EXPECT_LT((VehicleEkf::propagate(a, 0.1) - VehicleEkf::propagate(b, 0.1)).head<2>().norm(), 1e-9)
    << "omega → 0 で直線に連続（分岐なし）";

  // 傾斜：水平速度は v cos(pitch)、z は -v sin(pitch)（上り坂は pitch < 0）
  VS c = x;
  const double th = -std::atan(1.0 / 12.0);
  c(E::kPitch) = th;
  const double dt = 0.5;
  const VS cn = VehicleEkf::propagate(c, dt), xn = VehicleEkf::propagate(x, dt);
  const double flat = std::hypot(xn(E::kX) - x(E::kX), xn(E::kY) - x(E::kY));
  EXPECT_NEAR(std::hypot(cn(E::kX) - c(E::kX), cn(E::kY) - c(E::kY)), flat * std::cos(th), 1e-12);
  EXPECT_NEAR(cn(E::kZ) - c(E::kZ), -c(E::kV) * std::sin(th) * dt, 1e-12);
  EXPECT_GT(cn(E::kZ), c(E::kZ)) << "上り坂で z が増える";
}

TEST(VehicleEkf, TransitionJacobianMatchesNumerical)
{
  for (double w : {0.8, 1e-5, 0.0, -1.3}) {
    for (double th : {0.0, 0.2, -0.08}) {
      VS x;
      x << 1.0, -2.0, 0.5, 0.05, th, 2.9, 0.9, w;
      const double dt = 0.15, h = 1e-6;
      MS Fn;
      for (int j = 0; j < VehicleEkf::kN; ++j) {
        VS xp = x, xm = x;
        xp(j) += h; xm(j) -= h;
        VS d = VehicleEkf::propagate(xp, dt) - VehicleEkf::propagate(xm, dt);
        d(VehicleEkf::kYaw) = normalizeAngle(d(VehicleEkf::kYaw));
        Fn.col(j) = d / (2 * h);
      }
      const MS Fa = VehicleEkf::transitionJacobian(x, dt);
      EXPECT_LT((Fa - Fn).cwiseAbs().maxCoeff(), 1e-6) << "omega=" << w << " pitch=" << th;
    }
  }
}

TEST(VehicleEkf, ObservationJacobianMatchesAnalytic)
{
  // 数値微分の H の位置部分を解析式と照合する：p_L = p_b + R(roll, pitch, yaw) o
  const auto p = P0();
  VehicleEkf ekf;
  ekf.setParams(p);
  VS x;
  x << 1.0, -2.0, 0.3, 0.07, -0.12, 2.9, 0.9, 0.3;
  const auto H = ekf.observationJacobian(x);
  const double r = x(3), pt = x(4), y = x(5);
  const Eigen::Vector3d o(p.o_x, p.o_y, p.o_z);
  const auto skew = [](const Eigen::Vector3d & a) {
      Eigen::Matrix3d S;
      S << 0, -a.z(), a.y(), a.z(), 0, -a.x(), -a.y(), a.x(), 0;
      return S;
    };
  const Eigen::Matrix3d Rz = rotRpy(0, 0, y), Ry = rotRpy(0, pt, 0), Rx = rotRpy(r, 0, 0);
  Eigen::Matrix3d Hp;
  Hp.col(0) = Rz * Ry * skew(Eigen::Vector3d::UnitX()) * Rx * o;   // d/d roll
  Hp.col(1) = Rz * skew(Eigen::Vector3d::UnitY()) * Ry * Rx * o;   // d/d pitch
  Hp.col(2) = skew(Eigen::Vector3d::UnitZ()) * Rz * Ry * Rx * o;   // d/d yaw
  EXPECT_LT(((H.block<3, 3>(0, 0)) - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff(), 1e-8);
  EXPECT_LT(((H.block<3, 3>(0, 3)) - Hp).cwiseAbs().maxCoeff(), 1e-8);
  EXPECT_LT((H.block<3, 3>(3, 0).cwiseAbs().maxCoeff()), 1e-8) << "姿勢は位置に依存しない";
  EXPECT_LT(H.rightCols<2>().cwiseAbs().maxCoeff(), 1e-12) << "v, omega に依存しない";

  // 水平面（roll = pitch = 0）では旧 5 状態モデルの H と一致する
  VS xf = x;
  xf(3) = 0.0; xf(4) = 0.0;
  const auto Hf = ekf.observationJacobian(xf);
  const double c = std::cos(y), s = std::sin(y);
  EXPECT_NEAR(Hf(0, 5), -p.o_x * s - p.o_y * c, 1e-8);
  EXPECT_NEAR(Hf(1, 5), p.o_x * c - p.o_y * s, 1e-8);
  EXPECT_NEAR(Hf(5, 5), 1.0, 1e-8);
}

TEST(VehicleEkf, RecoversBaseLinkWhileTurning)
{
  const auto p = P0();
  VehicleEkf ekf;
  ekf.setParams(p);
  std::mt19937 rng(3);
  std::normal_distribution<double> np(0, 0.1), na(0, 0.07), nz(0, 0.1), nr(0, 0.03);
  double t = 1000, xb = 0, yb = 0, psib = 0.5;
  const double V = 0.8, W = 0.3, DT = 0.05;
  ekf.initialize(lidarObs(p, xb, yb, psib), t);
  double se = 0, roll_sum = 0, pitch_sum = 0;
  int n = 0;
  for (int k = 0; k < 800; ++k) {
    t += DT;
    xb += V / W * (std::sin(psib + W * DT) - std::sin(psib));
    yb -= V / W * (std::cos(psib + W * DT) - std::cos(psib));
    psib = normalizeAngle(psib + W * DT);
    ekf.predictTo(t);
    V6 z = lidarObs(p, xb, yb, psib);
    z(0) += np(rng); z(1) += np(rng); z(2) += nz(rng); z(3) += nr(rng); z(4) += nr(rng);
    z(5) += na(rng);
    ekf.update(z);
    if (k > 300) {
      const auto b = ekf.basePose().translation();
      se += std::hypot(b.x() - xb, b.y() - yb);
      roll_sum += ekf.state()(VehicleEkf::kRoll);
      pitch_sum += ekf.state()(VehicleEkf::kPitch);
      ++n;
    }
  }
  EXPECT_NEAR(ekf.velocity(), V, 0.15);
  EXPECT_NEAR(ekf.angularVelocity(), W, 0.1);
  EXPECT_LT(se / n, 0.09) << "base_link 位置誤差 < 生観測ノイズ(約0.13m)の7割";
  EXPECT_NEAR(ekf.basePose().translation().z(), 0.0, 0.05) << "LiDAR z 1.2 - o_z 1.2";
  // 1 サンプルは観測ノイズ（sd 0.03 rad）並みに揺れるので、区間平均でバイアスが無いことを見る
  EXPECT_NEAR(roll_sum / n, 0.0, 0.005) << "水平な床では車体 roll = 0";
  EXPECT_NEAR(pitch_sum / n, 0.0, 0.005) << "水平な床では車体 pitch = 0";
  EXPECT_TRUE(psdOK(ekf.covariance()));
}

TEST(VehicleEkf, OnlyZRejectedWhenZJumps)
{
  const auto p = P0();
  VehicleEkf ekf;
  ekf.setParams(p);
  double t = 1000;
  ekf.initialize(lidarObs(p, 0, 0, 0), t);
  for (int k = 0; k < 100; ++k) {t += 0.05; ekf.predictTo(t); ekf.update(lidarObs(p, 0, 0, 0));}
  const double z_before = ekf.lidarPose().translation().z();
  t += 0.05;
  ekf.predictTo(t);
  V6 bad = lidarObs(p, 0, 0, 0);
  bad(2) += 0.8;
  const auto r = ekf.update(bad);
  EXPECT_TRUE(r.horizontal_accepted);
  EXPECT_EQ(r.z, SR::kRejected);
  EXPECT_EQ(r.roll, SR::kUpdated);
  EXPECT_EQ(r.pitch, SR::kUpdated);
  EXPECT_EQ(ekf.zGate().reject_count, 1);
  // 観測が予測と一致している roll, pitch の更新では状態は動かない
  EXPECT_NEAR(ekf.lidarPose().translation().z(), z_before, 1e-9);
}

TEST(VehicleEkf, LockoutReinitializesAfterStep)
{
  auto p = P0();
  p.lockout_count = 10;
  VehicleEkf ekf;
  ekf.setParams(p);
  double t = 1000;
  ekf.initialize(lidarObs(p, 0, 0, 0), t);
  for (int k = 0; k < 100; ++k) {t += 0.05; ekf.predictTo(t); ekf.update(lidarObs(p, 0, 0, 0));}
  int reinit_at = -1;
  for (int k = 1; k <= 30; ++k) {
    t += 0.05;
    ekf.predictTo(t);
    V6 zz = lidarObs(p, 0, 0, 0);
    zz(2) += 0.8;
    const auto r = ekf.update(zz);
    if (reinit_at < 0 && r.z == SR::kReinitialized) {reinit_at = k;}
  }
  EXPECT_EQ(reinit_at, 10);
  EXPECT_NEAR(ekf.lidarPose().translation().z(), p.o_z + 0.8, 0.05);
  EXPECT_TRUE(psdOK(ekf.covariance()));
}

TEST(VehicleEkf, ScalarsIndependentOfHorizontalGate)
{
  const auto p = P0();
  VehicleEkf ekf;
  ekf.setParams(p);
  double t = 1000;
  ekf.initialize(lidarObs(p, 0, 0, 0), t);
  for (int k = 0; k < 100; ++k) {t += 0.05; ekf.predictTo(t); ekf.update(lidarObs(p, 0, 0, 0));}
  t += 0.05;
  ekf.predictTo(t);
  V6 zz = lidarObs(p, 3.0, 0, 0);
  zz(4) += 0.02;
  const double pitch_before = rpyOf(ekf.lidarPose().linear())(1);
  const auto r = ekf.update(zz);
  EXPECT_FALSE(r.horizontal_accepted);
  EXPECT_EQ(r.pitch, SR::kUpdated);
  EXPECT_GT(rpyOf(ekf.lidarPose().linear())(1), pitch_before + 1e-4);
}

TEST(VehicleEkf, StaticTfConsistent)
{
  const auto p = P0();
  VehicleEkf ekf;
  ekf.setParams(p);
  ekf.initialize(lidarObs(p, 4.0, -1.0, 1.1), 0.0);
  const Eigen::Affine3d d = ekf.lidarPose().inverse() * (ekf.basePose() * ekf.mountTransform());
  EXPECT_LT((d.matrix() - Eigen::Matrix4d::Identity()).cwiseAbs().maxCoeff(), 1e-12);
  const Eigen::Vector3d bt = ekf.basePose().translation();
  EXPECT_LT(std::hypot(bt.x() - ekf.state()(0), bt.y() - ekf.state()(1)), 1e-9);
}

TEST(VehicleEkf, InitializeRoundTrip)
{
  // roll, pitch, 取付回転がすべて 0 でない LiDAR 姿勢で初期化し、lidarPose() が観測に戻る
  const auto p = P0();
  VehicleEkf ekf;
  ekf.setParams(p);
  const Eigen::Affine3d T_mb = makePose(3.0, -1.0, 0.4, 0.05, -0.08, 2.0);
  const V6 z = lidarObs3d(ekf, T_mb);
  ekf.initialize(z, 0.0);
  const V6 back = poseVec(ekf.lidarPose());
  V6 d = back - z;
  for (int i = 3; i < 6; ++i) {d(i) = normalizeAngle(d(i));}
  EXPECT_LT(d.cwiseAbs().maxCoeff(), 1e-9);
  EXPECT_LT((ekf.state().head<6>() - poseVec(T_mb)).cwiseAbs().maxCoeff(), 1e-9)
    << "状態は base_link の姿勢";
  EXPECT_TRUE(psdOK(ekf.covariance()));
  // 初期共分散は P_init の対角
  EXPECT_LT((ekf.covariance() - MS(p.p_init.asDiagonal())).cwiseAbs().maxCoeff(), 1e-15);
}

// ---------------------------------------------------------------------------
// 傾斜（docs/ekf_3d_plan.md 3.2 節）。1/12 勾配を 1 m/s で直進して登る
// ---------------------------------------------------------------------------
namespace
{

struct SlopeResult
{
  double bias_xy;       // 収束後の base 水平位置誤差の符号つき平均の大きさ [m]
  double rms_xy;        // 収束後の base 水平位置誤差の rms [m]
  double bias_pitch;    // 収束後の車体 pitch 推定の平均誤差 [rad]
  double z_err_end;     // 最後の base z 誤差 [m]
};

/// ndt_off_from 以降 NDT を止める（負なら止めない）。noise = 0 で観測ノイズなし
SlopeResult runSlope(double duration, double ndt_off_from, double noise = 1.0)
{
  auto p = P0();
  p.r_ndt << 1e-2, 1e-2, 1e-2, 1e-4, 1e-4, 1e-3;
  VehicleEkf ekf;
  ekf.setParams(p);
  std::mt19937 rng(11);
  std::normal_distribution<double> n01(0.0, 1.0);

  const double slope = std::atan(1.0 / 12.0), V = 1.0, yaw = 0.4, DT = 0.025;
  const auto truth = [&](double t) {
      const double s = V * t;   // 斜面に沿った距離
      return makePose(
        s * std::cos(slope) * std::cos(yaw), s * std::cos(slope) * std::sin(yaw),
        s * std::sin(slope), 0.0, -slope, yaw);
    };
  const auto noisy = [&](const V6 & z) {
      V6 n = z;
      for (int i = 0; i < 3; ++i) {n(i) += noise * 0.1 * n01(rng);}
      n(3) += noise * 0.01 * n01(rng);
      n(4) += noise * 0.01 * n01(rng);
      n(5) += noise * 0.03 * n01(rng);
      return n;
    };

  double t0 = 1000.0, sq = 0.0, pitch_err = 0.0;
  Eigen::Vector2d sum = Eigen::Vector2d::Zero();
  int n = 0;
  SlopeResult res{};
  for (int k = 0; k * DT <= duration; ++k) {
    const double tr = k * DT, t = t0 + tr;
    const Eigen::Affine3d T = truth(tr);
    if (k % 4 == 0 && (ndt_off_from < 0 || tr < ndt_off_from)) {
      const V6 z = noisy(lidarObs3d(ekf, T));
      if (!ekf.initialized()) {ekf.initialize(z, t);} else {ekf.predictTo(t); ekf.update(z);}
    }
    if (!ekf.initialized()) {continue;}
    ekf.predictTo(t);
    const Eigen::Vector3d e = ekf.basePose().translation() - T.translation();
    if (tr > 5.0 && (ndt_off_from < 0 || tr < ndt_off_from)) {
      sum += e.head<2>();
      sq += e.head<2>().squaredNorm();
      pitch_err += ekf.state()(VehicleEkf::kPitch) + slope;
      ++n;
    }
    res.z_err_end = std::abs(e.z());
  }
  res.bias_xy = (sum / n).norm();
  res.rms_xy = std::sqrt(sq / n);
  res.bias_pitch = pitch_err / n;
  return res;
}

}  // namespace

TEST(VehicleEkf, SlopeHasNoLeverArmBias)
{
  // 旧 5 状態モデル（水平面仮定の h）では状態 x_b, y_b に o_z sin(slope) ≈ 10 cm のバイアスが乗った
  const auto clean = runSlope(15.0, -1.0, 0.0);
  const auto noisy = runSlope(15.0, -1.0, 1.0);
  std::printf(
    "  傾斜 1/12：ノイズなし バイアス %.4f m・pitch %.5f rad / ノイズあり バイアス %.3f m・rms %.3f m"
    "（旧モデルのバイアス %.3f m）\n",
    clean.bias_xy, clean.bias_pitch, noisy.bias_xy, noisy.rms_xy, 1.2 * std::sin(std::atan(1.0 / 12.0)));
  EXPECT_LT(clean.bias_xy, 1e-3) << "モデルが正しければノイズなしでバイアスは出ない";
  EXPECT_LT(std::abs(clean.bias_pitch), 1e-3) << "車体 pitch は斜面の角度に一致する";
  EXPECT_LT(noisy.bias_xy, 0.03);
  EXPECT_LT(noisy.rms_xy, 0.1) << "観測ノイズ（各軸 sd 0.1 m）より小さい";
}

TEST(VehicleEkf, SlopeZFollowsDuringNdtOutage)
{
  // 10 s から 2 s 欠測。z 一定の予測なら v sin(slope) * 2 s = 0.17 m ずれる
  const auto r = runSlope(12.0, 10.0);
  std::printf("  傾斜 1/12・NDT 2 s 欠測後の z 誤差 %.3f m（z 一定なら 0.166 m）\n", r.z_err_end);
  EXPECT_LT(r.z_err_end, 0.05);
}

// ===========================================================================
// VehicleOdomEkf（NDT ＋ オドメトリ観測）。docs/ekf_odom_plan.md 4.1 節
// ===========================================================================

TEST(VehicleOdomEkf, UpdateMatchesGenericKalmanWithH)
{
  // updateOdom が一般形の KF 更新（h = [v, omega]）と一致するか
  const auto p = P0();
  VehicleOdomEkf ekf = makeOdomEkf(p);
  ekf.initialize(lidarObs(p, 1.0, 2.0, 0.4), 10.0);
  // 非対角が埋まった P を作る
  double t = 10.0;
  for (int k = 0; k < 20; ++k) {
    t += 0.05;
    ekf.predictTo(t);
    ekf.update(lidarObs(p, 1.0 + 0.05 * k, 2.0, 0.4));
    ekf.updateOdom(1.0, 1.0, Eigen::Vector2d(2.5e-3, 5.0e-3), t);
  }
  const VS x0 = ekf.state();
  const MS P = ekf.covariance();
  ASSERT_GT(std::abs(P(VehicleEkf::kX, VehicleEkf::kV)), 1e-8);
  const Eigen::Vector2d z(0.9, 1.1), r(2.5e-3, 5.0e-3);

  constexpr int N = VehicleEkf::kN;
  Eigen::Matrix<double, 2, N> H = Eigen::Matrix<double, 2, N>::Zero();
  H(0, VehicleEkf::kV) = 1.0;
  H(1, VehicleEkf::kOmega) = 1.0;
  const Eigen::Vector2d y = z - Eigen::Vector2d(x0(VehicleEkf::kV), x0(VehicleEkf::kOmega));
  const Eigen::Matrix2d R = r.asDiagonal();
  const Eigen::Matrix2d S = H * P * H.transpose() + R;
  const Eigen::Matrix<double, N, 2> K = P * H.transpose() * S.inverse();
  VS x_exp = x0 + K * y;
  for (int i = VehicleEkf::kRoll; i <= VehicleEkf::kYaw; ++i) {x_exp(i) = normalizeAngle(x_exp(i));}
  const MS IKH = MS::Identity() - K * H;
  const MS P_exp = IKH * P * IKH.transpose() + K * R * K.transpose();

  ASSERT_TRUE(ekf.updateOdom(z(0), z(1), r, t));   // 同時刻なので予測は入らない
  EXPECT_LT((ekf.state() - x_exp).cwiseAbs().maxCoeff(), 1e-12);
  EXPECT_LT((ekf.covariance() - P_exp).cwiseAbs().maxCoeff(), 1e-12);
  EXPECT_NEAR(ekf.lastOdomMahalanobis(), y.dot(S.inverse() * y), 1e-9);
}

TEST(VehicleOdomEkf, LowerVelocityNoiseThanNdtOnly)
{
  SimConfig cfg;
  cfg.motion = [](double, double & v, double & w) {v = 1.0; w = 0.0;};
  VehicleEkf ndt_only;
  ndt_only.setParams(P0());
  VehicleOdomEkf odom = makeOdomEkf();
  const auto r0 = simulate(ndt_only, nullptr, cfg);
  const auto r1 = simulate(odom, &odom, cfg);

  const auto ev = [](const Sample & s) {return s.v_est - s.v_true;};
  const auto ew = [](const Sample & s) {return s.w_est - s.w_true;};
  const double v0 = rmsOver(r0, 5, 20, ev), v1 = rmsOver(r1, 5, 20, ev);
  const double w0 = rmsOver(r0, 5, 20, ew), w1 = rmsOver(r1, 5, 20, ew);
  RecordProperty("v_rms_ndt_only", std::to_string(v0));
  RecordProperty("v_rms_ndt_odom", std::to_string(v1));
  std::printf("  v rms: ndt_only %.4f  ndt_odom %.4f / omega rms: %.4f  %.4f\n", v0, v1, w0, w1);
  // 実測 v: 0.059 → 0.033、omega: 0.065 → 0.043（オドメトリ自体のノイズ 0.05 / 0.07 が下限を決める）
  EXPECT_LT(v1, 0.8 * v0);
  EXPECT_LT(w1, 0.8 * w0);
  EXPECT_TRUE(r1.psd_always);
}

TEST(VehicleOdomEkf, CoastsThroughNdtOutage)
{
  // 5〜7 s に NDT が欠測し、その間に真値は左旋回を始める
  SimConfig cfg;
  cfg.duration = 7.0;
  cfg.motion = [](double t, double & v, double & w) {v = 1.0; w = (t >= 5.0) ? 0.5 : 0.0;};
  cfg.ndt_on = [](double t) {return t < 5.0;};
  VehicleEkf ndt_only;
  ndt_only.setParams(P0());
  VehicleOdomEkf odom = makeOdomEkf();
  const auto r0 = simulate(ndt_only, nullptr, cfg);
  const auto r1 = simulate(odom, &odom, cfg);
  const double e0 = r0.samples.back().err_pos, e1 = r1.samples.back().err_pos;
  std::printf("  2 s 欠測後の位置誤差: ndt_only %.3f m  ndt_odom %.3f m\n", e0, e1);
  EXPECT_LT(e1, 0.3 * e0);
  EXPECT_LT(e1, 0.3);
  EXPECT_TRUE(r1.psd_always);
}

TEST(VehicleOdomEkf, SlipFollowsOdomUnlessROdomIsLarge)
{
  // オドメトリが真値より 10% 大きい（スリップ・スケール誤差）。
  // 【既知の制約】オドメトリは v を直接・高頻度で観測するのに対し、NDT は位置を介した間接情報なので、
  // R_odom が小さいと推定 v はほぼオドメトリに張り付く（NDT とのずれは Q_pose の位置ノイズとして吸収される）。
  // スケール誤差を NDT で補正するには係数を状態に入れる必要がある（計画書 段階 5）。
  SimConfig cfg;
  cfg.duration = 30.0;
  cfg.motion = [](double, double & v, double & w) {v = 1.0; w = 0.0;};
  cfg.odom_corrupt = [](double, double & v, double &) {v *= 1.1;};
  const auto run = [&](double r_v) {
      VehicleOdomEkf ekf = makeOdomEkf();
      VehicleOdomEkf::OdomParams op;
      op.r_odom(0) = r_v;
      ekf.setOdomParams(op);
      const auto r = simulate(ekf, &ekf, cfg);
      return meanOver(r, 10, 30, [](const Sample & s) {return s.v_est;});
    };
  const double v_small = run(2.5e-3), v_large = run(2.5e-1);
  std::printf("  推定 v（真 1.0 / odom 1.1）: R_odom 2.5e-3 → %.3f,  2.5e-1 → %.3f\n", v_small, v_large);
  EXPECT_NEAR(v_small, 1.1, 0.01) << "R_odom が小さいとオドメトリに張り付く";
  EXPECT_LT(v_large, v_small - 0.01) << "R_odom を大きくすると NDT 側へ寄る";
  EXPECT_GT(v_large, 0.98);
}

TEST(VehicleOdomEkf, SingleOutlierRejected)
{
  SimConfig cfg;
  cfg.duration = 10.0;
  VehicleOdomEkf ekf = makeOdomEkf();
  simulate(ekf, &ekf, cfg);
  const VS x_before = ekf.state();
  const MS P_before = ekf.covariance();
  // 直前の予測と同時刻に入れるので、予測は入らず更新の有無だけが見える
  const double t_end = 1000.0 + static_cast<int>(cfg.duration / 0.025) * 0.025;
  EXPECT_FALSE(ekf.updateOdom(3.0, 0.0, Eigen::Vector2d(2.5e-3, 5.0e-3), t_end));
  EXPECT_GT(ekf.lastOdomMahalanobis(), 9.21);
  EXPECT_EQ(ekf.odomRejectCount(), 1);
  // t_end は加算の累積誤差で sim の時刻とごくわずかにずれるので、極小の予測ぶんは許す
  EXPECT_LT((ekf.state() - x_before).cwiseAbs().maxCoeff(), 1e-9) << "外れ値で状態が動かない";
  EXPECT_LT((ekf.covariance() - P_before).cwiseAbs().maxCoeff(), 1e-9);
}

TEST(VehicleOdomEkf, WrongOmegaSignIsNotCaughtByOdomGate)
{
  // 左旋回中にオドメトリの omega の符号が逆。
  // 【既知の制約】オドメトリのゲートでは検出できない。omega の推定はオドメトリに引きずられて符号が逆になり、
  // NDT とのずれは yaw のプロセスノイズとして吸収される（位置は NDT で保たれる）。
  // Q_pose を小さくすると逆に NDT の方が棄却されて発散するので、符号は走らせる前に確認すること。
  SimConfig cfg;
  cfg.duration = 20.0;
  cfg.motion = [](double, double & v, double & w) {v = 0.5; w = 0.5;};
  cfg.odom_corrupt = [](double, double &, double & w) {w = -w;};
  VehicleOdomEkf ekf = makeOdomEkf();
  const auto r = simulate(ekf, &ekf, cfg);
  const double rate = static_cast<double>(r.odom_rejects) / r.odom_updates;
  const double w_est = meanOver(r, 10, 20, [](const Sample & s) {return s.w_est;});
  const double pos = rmsOver(r, 10, 20, [](const Sample & s) {return s.err_pos;});
  std::printf("  符号誤り: odom 棄却率 %.1f%%  推定 omega %.2f（真 0.5）  位置 rms %.3f m\n",
    100 * rate, w_est, pos);
  EXPECT_LT(rate, 0.05) << "ゲートは符号誤りを弾かない";
  EXPECT_LT(w_est, 0.0) << "omega の推定は誤ったオドメトリに従う";
  EXPECT_LT(pos, 0.15) << "位置は NDT で保たれる";
}

TEST(VehicleOdomEkf, OdomDropoutFallsBackToNdtOnly)
{
  // 5 s 以降オドメトリが止まる
  SimConfig cfg;
  cfg.duration = 20.0;
  cfg.motion = [](double t, double & v, double & w) {v = 0.8; w = (t > 10.0) ? 0.3 : 0.0;};
  cfg.odom_on = [](double t) {return t < 5.0;};
  VehicleOdomEkf ekf = makeOdomEkf();
  VehicleEkf ndt_only;
  ndt_only.setParams(P0());
  const auto r1 = simulate(ekf, &ekf, cfg);
  const auto r0 = simulate(ndt_only, nullptr, cfg);
  const auto ep = [](const Sample & s) {return s.err_pos;};
  const double e1 = rmsOver(r1, 12, 20, ep), e0 = rmsOver(r0, 12, 20, ep);
  std::printf("  odom 途絶後の位置誤差 rms: ndt_odom %.3f  ndt_only %.3f\n", e1, e0);
  EXPECT_TRUE(r1.psd_always);
  EXPECT_NEAR(e1, e0, 0.02);
  EXPECT_NEAR(r1.samples.back().w_est, 0.3, 0.1);
}

TEST(VehicleOdomEkf, InitializesVelocityFromRecentOdom)
{
  const auto p = P0();
  const Eigen::Vector2d r(2.5e-3, 5.0e-3);

  VehicleOdomEkf recent = makeOdomEkf(p);
  EXPECT_FALSE(recent.updateOdom(0.8, 0.2, r, 99.9));   // 未初期化：記録だけ
  recent.initialize(lidarObs(p, 0, 0, 0), 100.0);
  EXPECT_TRUE(recent.initializedFromOdom());
  EXPECT_DOUBLE_EQ(recent.velocity(), 0.8);
  EXPECT_DOUBLE_EQ(recent.angularVelocity(), 0.2);
  EXPECT_DOUBLE_EQ(recent.covariance()(VehicleEkf::kV, VehicleEkf::kV), r(0));
  EXPECT_DOUBLE_EQ(recent.covariance()(VehicleEkf::kOmega, VehicleEkf::kOmega), r(1));

  VehicleOdomEkf stale = makeOdomEkf(p);
  stale.updateOdom(0.8, 0.2, r, 99.0);   // 1 s 前は odom_timeout_init(0.2) を超える
  stale.initialize(lidarObs(p, 0, 0, 0), 100.0);
  EXPECT_FALSE(stale.initializedFromOdom());
  EXPECT_DOUBLE_EQ(stale.velocity(), 0.0);
  EXPECT_DOUBLE_EQ(
    stale.covariance()(VehicleEkf::kV, VehicleEkf::kV), p.p_init(VehicleEkf::kV));

  // reset 後も直近のオドメトリで初期化し直せる（初期姿勢の再設定）
  recent.reset();
  EXPECT_FALSE(recent.initialized());
  recent.updateOdom(0.5, -0.1, r, 105.0);
  recent.initialize(lidarObs(p, 0, 0, 0), 105.05);
  EXPECT_TRUE(recent.initializedFromOdom());
  EXPECT_DOUBLE_EQ(recent.velocity(), 0.5);
}
