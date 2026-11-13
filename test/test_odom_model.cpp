// OdomModel（オドメトリの観測モデル）と OdomNoise の単体テスト（ROS 非依存）
//   colcon build --packages-select ekf_localizer
//   colcon test --packages-select ekf_localizer --event-handlers console_direct+
#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <functional>
#include <random>
#include <string>
#include <vector>

#include "test_helpers.hpp"

using namespace test_helpers;  // NOLINT(build/namespaces)
using ekf_localizer::OdomNoise;

namespace
{

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

/// rig.odom が null なら ndt_only として走らせる（オドメトリを渡さない）
SimResult simulate(Rig & rig, const SimConfig & cfg, const TestParams & p)
{
  std::mt19937 rng(cfg.seed);
  std::normal_distribution<double> n01(0.0, 1.0);
  const Eigen::Vector2d r_odom = rig.odom ? rig.odom->params().r : Eigen::Vector2d(2.5e-3, 5.0e-3);
  EkfCore & ekf = rig.core;

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

    if (rig.odom && (k % 2 == 1) && cfg.odom_on(tr)) {
      double v = s.v + cfg.odom_v_sigma * n01(rng);
      double w = s.w + cfg.odom_w_sigma * n01(rng);
      cfg.odom_corrupt(tr, v, w);
      const ApplyResult o = rig.addOdom(v, w, r_odom, t);
      if (o.was_initialized) {
        ++res.odom_updates;
        if (o.stages.at(0).status != SR::kUpdated) {++res.odom_rejects;}
      }
    }
    if (k % 4 == 0 && cfg.ndt_on(tr)) {
      V6 z = lidarObs(p, s.x, s.y, s.psi);
      z(0) += cfg.ndt_pos_sigma * n01(rng);
      z(1) += cfg.ndt_pos_sigma * n01(rng);
      z(5) = normalizeAngle(z(5) + cfg.ndt_yaw_sigma * n01(rng));
      rig.addNdt(z, t);
    }
    if (!ekf.initialized()) {continue;}
    ekf.predictTo(t);
    res.psd_always &= psdOK(ekf.covariance());
    const auto b = ekf.basePose().translation();
    res.samples.push_back(
      {tr, std::hypot(b.x() - s.x, b.y() - s.y), ekf.vXy(), ekf.omegaYaw(), s.v, s.w,
        normalizeAngle(ekf.state()(EkfCore::kYaw) - s.psi)});
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


}  // namespace

// ===========================================================================
// OdomModel（NDT ＋ オドメトリ観測）
// ===========================================================================

TEST(OdomModel, UpdateMatchesGenericKalmanWithH)
{
  // オドメトリの更新が一般形の KF 更新（h = [v, omega]）と一致するか
  const auto p = P0();
  Rig rig = makeRig(p, true);
  rig.addNdt(lidarObs(p, 1.0, 2.0, 0.4), 10.0);
  // 非対角が埋まった P を作る
  double t = 10.0;
  for (int k = 0; k < 20; ++k) {
    t += 0.05;
    rig.addNdt(lidarObs(p, 1.0 + 0.05 * k, 2.0, 0.4), t);
    rig.addOdom(1.0, 1.0, Eigen::Vector2d(2.5e-3, 5.0e-3), t);
  }
  const VS x0 = rig.core.state();
  const MS P = rig.core.covariance();
  ASSERT_GT(std::abs(P(EkfCore::kX, EkfCore::kVxy)), 1e-8);
  const Eigen::Vector2d z(0.9, 1.1), r(2.5e-3, 5.0e-3);

  constexpr int N = EkfCore::kStateSize;
  Eigen::Matrix<double, 2, N> H = Eigen::Matrix<double, 2, N>::Zero();
  H(0, EkfCore::kVxy) = 1.0;
  H(1, EkfCore::kOmegaYaw) = 1.0;
  const Eigen::Vector2d y = z - Eigen::Vector2d(x0(EkfCore::kVxy), x0(EkfCore::kOmegaYaw));
  const Eigen::Matrix2d R = r.asDiagonal();
  const Eigen::Matrix2d S = H * P * H.transpose() + R;
  const Eigen::Matrix<double, N, 2> K = P * H.transpose() * S.inverse();
  VS x_exp = x0 + K * y;
  for (int i = EkfCore::kRoll; i <= EkfCore::kYaw; ++i) {x_exp(i) = normalizeAngle(x_exp(i));}
  const MS IKH = MS::Identity() - K * H;
  const MS P_exp = IKH * P * IKH.transpose() + K * R * K.transpose();

  // 同時刻なので予測は入らない
  const ApplyResult o = rig.addOdom(z(0), z(1), r, t);
  ASSERT_EQ(o.stages.at(0).status, SR::kUpdated);
  EXPECT_LT((rig.core.state() - x_exp).cwiseAbs().maxCoeff(), 1e-12);
  EXPECT_LT((rig.core.covariance() - P_exp).cwiseAbs().maxCoeff(), 1e-12);
  EXPECT_NEAR(o.stages.at(0).d2, y.dot(S.inverse() * y), 1e-9);
}

TEST(OdomModel, LowerVelocityNoiseThanNdtOnly)
{
  SimConfig cfg;
  cfg.motion = [](double, double & v, double & w) {v = 1.0; w = 0.0;};
  const auto p = P0();
  Rig ndt_only = makeRig(p, false);
  Rig odom = makeRig(p, true);
  const auto r0 = simulate(ndt_only, cfg, p);
  const auto r1 = simulate(odom, cfg, p);

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

TEST(OdomModel, CoastsThroughNdtOutage)
{
  // 5〜7 s に NDT が欠測し、その間に真値は左旋回を始める
  SimConfig cfg;
  cfg.duration = 7.0;
  cfg.motion = [](double t, double & v, double & w) {v = 1.0; w = (t >= 5.0) ? 0.5 : 0.0;};
  cfg.ndt_on = [](double t) {return t < 5.0;};
  const auto p = P0();
  Rig ndt_only = makeRig(p, false);
  Rig odom = makeRig(p, true);
  const auto r0 = simulate(ndt_only, cfg, p);
  const auto r1 = simulate(odom, cfg, p);
  const double e0 = r0.samples.back().err_pos, e1 = r1.samples.back().err_pos;
  std::printf("  2 s 欠測後の位置誤差: ndt_only %.3f m  ndt_odom %.3f m\n", e0, e1);
  EXPECT_LT(e1, 0.3 * e0);
  EXPECT_LT(e1, 0.3);
  EXPECT_TRUE(r1.psd_always);
}

TEST(OdomModel, SlipFollowsOdomUnlessROdomIsLarge)
{
  // オドメトリが真値より 10% 大きい（スリップ・スケール誤差）。
  // 【既知の制約】オドメトリは v を直接・高頻度で観測するのに対し、NDT は位置を介した間接情報なので、
  // R_odom が小さいと推定 v はほぼオドメトリに張り付く（NDT とのずれは Q_pose の位置ノイズとして吸収される）。
  // スケール誤差を NDT で補正するには係数を状態に入れる必要がある。
  SimConfig cfg;
  cfg.duration = 30.0;
  cfg.motion = [](double, double & v, double & w) {v = 1.0; w = 0.0;};
  cfg.odom_corrupt = [](double, double & v, double &) {v *= 1.1;};
  const auto run = [&](double r_v) {
      auto p = P0();
      p.odom.r(0) = r_v;
      Rig rig = makeRig(p, true);
      const auto r = simulate(rig, cfg, p);
      return meanOver(r, 10, 30, [](const Sample & s) {return s.v_est;});
    };
  const double v_small = run(2.5e-3), v_large = run(2.5e-1);
  std::printf("  推定 v（真 1.0 / odom 1.1）: R_odom 2.5e-3 → %.3f,  2.5e-1 → %.3f\n", v_small, v_large);
  EXPECT_NEAR(v_small, 1.1, 0.01) << "R_odom が小さいとオドメトリに張り付く";
  EXPECT_LT(v_large, v_small - 0.01) << "R_odom を大きくすると NDT 側へ寄る";
  EXPECT_GT(v_large, 0.98);
}

TEST(OdomModel, SingleOutlierRejected)
{
  SimConfig cfg;
  cfg.duration = 10.0;
  const auto p = P0();
  Rig rig = makeRig(p, true);
  simulate(rig, cfg, p);
  const VS x_before = rig.core.state();
  const MS P_before = rig.core.covariance();
  // 状態と同じ時刻に入れるので予測は入らず、更新の有無だけが見える
  const ApplyResult o = rig.addOdom(3.0, 0.0, Eigen::Vector2d(2.5e-3, 5.0e-3), rig.core.time());
  EXPECT_EQ(o.stages.at(0).status, SR::kRejected);
  EXPECT_GT(o.stages.at(0).d2, p.odom.gate_threshold);
  EXPECT_EQ(rig.odom->gateState(rig.core).reject_count, 1);
  EXPECT_TRUE(rig.core.state() == x_before) << "外れ値で状態が動かない";
  EXPECT_TRUE(rig.core.covariance() == P_before);
}

TEST(OdomModel, WrongOmegaSignIsNotCaughtByOdomGate)
{
  // 左旋回中にオドメトリの omega の符号が逆。
  // 【既知の制約】オドメトリのゲートでは検出できない。omega の推定はオドメトリに引きずられて符号が逆になり、
  // NDT とのずれは yaw のプロセスノイズとして吸収される（位置は NDT で保たれる）。
  // Q_pose を小さくすると逆に NDT の方が棄却されて発散するので、符号は走らせる前に確認すること。
  SimConfig cfg;
  cfg.duration = 20.0;
  cfg.motion = [](double, double & v, double & w) {v = 0.5; w = 0.5;};
  cfg.odom_corrupt = [](double, double &, double & w) {w = -w;};
  const auto p = P0();
  Rig rig = makeRig(p, true);
  const auto r = simulate(rig, cfg, p);
  const double rate = static_cast<double>(r.odom_rejects) / r.odom_updates;
  const double w_est = meanOver(r, 10, 20, [](const Sample & s) {return s.w_est;});
  const double pos = rmsOver(r, 10, 20, [](const Sample & s) {return s.err_pos;});
  std::printf("  符号誤り: odom 棄却率 %.1f%%  推定 omega %.2f（真 0.5）  位置 rms %.3f m\n",
    100 * rate, w_est, pos);
  EXPECT_LT(rate, 0.05) << "ゲートは符号誤りを弾かない";
  EXPECT_LT(w_est, 0.0) << "omega の推定は誤ったオドメトリに従う";
  EXPECT_LT(pos, 0.15) << "位置は NDT で保たれる";
}

TEST(OdomModel, OdomDropoutFallsBackToNdtOnly)
{
  // 5 s 以降オドメトリが止まる
  SimConfig cfg;
  cfg.duration = 20.0;
  cfg.motion = [](double t, double & v, double & w) {v = 0.8; w = (t > 10.0) ? 0.3 : 0.0;};
  cfg.odom_on = [](double t) {return t < 5.0;};
  const auto p = P0();
  Rig odom = makeRig(p, true);
  Rig ndt_only = makeRig(p, false);
  const auto r1 = simulate(odom, cfg, p);
  const auto r0 = simulate(ndt_only, cfg, p);
  const auto ep = [](const Sample & s) {return s.err_pos;};
  const double e1 = rmsOver(r1, 12, 20, ep), e0 = rmsOver(r0, 12, 20, ep);
  std::printf("  odom 途絶後の位置誤差 rms: ndt_odom %.3f  ndt_only %.3f\n", e1, e0);
  EXPECT_TRUE(r1.psd_always);
  EXPECT_NEAR(e1, e0, 0.02);
  EXPECT_NEAR(r1.samples.back().w_est, 0.3, 0.1);
}

TEST(OdomModel, InitializesVelocityFromRecentOdom)
{
  const auto p = P0();
  const Eigen::Vector2d r(2.5e-3, 5.0e-3);

  Rig recent = makeRig(p, true);
  const ApplyResult o = recent.addOdom(0.8, 0.2, r, 99.9);   // 未初期化：覚えるだけ
  EXPECT_FALSE(o.was_initialized);
  EXPECT_TRUE(o.stages.empty());
  EXPECT_FALSE(recent.core.initialized());
  EXPECT_TRUE(recent.addNdt(lidarObs(p, 0, 0, 0), 100.0).aided);
  EXPECT_DOUBLE_EQ(recent.core.vXy(), 0.8);
  EXPECT_DOUBLE_EQ(recent.core.omegaYaw(), 0.2);
  EXPECT_DOUBLE_EQ(recent.core.covariance()(EkfCore::kVxy, EkfCore::kVxy), r(0));
  EXPECT_DOUBLE_EQ(recent.core.covariance()(EkfCore::kOmegaYaw, EkfCore::kOmegaYaw), r(1));

  Rig stale = makeRig(p, true);
  stale.addOdom(0.8, 0.2, r, 99.0);   // 1 s 前は odom_timeout_init(0.2) を超える
  EXPECT_FALSE(stale.addNdt(lidarObs(p, 0, 0, 0), 100.0).aided);
  EXPECT_DOUBLE_EQ(stale.core.vXy(), 0.0);
  EXPECT_DOUBLE_EQ(
    stale.core.covariance()(EkfCore::kVxy, EkfCore::kVxy), p.core.p_init(EkfCore::kVxy));

  // reset 後も直近のオドメトリで初期化し直せる（初期姿勢の再設定）
  recent.core.reset();
  EXPECT_FALSE(recent.core.initialized());
  recent.addOdom(0.5, -0.1, r, 105.0);
  EXPECT_TRUE(recent.addNdt(lidarObs(p, 0, 0, 0), 105.05).aided);
  EXPECT_DOUBLE_EQ(recent.core.vXy(), 0.5);
}

// ---- OdomNoise（オドメトリの観測ノイズの計算）----

TEST(OdomNoise, FixedWhenAlphaAndTauAreZero)
{
  OdomModel::Params p;
  p.r = Eigen::Vector2d(6.08e-3, 9.92e-4);
  OdomNoise noise(p);
  EXPECT_TRUE(noise.variance(1.0, 0.5, 0.2).isApprox(p.r));
  EXPECT_TRUE(noise.variance(1.02, 1.5, -1.0).isApprox(p.r));
}

TEST(OdomNoise, SpeedTerms)
{
  OdomModel::Params p;
  p.r = Eigen::Vector2d(1.0e-3, 2.0e-3);
  p.alpha = Eigen::Vector4d(0.1, 0.2, 0.3, 0.4);
  OdomNoise noise(p);
  const Eigen::Vector2d r = noise.variance(1.0, 2.0, 3.0);
  EXPECT_NEAR(r(0), 1.0e-3 + 0.1 * 4.0 + 0.2 * 9.0, 1e-12);
  EXPECT_NEAR(r(1), 2.0e-3 + 0.3 * 4.0 + 0.4 * 9.0, 1e-12);
}

TEST(OdomNoise, AccelTermsFromPreviousMessage)
{
  OdomModel::Params p;
  p.r = Eigen::Vector2d(1.0e-3, 2.0e-3);
  p.tau = Eigen::Vector2d(0.5, 0.25);
  OdomNoise noise(p);

  // 最初のメッセージは加速度が分からないので 0
  EXPECT_TRUE(noise.variance(1.0, 1.0, 0.0).isApprox(p.r));

  // dv/dt = 2, domega/dt = -4
  Eigen::Vector2d r = noise.variance(1.5, 2.0, -2.0);
  EXPECT_NEAR(r(0), 1.0e-3 + std::pow(0.5 * 2.0, 2), 1e-12);
  EXPECT_NEAR(r(1), 2.0e-3 + std::pow(0.25 * 4.0, 2), 1e-12);

  // stamp が進まなければ加速度は前の値のまま
  r = noise.variance(1.5, 9.0, 9.0);
  EXPECT_NEAR(r(0), 1.0e-3 + std::pow(0.5 * 2.0, 2), 1e-12);
  EXPECT_NEAR(r(1), 2.0e-3 + std::pow(0.25 * 4.0, 2), 1e-12);
}
