// LaggedEkf（巻き戻し・再計算）の単体テスト（ROS 非依存）
//   colcon build --packages-select ekf_localizer
//   colcon test --packages-select ekf_localizer --event-handlers console_direct+
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <random>
#include <vector>

#include "ekf_localizer/lagged_ekf.hpp"
#include "ekf_localizer/ndt_model.hpp"
#include "ekf_localizer/odom_model.hpp"

using ekf_localizer::applyMeasurement;
using ekf_localizer::EkfCore;
using ekf_localizer::LaggedEkf;
using ekf_localizer::NdtModel;
using ekf_localizer::OdomModel;
using V6 = NdtModel::Vector6d;
using Status = LaggedEkf::Status;

namespace
{

constexpr double kT0 = 100.0;
constexpr double kV = 1.0;
constexpr double kOmega = 0.2;
const Eigen::Vector2d kROdom(2.5e-3, 5.0e-3);

/// 取付 base → LiDAR
Eigen::Affine3d mount()
{
  Eigen::Affine3d T = Eigen::Affine3d::Identity();
  T.linear() = Eigen::AngleAxisd(-0.017, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  T.translation() << -0.17, 0.0, 1.2;
  return T;
}

/// EKF 本体と観測モデル（use_odom が false ならオドメトリの観測モデルなし）
struct Filter
{
  EkfCore core;
  std::shared_ptr<NdtModel> ndt;
  std::shared_ptr<OdomModel> odom;
};

Filter makeFilter(bool use_odom = true)
{
  EkfCore::Params cp;
  cp.q << 5e-2, 5e-2, 5e-3, 5e-2, 5e-2, 5e-2, 0.05, 0.1;
  cp.p_init << 2e-2, 2e-2, 2e-2, 0.06, 0.06, 2e-3, 1.0, 1.0;
  NdtModel::Params np;
  np.r << 1e-2, 1e-2, 1e-2, 0.03, 0.03, 1e-3;
  Filter f{EkfCore(cp), nullptr, nullptr};
  f.ndt = std::make_shared<NdtModel>(np, mount(), f.core);
  if (use_odom) {f.odom = std::make_shared<OdomModel>(OdomModel::Params(), f.core);}
  return f;
}

/// 等速円運動の真値（map → base）
Eigen::Affine3d truth(double t)
{
  const double s = t - kT0;
  const double yaw = kOmega * s;
  Eigen::Affine3d T = Eigen::Affine3d::Identity();
  T.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  T.translation() << kV / kOmega * std::sin(yaw), kV / kOmega * (1.0 - std::cos(yaw)), 0.0;
  return T;
}

/// 姿勢 → [x, y, z, roll, pitch, yaw]（ZYX）
V6 poseVec(const Eigen::Affine3d & T)
{
  const Eigen::Matrix3d R = T.linear();
  V6 z;
  z << T.translation(), std::atan2(R(2, 1), R(2, 2)), std::asin(-R(2, 0)),
    std::atan2(R(1, 0), R(0, 0));
  return z;
}

struct Meas
{
  bool ndt{false};
  double stamp{0.0};
  /// LaggedEkf に届く時刻
  double arrival{0.0};
  V6 z{V6::Zero()};
  double v{0.0};
  double omega{0.0};
};

/// 50 Hz のオドメトリと 10 Hz の NDT（ndt_delay 秒遅れて届く）を duration 秒ぶん作る
std::vector<Meas> scenario(double ndt_delay, double duration = 2.0, unsigned seed = 1)
{
  std::mt19937 rng(seed);
  std::normal_distribution<double> n(0.0, 1.0);
  std::vector<Meas> ms;
  for (int i = 0; i * 0.02 <= duration; ++i) {
    Meas m;
    m.stamp = m.arrival = kT0 + i * 0.02;
    m.v = kV + 0.02 * n(rng);
    m.omega = kOmega + 0.02 * n(rng);
    ms.push_back(m);
  }
  // NDT はオドメトリと同じ時刻にならないよう 5 ms ずらす
  for (int k = 0; k * 0.1 <= duration; ++k) {
    Meas m;
    m.ndt = true;
    m.stamp = kT0 + k * 0.1 + 0.005;
    m.arrival = m.stamp + ndt_delay;
    m.z = poseVec(truth(m.stamp) * mount());
    m.z(0) += 0.05 * n(rng);
    m.z(1) += 0.05 * n(rng);
    m.z(5) += 0.01 * n(rng);
    ms.push_back(m);
  }
  return ms;
}

ekf_localizer::MeasurementPtr toMeasurement(const Filter & f, const Meas & m)
{
  return m.ndt ? f.ndt->measurement(m.z, m.stamp) :
         f.odom->measurement(m.v, m.omega, kROdom, m.stamp);
}

/// 時刻順に並べ直して、巻き戻しなしの EKF にそのまま適用した結果（正解）
EkfCore runInOrder(std::vector<Meas> ms, bool use_odom = true)
{
  std::stable_sort(
    ms.begin(), ms.end(), [](const Meas & a, const Meas & b) {return a.stamp < b.stamp;});
  Filter f = makeFilter(use_odom);
  for (const Meas & m : ms) {
    applyMeasurement(f.core, toMeasurement(f, m));
  }
  return f.core;
}

/// 届いた順に LaggedEkf に入れる。巻き戻しが起きた回数を返す
int runLagged(LaggedEkf & lagged, const Filter & f, std::vector<Meas> ms)
{
  std::stable_sort(
    ms.begin(), ms.end(), [](const Meas & a, const Meas & b) {return a.arrival < b.arrival;});
  int replays = 0;
  for (const Meas & m : ms) {
    const Status s = lagged.add(toMeasurement(f, m)).status;
    EXPECT_NE(s, Status::kTooOld);
    if (s == Status::kReplayed) {++replays;}
  }
  return replays;
}

void expectSameState(const EkfCore & a, const EkfCore & b, double tol = 1e-9)
{
  ASSERT_TRUE(a.initialized());
  ASSERT_TRUE(b.initialized());
  EXPECT_NEAR(a.lastStamp(), b.lastStamp(), 1e-12);
  EXPECT_LT((a.state() - b.state()).cwiseAbs().maxCoeff(), tol);
  EXPECT_LT((a.covariance() - b.covariance()).cwiseAbs().maxCoeff(), tol);
}

}  // namespace

TEST(LaggedEkf, InOrderMatchesPlainEkf)
{
  const auto ms = scenario(0.0);
  const Filter f = makeFilter();
  LaggedEkf lagged(f.core, 1.0);
  EXPECT_EQ(runLagged(lagged, f, ms), 0);
  expectSameState(lagged.current(), runInOrder(ms));
}

TEST(LaggedEkf, DelayedNdtMatchesInOrder)
{
  // NDT の計算に 0.15 s かかる：その間のオドメトリ 7〜8 個を毎回適用し直す
  const auto ms = scenario(0.15);
  const Filter f = makeFilter();
  LaggedEkf lagged(f.core, 1.0);
  EXPECT_GT(runLagged(lagged, f, ms), 15);
  expectSameState(lagged.current(), runInOrder(ms));
  // 推定も真値に追従している
  const Eigen::Affine3d T = lagged.current().basePose();
  const Eigen::Affine3d T_true = truth(lagged.current().lastStamp());
  EXPECT_LT((T.translation() - T_true.translation()).head<2>().norm(), 0.1);
  EXPECT_NEAR(lagged.current().velocity(), kV, 0.1);
}

TEST(LaggedEkf, OutOfOrderOdomMatchesInOrder)
{
  // オドメトリが 2 個ずつ入れ替わって届く
  auto ms = scenario(0.15);
  for (std::size_t i = 0; i + 1 < ms.size(); i += 2) {
    if (!ms[i].ndt && !ms[i + 1].ndt) {std::swap(ms[i].arrival, ms[i + 1].arrival);}
  }
  const Filter f = makeFilter();
  LaggedEkf lagged(f.core, 1.0);
  runLagged(lagged, f, ms);
  expectSameState(lagged.current(), runInOrder(ms));
}

TEST(LaggedEkf, InitializesFromDelayedNdtAndCatchesUp)
{
  const Filter f = makeFilter();
  LaggedEkf lagged(f.core, 1.0);
  for (int i = 0; i <= 15; ++i) {
    const auto r = lagged.add(f.odom->measurement(kV, kOmega, kROdom, kT0 + i * 0.02));
    EXPECT_FALSE(r.outcome.was_initialized);
  }
  EXPECT_FALSE(lagged.initialized());

  // t0 + 0.105 の NDT が t0 + 0.3 に届く
  const auto r = lagged.add(
    f.ndt->measurement(poseVec(truth(kT0 + 0.105) * mount()), kT0 + 0.105));
  EXPECT_EQ(r.status, Status::kReplayed);
  EXPECT_TRUE(r.outcome.initialized_now);
  EXPECT_TRUE(r.outcome.aided);
  EXPECT_EQ(r.replayed, 10u);   // t0 + 0.12 〜 t0 + 0.30 のオドメトリ
  ASSERT_TRUE(lagged.initialized());
  // 最新のオドメトリの時刻まで追いつき、速度は初期化前のオドメトリから始まっている
  EXPECT_NEAR(lagged.current().lastStamp(), kT0 + 0.30, 1e-12);
  EXPECT_NEAR(lagged.current().velocity(), kV, 1e-3);
  const Eigen::Affine3d T_true = truth(kT0 + 0.30);
  EXPECT_LT(
    (lagged.current().basePose().translation() - T_true.translation()).norm(), 1e-2);
}

TEST(LaggedEkf, TooOldMeasurementIsDropped)
{
  const auto ms = scenario(0.0, 2.0);
  const Filter f = makeFilter();
  LaggedEkf lagged(f.core, 0.5);
  runLagged(lagged, f, ms);
  // 履歴は最新から 0.5 s ぶん（50 Hz ＋ 10 Hz）程度に収まっている
  EXPECT_LE(lagged.historySize(), 33u);

  const EkfCore::StateVector x_before = lagged.current().state();
  const auto r = lagged.add(f.ndt->measurement(ms.back().z, kT0 + 1.0));   // 1 s 前（履歴の外）
  EXPECT_EQ(r.status, Status::kTooOld);
  const auto o = lagged.add(f.odom->measurement(kV, kOmega, kROdom, kT0 + 1.0));
  EXPECT_EQ(o.status, Status::kTooOld);
  EXPECT_EQ(lagged.current().state(), x_before);
}

TEST(LaggedEkf, PredictedDoesNotChangeCurrent)
{
  const Filter f = makeFilter();
  LaggedEkf lagged(f.core, 1.0);
  runLagged(lagged, f, scenario(0.0, 1.0));
  const EkfCore::StateVector x = lagged.current().state();
  const double t = lagged.current().lastStamp();

  const EkfCore pred = lagged.predicted(t + 0.05);
  EXPECT_EQ(lagged.current().state(), x);
  EXPECT_NEAR(pred.lastStamp(), t + 0.05, 1e-12);
  EkfCore ref = lagged.current();
  ref.predictTo(t + 0.05);
  EXPECT_EQ(pred.state(), ref.state());
  // 前進している
  EXPECT_GT((pred.basePose().translation() - lagged.current().basePose().translation()).norm(),
    0.03);
}

TEST(LaggedEkf, ResetWaitsForNextNdt)
{
  const auto ms = scenario(0.15, 1.0);
  const Filter f = makeFilter();
  LaggedEkf lagged(f.core, 1.0);
  runLagged(lagged, f, ms);
  ASSERT_TRUE(lagged.initialized());

  lagged.reset();
  EXPECT_FALSE(lagged.initialized());
  EXPECT_EQ(lagged.historySize(), 0u);
  const double t = kT0 + 1.2;
  EXPECT_FALSE(
    lagged.add(f.odom->measurement(kV, kOmega, kROdom, t)).outcome.was_initialized);
  EXPECT_FALSE(lagged.initialized());

  const auto r = lagged.add(f.ndt->measurement(poseVec(truth(t + 0.01) * mount()), t + 0.01));
  EXPECT_TRUE(r.outcome.initialized_now);
  EXPECT_TRUE(lagged.initialized());
}

TEST(LaggedEkf, NdtOnlyReordersDelayedNdt)
{
  // オドメトリの観測モデルがなくても、遅れて届いた NDT を時刻順に入れ直す
  const auto ms = scenario(0.15, 1.0);
  std::vector<Meas> ndt_only;
  for (const Meas & m : ms) {
    if (m.ndt) {ndt_only.push_back(m);}
  }
  std::swap(ndt_only[3].arrival, ndt_only[4].arrival);
  const Filter f = makeFilter(false);
  LaggedEkf lagged(f.core, 1.0);
  runLagged(lagged, f, ndt_only);
  expectSameState(lagged.current(), runInOrder(ndt_only, false));
}
