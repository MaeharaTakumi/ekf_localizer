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

using ekf_localizer::LaggedEkf;
using ekf_localizer::VehicleEkf;
using ekf_localizer::VehicleOdomEkf;
using V6 = VehicleEkf::Vector6d;
using Status = LaggedEkf::Status;

namespace
{

constexpr double kT0 = 100.0;
constexpr double kV = 1.0;
constexpr double kOmega = 0.2;
const Eigen::Vector2d kROdom(2.5e-3, 5.0e-3);

VehicleEkf::Params params()
{
  VehicleEkf::Params p;
  p.o_x = -0.17; p.o_z = 1.2; p.yaw_o = -0.017;
  p.q << 5e-2, 5e-2, 5e-3, 5e-2, 5e-2, 5e-2, 0.05, 0.1;
  p.r_ndt << 1e-2, 1e-2, 1e-2, 0.03, 0.03, 1e-3;
  p.p_init << 2e-2, 2e-2, 2e-2, 0.06, 0.06, 2e-3, 1.0, 1.0;
  return p;
}

std::unique_ptr<VehicleOdomEkf> makeOdomEkf()
{
  auto ekf = std::make_unique<VehicleOdomEkf>();
  ekf->setParams(params());
  ekf->reset();
  return ekf;
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
  VehicleEkf proto;
  proto.setParams(params());
  const Eigen::Affine3d mount = proto.mountTransform();

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
    m.z = poseVec(truth(m.stamp) * mount);
    m.z(0) += 0.05 * n(rng);
    m.z(1) += 0.05 * n(rng);
    m.z(5) += 0.01 * n(rng);
    ms.push_back(m);
  }
  return ms;
}

/// 時刻順に並べ直して、普通の EKF にそのまま適用した結果（正解）
std::unique_ptr<VehicleOdomEkf> runInOrder(std::vector<Meas> ms)
{
  std::stable_sort(
    ms.begin(), ms.end(), [](const Meas & a, const Meas & b) {return a.stamp < b.stamp;});
  auto ekf = makeOdomEkf();
  for (const Meas & m : ms) {
    if (!m.ndt) {
      ekf->updateOdom(m.v, m.omega, kROdom, m.stamp);
    } else if (!ekf->initialized()) {
      ekf->initialize(m.z, m.stamp);
    } else {
      ekf->predictTo(m.stamp);
      ekf->update(m.z);
    }
  }
  return ekf;
}

/// 届いた順に LaggedEkf に入れる。巻き戻しが起きた回数を返す
int runLagged(LaggedEkf & lagged, std::vector<Meas> ms)
{
  std::stable_sort(
    ms.begin(), ms.end(), [](const Meas & a, const Meas & b) {return a.arrival < b.arrival;});
  int replays = 0;
  for (const Meas & m : ms) {
    const Status s = m.ndt ? lagged.addNdt(m.z, m.stamp).status :
      lagged.addOdom(m.v, m.omega, kROdom, m.stamp).status;
    EXPECT_NE(s, Status::kTooOld);
    if (s == Status::kReplayed) {++replays;}
  }
  return replays;
}

void expectSameState(const VehicleEkf & a, const VehicleEkf & b, double tol = 1e-9)
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
  LaggedEkf lagged(makeOdomEkf(), 1.0);
  EXPECT_EQ(runLagged(lagged, ms), 0);
  expectSameState(lagged.current(), *runInOrder(ms));
}

TEST(LaggedEkf, DelayedNdtMatchesInOrder)
{
  // NDT の計算に 0.15 s かかる：その間のオドメトリ 7〜8 個を毎回適用し直す
  const auto ms = scenario(0.15);
  LaggedEkf lagged(makeOdomEkf(), 1.0);
  EXPECT_GT(runLagged(lagged, ms), 15);
  expectSameState(lagged.current(), *runInOrder(ms));
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
  LaggedEkf lagged(makeOdomEkf(), 1.0);
  runLagged(lagged, ms);
  expectSameState(lagged.current(), *runInOrder(ms));
}

TEST(LaggedEkf, InitializesFromDelayedNdtAndCatchesUp)
{
  LaggedEkf lagged(makeOdomEkf(), 1.0);
  for (int i = 0; i <= 15; ++i) {
    const auto r = lagged.addOdom(kV, kOmega, kROdom, kT0 + i * 0.02);
    EXPECT_FALSE(r.filter_initialized);
  }
  EXPECT_FALSE(lagged.initialized());

  // t0 + 0.105 の NDT が t0 + 0.3 に届く
  VehicleEkf proto;
  proto.setParams(params());
  const auto r = lagged.addNdt(poseVec(truth(kT0 + 0.105) * proto.mountTransform()), kT0 + 0.105);
  EXPECT_EQ(r.status, Status::kReplayed);
  EXPECT_TRUE(r.initialized);
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
  LaggedEkf lagged(makeOdomEkf(), 0.5);
  runLagged(lagged, ms);
  // 履歴は最新から 0.5 s ぶん（50 Hz ＋ 10 Hz）程度に収まっている
  EXPECT_LE(lagged.historySize(), 33u);

  const VehicleEkf::Vector8d x_before = lagged.current().state();
  const auto r = lagged.addNdt(ms.back().z, kT0 + 1.0);   // 1 s 前（履歴の外）
  EXPECT_EQ(r.status, Status::kTooOld);
  const auto o = lagged.addOdom(kV, kOmega, kROdom, kT0 + 1.0);
  EXPECT_EQ(o.status, Status::kTooOld);
  EXPECT_EQ(lagged.current().state(), x_before);
}

TEST(LaggedEkf, PredictedDoesNotChangeCurrent)
{
  LaggedEkf lagged(makeOdomEkf(), 1.0);
  runLagged(lagged, scenario(0.0, 1.0));
  const VehicleEkf::Vector8d x = lagged.current().state();
  const double t = lagged.current().lastStamp();

  const auto pred = lagged.predicted(t + 0.05);
  EXPECT_EQ(lagged.current().state(), x);
  EXPECT_NEAR(pred->lastStamp(), t + 0.05, 1e-12);
  auto ref = lagged.current().clone();
  ref->predictTo(t + 0.05);
  EXPECT_EQ(pred->state(), ref->state());
  // 前進している
  EXPECT_GT((pred->basePose().translation() - lagged.current().basePose().translation()).norm(),
    0.03);
}

TEST(LaggedEkf, ResetWaitsForNextNdt)
{
  const auto ms = scenario(0.15, 1.0);
  LaggedEkf lagged(makeOdomEkf(), 1.0);
  runLagged(lagged, ms);
  ASSERT_TRUE(lagged.initialized());

  lagged.reset();
  EXPECT_FALSE(lagged.initialized());
  EXPECT_EQ(lagged.historySize(), 0u);
  const double t = kT0 + 1.2;
  EXPECT_FALSE(lagged.addOdom(kV, kOmega, kROdom, t).filter_initialized);
  EXPECT_FALSE(lagged.initialized());

  VehicleEkf proto;
  proto.setParams(params());
  const auto r = lagged.addNdt(poseVec(truth(t + 0.01) * proto.mountTransform()), t + 0.01);
  EXPECT_TRUE(r.initialized);
  EXPECT_TRUE(lagged.initialized());
}

TEST(LaggedEkf, NdtOnlyIgnoresOdom)
{
  auto ekf = std::make_unique<VehicleEkf>();
  ekf->setParams(params());
  ekf->reset();
  LaggedEkf lagged(std::move(ekf), 1.0);
  EXPECT_FALSE(lagged.usesOdom());
  EXPECT_EQ(lagged.addOdom(kV, kOmega, kROdom, kT0).status, Status::kIgnored);
  EXPECT_EQ(lagged.historySize(), 0u);

  // NDT だけでも遅れて届いたものを時刻順に入れ直す
  const auto ms = scenario(0.15, 1.0);
  std::vector<Meas> ndt_only;
  for (const Meas & m : ms) {
    if (m.ndt) {ndt_only.push_back(m);}
  }
  std::swap(ndt_only[3].arrival, ndt_only[4].arrival);
  runLagged(lagged, ndt_only);
  auto ref = std::make_unique<VehicleEkf>();
  ref->setParams(params());
  ref->reset();
  for (const Meas & m : ndt_only) {
    if (!ref->initialized()) {
      ref->initialize(m.z, m.stamp);
    } else {
      ref->predictTo(m.stamp);
      ref->update(m.z);
    }
  }
  expectSameState(lagged.current(), *ref);
}
