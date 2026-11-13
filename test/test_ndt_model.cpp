// NdtModel（NDT の解の観測モデル）の単体テスト（ROS 非依存）
//   colcon build --packages-select ekf_localizer
//   colcon test --packages-select ekf_localizer --event-handlers console_direct+
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>

#include "test_helpers.hpp"

using namespace test_helpers;  // NOLINT(build/namespaces)

// ===========================================================================
// NdtModel（NDT のみ）
// ===========================================================================

TEST(NdtModel, LidarPoseJacobianMatchesAnalytic)
{
  // /ekf_pose の共分散に使う LiDAR の姿勢のヤコビアンの位置部分を解析式と照合する：p_L = p_b + R(roll, pitch, yaw) o
  const auto p = P0();
  Rig rig = makeRig(p, false);
  VS x;
  x << 1.0, -2.0, 0.3, 0.07, -0.12, 2.9, 0.9, -0.05, 0.02, -0.03, 0.3;
  const auto H = rig.ndt->lidarPoseJacobian(x);
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
  EXPECT_LT(H.rightCols<EkfCore::kStateSize - 6>().cwiseAbs().maxCoeff(), 1e-12) << "速度に依存しない";

  // 水平面（roll = pitch = 0）では旧 5 状態モデルの H と一致する
  VS xf = x;
  xf(3) = 0.0; xf(4) = 0.0;
  const auto Hf = rig.ndt->lidarPoseJacobian(xf);
  const double c = std::cos(y), s = std::sin(y);
  EXPECT_NEAR(Hf(0, 5), -p.o_x * s - p.o_y * c, 1e-8);
  EXPECT_NEAR(Hf(1, 5), p.o_x * c - p.o_y * s, 1e-8);
  EXPECT_NEAR(Hf(5, 5), 1.0, 1e-8);
}

TEST(NdtModel, RecoversBaseLinkWhileTurning)
{
  const auto p = P0();
  Rig rig = makeRig(p, false);
  const EkfCore & ekf = rig.core;
  std::mt19937 rng(3);
  std::normal_distribution<double> np(0, 0.1), na(0, 0.07), nz(0, 0.1), nr(0, 0.03);
  double t = 1000, xb = 0, yb = 0, psib = 0.5;
  const double V = 0.8, W = 0.3, DT = 0.05;
  rig.addNdt(lidarObs(p, xb, yb, psib), t);
  double se = 0, roll_sum = 0, pitch_sum = 0;
  int n = 0;
  for (int k = 0; k < 800; ++k) {
    t += DT;
    xb += V / W * (std::sin(psib + W * DT) - std::sin(psib));
    yb -= V / W * (std::cos(psib + W * DT) - std::cos(psib));
    psib = normalizeAngle(psib + W * DT);
    V6 z = lidarObs(p, xb, yb, psib);
    z(0) += np(rng); z(1) += np(rng); z(2) += nz(rng); z(3) += nr(rng); z(4) += nr(rng);
    z(5) += na(rng);
    rig.addNdt(z, t);
    if (k > 300) {
      const auto b = ekf.basePose().translation();
      se += std::hypot(b.x() - xb, b.y() - yb);
      roll_sum += ekf.state()(EkfCore::kRoll);
      pitch_sum += ekf.state()(EkfCore::kPitch);
      ++n;
    }
  }
  EXPECT_NEAR(ekf.vXy(), V, 0.15);
  EXPECT_NEAR(ekf.omegaYaw(), W, 0.1);
  EXPECT_LT(se / n, 0.09) << "base_link 位置誤差 < 生観測ノイズ(約0.13m)の7割";
  EXPECT_NEAR(ekf.basePose().translation().z(), 0.0, 0.05) << "LiDAR z 1.2 - o_z 1.2";
  // 1 サンプルは観測ノイズ（sd 0.03 rad）並みに揺れるので、区間平均でバイアスが無いことを見る
  EXPECT_NEAR(roll_sum / n, 0.0, 0.005) << "水平な床では車体 roll = 0";
  EXPECT_NEAR(pitch_sum / n, 0.0, 0.005) << "水平な床では車体 pitch = 0";
  EXPECT_TRUE(psdOK(ekf.covariance()));
}

TEST(NdtModel, BaseObservationRoundTrip)
{
  // LiDAR の姿勢を base の姿勢に直す変換（T_L · T_BL⁻¹）は厳密
  const auto p = P0();
  Rig rig = makeRig(p, false);
  const Eigen::Affine3d T_mb = makePose(3.0, -1.0, 0.4, 0.05, -0.08, 2.0);
  V6 d = rig.ndt->basePoseFromLidar(lidarObs3d(*rig.ndt, T_mb)) - poseVec(T_mb);
  for (int i = 3; i < 6; ++i) {d(i) = normalizeAngle(d(i));}
  EXPECT_LT(d.cwiseAbs().maxCoeff(), 1e-12);
}

TEST(NdtModel, BaseCovarianceMatchesSampling)
{
  // LiDAR の姿勢に R_ndt のノイズを足して base に直したときのばらつきと、J R_ndt Jᵀ が合う
  auto p = P0();
  p.ndt.r << 1e-4, 4e-4, 2e-4, 1e-5, 1e-5, 2e-4;
  Rig rig = makeRig(p, false);
  const V6 zl = lidarObs3d(*rig.ndt, makePose(3.0, -1.0, 0.4, 0.03, -0.05, 2.0));
  const auto R = rig.ndt->baseCovariance(zl, p.ndt.r);
  const V6 zb = rig.ndt->basePoseFromLidar(zl);
  std::mt19937 rng(7);
  std::normal_distribution<double> n01(0.0, 1.0);
  Eigen::Matrix<double, 6, 6> C = Eigen::Matrix<double, 6, 6>::Zero();
  const int N = 200000;
  for (int k = 0; k < N; ++k) {
    V6 z = zl;
    for (int i = 0; i < 6; ++i) {z(i) += std::sqrt(p.ndt.r(i)) * n01(rng);}
    V6 d = rig.ndt->basePoseFromLidar(z) - zb;
    for (int i = 3; i < 6; ++i) {d(i) = normalizeAngle(d(i));}
    C += d * d.transpose();
  }
  C /= N;
  for (int i = 0; i < 6; ++i) {
    EXPECT_NEAR(C(i, i), R(i, i), 0.03 * R(i, i)) << "i=" << i;
    for (int j = 0; j < i; ++j) {
      EXPECT_NEAR(
        C(i, j) / std::sqrt(C(i, i) * C(j, j)), R(i, j) / std::sqrt(R(i, i) * R(j, j)), 0.02)
        << "i=" << i << " j=" << j;
    }
  }
  EXPECT_TRUE(psdOK(R));

  // 姿勢の誤差が取付のレバーアーム（高さ 1.2 m）で base の位置に回る。roll・pitch の分散が大きい今の R_ndt
  // （位置 0.01、roll・pitch 0.03）では、base の水平位置の分散が LiDAR の何倍にもなり、roll・pitch と強く相関する
  V6 r_now;
  r_now << 1e-2, 1e-2, 1e-2, 0.03, 0.03, 1e-3;
  const auto Rn = rig.ndt->baseCovariance(zl, r_now);
  EXPECT_GT(Rn(0, 0) + Rn(1, 1), 3.0 * (r_now(0) + r_now(1)));
  double max_corr = 0.0;
  for (int i : {0, 1}) {
    for (int j : {3, 4}) {max_corr = std::max(max_corr, std::abs(Rn(i, j)) / std::sqrt(Rn(i, i) * Rn(j, j)));}
  }
  EXPECT_GT(max_corr, 0.5) << "水平位置と roll・pitch の相関";
}

TEST(NdtModel, UpdateMatchesLidarFrameUpdate)
{
  // base の姿勢に直して更新した結果は、LiDAR の姿勢のまま（h(x) = T_B(x) T_BL、R = diag(R_ndt)）で
  // 6 成分を 1 回で更新した結果と 1 次で一致する（違いは線形化する点が観測か予測かだけ）
  const auto p = P0();
  Rig rig = makeRig(p, false);
  double t = 1000;
  const Eigen::Affine3d T0 = makePose(2.0, 1.0, 0.0, 0.0, 0.0, 0.7);
  rig.addNdt(lidarObs3d(*rig.ndt, T0), t);
  t += 0.1;
  rig.core.predictTo(t);
  const VS x0 = rig.core.state();
  const MS P = rig.core.covariance();
  V6 z = lidarObs3d(*rig.ndt, T0);
  z(0) += 0.02; z(1) -= 0.01; z(2) += 0.01; z(3) += 0.003; z(4) -= 0.002; z(5) += 0.01;

  V6 y = z - poseVec(rig.ndt->lidarPose(x0));
  for (int i = 3; i < 6; ++i) {y(i) = normalizeAngle(y(i));}
  const auto H = rig.ndt->lidarPoseJacobian(x0);
  const Eigen::Matrix<double, 6, 6> S = H * P * H.transpose() + Eigen::Matrix<double, 6, 6>(p.ndt.r.asDiagonal());
  const VS x_lidar = x0 + P * H.transpose() * S.inverse() * y;

  const auto r = rig.updateNdt(z, t);
  ASSERT_EQ(r.size(), 1u);
  EXPECT_EQ(r[0].status, SR::kUpdated);
  VS d = rig.core.state() - x_lidar;
  for (int i = 3; i < 6; ++i) {d(i) = normalizeAngle(d(i));}
  VS corr = x_lidar - x0;
  std::printf(
    "  LiDAR の座標の更新との差：位置 %.2e m・姿勢 %.2e rad（更新量 %.2e m・%.2e rad）\n",
    d.head<3>().cwiseAbs().maxCoeff(), d.segment<3>(3).cwiseAbs().maxCoeff(),
    corr.head<3>().cwiseAbs().maxCoeff(), corr.segment<3>(3).cwiseAbs().maxCoeff());
  EXPECT_LT(d.head<3>().cwiseAbs().maxCoeff(), 0.05 * corr.head<3>().cwiseAbs().maxCoeff());
  EXPECT_LT(d.segment<3>(3).cwiseAbs().maxCoeff(), 0.05 * corr.segment<3>(3).cwiseAbs().maxCoeff());
  EXPECT_TRUE(psdOK(rig.core.covariance()));
}

TEST(NdtModel, WholePoseRejectedWhenOneComponentJumps)
{
  // 6 成分をまとめて判定するので、z だけが飛んでも観測全体を使わない（状態は変わらない）
  const auto p = P0();
  Rig rig = makeRig(p, false);
  double t = 1000;
  rig.addNdt(lidarObs(p, 0, 0, 0), t);
  for (int k = 0; k < 100; ++k) {t += 0.05; rig.addNdt(lidarObs(p, 0, 0, 0), t);}
  t += 0.05;
  rig.core.predictTo(t);
  const VS before = rig.core.state();
  V6 bad = lidarObs(p, 0, 0, 0);
  bad(2) += 0.8;
  const auto r = rig.updateNdt(bad, t);
  ASSERT_EQ(r.size(), 1u);
  EXPECT_EQ(r[0].status, SR::kRejected);
  EXPECT_GT(r[0].d2, p.ndt.gate_threshold);
  EXPECT_EQ(rig.ndt->gateState(rig.core).reject_count, 1);
  EXPECT_LT((rig.core.state() - before).cwiseAbs().maxCoeff(), 1e-12);
  // 次の正しい観測は通り、連続棄却の回数は 0 に戻る
  t += 0.05;
  EXPECT_EQ(rig.updateNdt(lidarObs(p, 0, 0, 0), t).at(0).status, SR::kUpdated);
  EXPECT_EQ(rig.ndt->gateState(rig.core).reject_count, 0);
}

TEST(NdtModel, StaticTfConsistent)
{
  const auto p = P0();
  Rig rig = makeRig(p, false);
  rig.addNdt(lidarObs(p, 4.0, -1.0, 1.1), 0.0);
  const Eigen::Affine3d d =
    lidarPose(rig).inverse() * (rig.core.basePose() * rig.ndt->mountTransform());
  EXPECT_LT((d.matrix() - Eigen::Matrix4d::Identity()).cwiseAbs().maxCoeff(), 1e-12);
  const Eigen::Vector3d bt = rig.core.basePose().translation();
  EXPECT_LT(std::hypot(bt.x() - rig.core.state()(0), bt.y() - rig.core.state()(1)), 1e-9);
}

TEST(NdtModel, InitializeRoundTrip)
{
  // roll, pitch, 取付回転がすべて 0 でない LiDAR 姿勢で初期化し、lidarPose() が観測に戻る
  const auto p = P0();
  Rig rig = makeRig(p, false);
  const Eigen::Affine3d T_mb = makePose(3.0, -1.0, 0.4, 0.05, -0.08, 2.0);
  const V6 z = lidarObs3d(*rig.ndt, T_mb);
  EXPECT_TRUE(rig.addNdt(z, 0.0).initialized_now);
  const V6 back = poseVec(lidarPose(rig));
  V6 d = back - z;
  for (int i = 3; i < 6; ++i) {d(i) = normalizeAngle(d(i));}
  EXPECT_LT(d.cwiseAbs().maxCoeff(), 1e-9);
  EXPECT_LT((rig.core.state().head<6>() - poseVec(T_mb)).cwiseAbs().maxCoeff(), 1e-9)
    << "状態は base_link の姿勢";
  EXPECT_TRUE(psdOK(rig.core.covariance()));
  // 初期共分散は P_init の対角
  EXPECT_LT((rig.core.covariance() - MS(p.core.p_init.asDiagonal())).cwiseAbs().maxCoeff(), 1e-15);
}

// ---------------------------------------------------------------------------
// 傾斜。1/12 勾配を 1 m/s で直進して登る
// ---------------------------------------------------------------------------
namespace
{

struct SlopeResult
{
  double bias_xy;       // 収束後の base 水平位置誤差の符号つき平均の大きさ [m]
  double rms_xy;        // 収束後の base 水平位置誤差の rms [m]
  double bias_pitch;    // 収束後の車体 pitch 推定の平均誤差 [rad]
  double z_err_end;     // 最後の base z 誤差 [m]
  double z_held;        // NDT を止めてから最後までの base z の推定の変化 [m]（ndt_off_from >= 0 のとき）
};

/// ndt_off_from 以降 NDT を止める（負なら止めない）。noise = 0 で観測ノイズなし。
/// q_vz > 0 なら v_z を推定する（Q と P_init に q_vz を入れる）
SlopeResult runSlope(double duration, double ndt_off_from, double noise = 1.0, double q_vz = 0.0)
{
  auto p = P0();
  p.ndt.r << 1e-2, 1e-2, 1e-2, 1e-4, 1e-4, 1e-3;
  p.core.q(EkfCore::kVz) = q_vz;
  p.core.p_init(EkfCore::kVz) = q_vz;
  Rig rig = makeRig(p, false);
  EkfCore & ekf = rig.core;
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

  double t0 = 1000.0, sq = 0.0, pitch_err = 0.0, z_off = 0.0;
  bool off_seen = false;
  Eigen::Vector2d sum = Eigen::Vector2d::Zero();
  int n = 0;
  SlopeResult res{};
  for (int k = 0; k * DT <= duration; ++k) {
    const double tr = k * DT, t = t0 + tr;
    const Eigen::Affine3d T = truth(tr);
    if (k % 4 == 0 && (ndt_off_from < 0 || tr < ndt_off_from)) {
      rig.addNdt(noisy(lidarObs3d(*rig.ndt, T)), t);
    }
    if (!ekf.initialized()) {continue;}
    ekf.predictTo(t);
    const Eigen::Vector3d e = ekf.basePose().translation() - T.translation();
    if (tr > 5.0 && (ndt_off_from < 0 || tr < ndt_off_from)) {
      sum += e.head<2>();
      sq += e.head<2>().squaredNorm();
      pitch_err += ekf.state()(EkfCore::kPitch) + slope;
      ++n;
    }
    res.z_err_end = std::abs(e.z());
    if (ndt_off_from >= 0 && tr >= ndt_off_from) {
      if (!off_seen) {z_off = ekf.basePose().translation().z(); off_seen = true;}
      res.z_held = std::abs(ekf.basePose().translation().z() - z_off);
    }
  }
  res.bias_xy = (sum / n).norm();
  res.rms_xy = std::sqrt(sq / n);
  res.bias_pitch = pitch_err / n;
  return res;
}

}  // namespace

TEST(NdtModel, SlopeHasNoLeverArmBias)
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

TEST(NdtModel, SlopeZHeldDuringNdtOutage)
{
  // v_z の Q・P_init が 0 なら v_z は 0 のままで、予測では z を動かさない。10 s から 2 s 欠測すると、
  // 坂では z が v sin(slope) * 2 s = 0.17 m 遅れる（NDT が戻れば観測で直る）
  const auto r = runSlope(12.0, 10.0);
  std::printf("  傾斜 1/12・NDT 2 s 欠測：z の推定の変化 %.2e m、最後の z 誤差 %.3f m\n", r.z_held, r.z_err_end);
  EXPECT_LT(r.z_held, 1e-9) << "欠測中の予測で z は動かない";
  EXPECT_GT(r.z_err_end, 0.12);
}

TEST(NdtModel, SlopeZFollowsVzDuringNdtOutage)
{
  // v_z を推定すると（Q > 0）、欠測中も z は v_z で坂に沿って進む（真の v_z = v sin(slope) = 0.083 m/s）
  const auto r = runSlope(12.0, 10.0, 1.0, 1.0e-2);
  std::printf("  傾斜 1/12・NDT 2 s 欠測・v_z あり：z の推定の変化 %.3f m、最後の z 誤差 %.3f m\n",
    r.z_held, r.z_err_end);
  EXPECT_GT(r.z_held, 0.12) << "欠測中も z は v_z で進む（2 s で 0.17 m）";
  EXPECT_LT(r.z_err_end, 0.08) << "v_z なしの 0.17 m より小さい（観測ノイズ z の sd 0.1 m で v_z にも誤差が残る）";
}
