// EkfCore（運動モデル）の単体テスト（ROS 非依存）
//   colcon build --packages-select ekf_localizer
//   colcon test --packages-select ekf_localizer --event-handlers console_direct+
#include <gtest/gtest.h>

#include <cmath>

#include "ekf_localizer/ekf_core.hpp"

using ekf_localizer::EkfCore;
using ekf_localizer::normalizeAngle;
using VS = EkfCore::StateVector;
using MS = EkfCore::StateMatrix;

// ===========================================================================
// EkfCore（運動モデル）
// ===========================================================================

TEST(EkfCore, MidpointIntegrationOnHorizontalPlane)
{
  using E = EkfCore;
  VS x = VS::Zero();
  x(E::kX) = 1.0; x(E::kY) = 2.0; x(E::kZ) = 0.5; x(E::kRoll) = 0.05; x(E::kPitch) = -0.08;
  x(E::kYaw) = 0.3; x(E::kVxy) = 1.2; x(E::kOmegaYaw) = 0.8;
  x(E::kVz) = -0.07; x(E::kOmegaRoll) = 0.02; x(E::kOmegaPitch) = -0.03;
  for (double dt : {0.05, 0.2, 1.0}) {
    const VS xn = EkfCore::propagate(x, dt);
    const double yaw_m = x(E::kYaw) + 0.5 * x(E::kOmegaYaw) * dt;   // 区間の中点の向き
    EXPECT_NEAR(xn(E::kX), x(E::kX) + x(E::kVxy) * dt * std::cos(yaw_m), 1e-12) << "dt=" << dt;
    EXPECT_NEAR(xn(E::kY), x(E::kY) + x(E::kVxy) * dt * std::sin(yaw_m), 1e-12) << "dt=" << dt;
    EXPECT_NEAR(normalizeAngle(xn(E::kYaw) - x(E::kYaw) - x(E::kOmegaYaw) * dt), 0.0, 1e-12);
    // 各軸は独立：z, roll, pitch はそれぞれの速度だけで動き、roll, pitch があっても v_xy を分解しない
    EXPECT_NEAR(xn(E::kZ), x(E::kZ) + x(E::kVz) * dt, 1e-12) << "dt=" << dt;
    EXPECT_NEAR(xn(E::kRoll), x(E::kRoll) + x(E::kOmegaRoll) * dt, 1e-12) << "dt=" << dt;
    EXPECT_NEAR(xn(E::kPitch), x(E::kPitch) + x(E::kOmegaPitch) * dt, 1e-12) << "dt=" << dt;
    for (int i = E::kVxy; i < E::kStateSize; ++i) {
      EXPECT_DOUBLE_EQ(xn(i), x(i)) << "速度はランダムウォーク i=" << i;
    }
  }
  // 速度 v_z, omega_roll, omega_pitch が 0 なら z, roll, pitch は予測で変わらない（足す前の 8 状態と同じ）
  VS x0 = x;
  x0(E::kVz) = x0(E::kOmegaRoll) = x0(E::kOmegaPitch) = 0.0;
  const VS x0n = EkfCore::propagate(x0, 0.2);
  EXPECT_EQ(x0n(E::kZ), x0(E::kZ));
  EXPECT_EQ(x0n(E::kRoll), x0(E::kRoll));
  EXPECT_EQ(x0n(E::kPitch), x0(E::kPitch));
  // 円弧の厳密解とは向きが同じ（中点の向き）で、長さが v dt (1 - sinc(omega dt / 2)) だけ長い（1 ステップで O(dt^3)）
  VS f = x;
  f(E::kRoll) = f(E::kPitch) = 0.0;
  for (double dt : {0.05, 0.2}) {
    const VS xn = EkfCore::propagate(f, dt);
    const double R = f(E::kVxy) / f(E::kOmegaYaw), u = 0.5 * f(E::kOmegaYaw) * dt;
    const double ex =
      f(E::kX) + R * (std::sin(f(E::kYaw) + f(E::kOmegaYaw) * dt) - std::sin(f(E::kYaw)));
    const double ey =
      f(E::kY) - R * (std::cos(f(E::kYaw) + f(E::kOmegaYaw) * dt) - std::cos(f(E::kYaw)));
    EXPECT_NEAR(
      std::hypot(xn(E::kX) - ex, xn(E::kY) - ey), f(E::kVxy) * dt * (1.0 - std::sin(u) / u), 1e-12)
      << "dt=" << dt;
  }
  VS a = VS::Zero();
  a(E::kYaw) = 0.3; a(E::kVxy) = 1.0; a(E::kOmegaYaw) = 1e-9;
  VS b = a;
  b(E::kOmegaYaw) = 0.0;
  EXPECT_LT((EkfCore::propagate(a, 0.1) - EkfCore::propagate(b, 0.1)).head<2>().norm(), 1e-9)
    << "omega → 0 で直線に連続（分岐なし）";
}

TEST(EkfCore, TransitionJacobianMatchesNumerical)
{
  for (double w : {0.8, 1e-5, 0.0, -1.3}) {
    for (double th : {0.0, 0.2, -0.08}) {
      VS x;
      x << 1.0, -2.0, 0.5, 0.05, th, 2.9, 0.9, -0.06, 0.03, -0.04, w;
      const double dt = 0.15, h = 1e-6;
      MS Fn;
      for (int j = 0; j < EkfCore::kStateSize; ++j) {
        VS xp = x, xm = x;
        xp(j) += h; xm(j) -= h;
        VS d = EkfCore::propagate(xp, dt) - EkfCore::propagate(xm, dt);
        d(EkfCore::kYaw) = normalizeAngle(d(EkfCore::kYaw));
        Fn.col(j) = d / (2 * h);
      }
      const MS Fa = EkfCore::transitionJacobian(x, dt);
      EXPECT_LT((Fa - Fn).cwiseAbs().maxCoeff(), 1e-6) << "omega=" << w << " pitch=" << th;
    }
  }
}
