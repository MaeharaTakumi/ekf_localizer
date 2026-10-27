// TF を送る刻みの stamp（tfGridStamps）の単体テスト（ROS 非依存）
//   colcon test --packages-select ekf_localizer --ctest-args -R test_tf_grid
#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "ekf_localizer/tf_grid.hpp"

using ekf_localizer::tfGridStamps;

namespace
{

constexpr int64_t kMs = 1000000;
constexpr int64_t kT = 20 * kMs;   // 50 Hz

std::vector<int64_t> ms(std::vector<int64_t> v)
{
  for (int64_t & x : v) {x *= kMs;}
  return v;
}

}  // namespace

TEST(TfGrid, FutureAndPastSteps)
{
  // 今 1013 ms：今以前の最後の刻みは 1000 ms
  EXPECT_EQ(tfGridStamps(1013 * kMs, kT, 2, 1), ms({1000, 1020, 1040}));
  EXPECT_EQ(tfGridStamps(1013 * kMs, kT, 2, 0), ms({1020, 1040}));
  EXPECT_EQ(tfGridStamps(1013 * kMs, kT, 2, 3), ms({960, 980, 1000, 1020, 1040}));
  EXPECT_EQ(tfGridStamps(1013 * kMs, kT, 1, 1), ms({1000, 1020}));
}

TEST(TfGrid, NowOnTheGridIsPast)
{
  // 今がちょうど刻みなら、その刻みは今以前に数える
  EXPECT_EQ(tfGridStamps(1020 * kMs, kT, 2, 1), ms({1020, 1040, 1060}));
  EXPECT_EQ(tfGridStamps(1020 * kMs, kT, 2, 0), ms({1040, 1060}));
}

TEST(TfGrid, SameStampsWhenThePeriodJitters)
{
  // タイマーが同じ刻みの間のどこで動いても、同じ stamp になる
  const std::vector<int64_t> ref = tfGridStamps(1000 * kMs, kT, 2, 1);
  for (int64_t t = 1000 * kMs; t < 1020 * kMs; t += 137 * 1000) {
    EXPECT_EQ(tfGridStamps(t, kT, 2, 1), ref) << "t = " << t;
  }
  EXPECT_NE(tfGridStamps(1020 * kMs, kT, 2, 1), ref);
}

TEST(TfGrid, ExactMultiplesAtRosTime)
{
  // 壁時計の大きな時刻でも、stamp は周期のちょうど整数倍（1 ns もずれない）
  const int64_t t = 1793071565852931004;
  const std::vector<int64_t> s = tfGridStamps(t, kT, 2, 1);
  ASSERT_EQ(s.size(), 3u);
  for (const int64_t x : s) {EXPECT_EQ(x % kT, 0);}
  EXPECT_LE(s[0], t);
  EXPECT_GT(s[1], t);
  EXPECT_EQ(s[1] - s[0], kT);
}

TEST(TfGrid, NegativeTimeRoundsDown)
{
  EXPECT_EQ(tfGridStamps(-5 * kMs, kT, 1, 1), ms({-20, 0}));
  EXPECT_EQ(tfGridStamps(0, kT, 1, 1), ms({0, 20}));
}
