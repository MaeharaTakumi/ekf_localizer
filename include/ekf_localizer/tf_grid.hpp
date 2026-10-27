#ifndef EKF_LOCALIZER__TF_GRID_HPP_
#define EKF_LOCALIZER__TF_GRID_HPP_

#include <cstdint>
#include <vector>

namespace ekf_localizer
{

/// TF を送る刻みの stamp [ns]
///
///   刻みは period_ns の整数倍（k·period_ns）。t_ns 以前の最後の刻みを k0 として、
///   k0 − past_steps + 1 〜 k0 + future_steps の刻みを古い順に返す。
///   stamp は整数で作るので、周期が揺れても同じ刻みは必ず同じ stamp になる（購読側の tf2 で置き換わる）。
///   ROS に依存しない。
std::vector<int64_t> tfGridStamps(
  int64_t t_ns, int64_t period_ns, int future_steps, int past_steps);

}  // namespace ekf_localizer

#endif  // EKF_LOCALIZER__TF_GRID_HPP_
