#include "ekf_localizer/tf_grid.hpp"

namespace ekf_localizer
{

std::vector<int64_t> tfGridStamps(
  int64_t t_ns, int64_t period_ns, int future_steps, int past_steps)
{
  // 負の時刻でも t_ns 以前の刻みにする（整数の割り算は 0 の向きに丸めるので直す）
  int64_t k0 = t_ns / period_ns;
  if (t_ns % period_ns < 0) {--k0;}

  std::vector<int64_t> stamps;
  for (int64_t k = k0 - past_steps + 1; k <= k0 + future_steps; ++k) {
    stamps.push_back(k * period_ns);
  }
  return stamps;
}

}  // namespace ekf_localizer
