#include "ekf_localizer/ekf_filter.hpp"

namespace ekf_localizer
{

EkfFilter createFilter(const EkfParams & params, const Eigen::Affine3d & mount)
{
  // 観測モデルは作るときに core にゲートを確保するので、LaggedEkf より先に作る
  EkfCore core(params.core);
  EkfFilter f;
  f.ndt = std::make_shared<NdtModel>(params.ndt, mount, core);
  if (params.use_odom) {
    f.odom = std::make_shared<OdomModel>(params.odom, core);
    f.odom_noise = std::make_shared<OdomNoise>(params.odom);
  }
  f.ekf = std::make_unique<LaggedEkf>(core, params.history_length);
  return f;
}

}  // namespace ekf_localizer
