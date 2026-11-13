#ifndef EKF_LOCALIZER__EKF_FILTER_HPP_
#define EKF_LOCALIZER__EKF_FILTER_HPP_

#include <Eigen/Geometry>

#include <memory>

#include "ekf_localizer/ekf_params.hpp"
#include "ekf_localizer/lagged_ekf.hpp"
#include "ekf_localizer/ndt_model.hpp"
#include "ekf_localizer/odom_model.hpp"

namespace ekf_localizer
{

/// EKF 一式：履歴つきの EKF 本体と、観測モデル
struct EkfFilter
{
  std::shared_ptr<const NdtModel> ndt;
  /// use_odom: false なら null
  std::shared_ptr<const OdomModel> odom;
  /// オドメトリの観測ノイズの計算（直前の速度を覚える）。use_odom: false なら null
  std::shared_ptr<OdomNoise> odom_noise;
  /// 履歴つきの EKF 本体
  std::unique_ptr<LaggedEkf> ekf;
};

/// EKF 一式を作る（use_odom: true ならオドメトリの観測モデルも）。mount は base → LiDAR の取付変換
EkfFilter createFilter(const EkfParams & params, const Eigen::Affine3d & mount);

}  // namespace ekf_localizer

#endif  // EKF_LOCALIZER__EKF_FILTER_HPP_
