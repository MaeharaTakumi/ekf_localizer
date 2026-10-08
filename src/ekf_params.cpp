#include "ekf_localizer/ekf_params.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace ekf_localizer
{

void declareEkfParameters(rclcpp::Node & node)
{
  node.declare_parameter("use_odom", false);
  node.declare_parameter("ndt_pose_topic", "ndt_pose");
  node.declare_parameter("odom_topic", "odom");
  node.declare_parameter("initialpose_topic", "initialpose");
  node.declare_parameter("predict_rate", 50.0);
  node.declare_parameter("history_length", 1.0);
  node.declare_parameter("publish_tf", true);
  node.declare_parameter("map_frame_id", "map");
  node.declare_parameter("base_frame_id", "base_link");
  node.declare_parameter("lidar_frame_id", "velodyne");
  node.declare_parameter(
    "Q", std::vector<double>{1.0e-3, 1.0e-3, 2.0e-3, 5.0e-3, 5.0e-3, 1.0e-3, 0.05, 0.2});
  node.declare_parameter(
    "R_ndt", std::vector<double>{1.0e-4, 1.0e-4, 1.0e-4, 1.0e-6, 1.0e-6, 1.0e-6});
  node.declare_parameter("max_predict_dt", 0.2);
  node.declare_parameter(
    "P_init", std::vector<double>{2.0e-4, 2.0e-4, 2.0e-4, 2.0e-6, 2.0e-6, 2.0e-6, 1.0, 1.0});
  node.declare_parameter("gate_horizontal", 11.34);
  node.declare_parameter("gate_1d", 6.63);
  node.declare_parameter("lockout_count", 10);
  node.declare_parameter("ndt_delay", 0.0);

  // オドメトリ観測（use_odom: true のみ）
  node.declare_parameter("odom_covariance_source", "param");
  node.declare_parameter("R_odom", std::vector<double>{2.5e-3, 5.0e-3});
  node.declare_parameter("gate_odom", 9.21);
  node.declare_parameter("odom_timeout_init", 0.2);
  node.declare_parameter("odom_delay", 0.0);
}

EkfConfig loadEkfConfig(rclcpp::Node & node)
{
  const rclcpp::Logger logger = node.get_logger();
  EkfConfig cfg;

  // ベクトルパラメータ。要素数が合わなければ既定値のまま WARN を出す。
  const auto load = [&](const std::string & name, std::size_t n, std::vector<double> & out) {
      node.get_parameter(name, out);
      if (out.size() != n) {
        RCLCPP_WARN(
          logger, "Parameter \"%s\" must have %zu elements (got %zu). Using default.",
          name.c_str(), n, out.size());
        return false;
      }
      return true;
    };
  // 文字列パラメータ。候補外なら先頭の候補に戻す
  const auto choose = [&](const std::string & name, std::string & out,
      std::initializer_list<const char *> candidates) {
      node.get_parameter(name, out);
      for (const char * c : candidates) {
        if (out == c) {return;}
      }
      const char * fallback = *candidates.begin();
      RCLCPP_WARN(
        logger, "Unknown %s \"%s\". Falling back to \"%s\".", name.c_str(), out.c_str(), fallback);
      out = fallback;
    };

  node.get_parameter("use_odom", cfg.use_odom);
  node.get_parameter("ndt_pose_topic", cfg.ndt_pose_topic);
  node.get_parameter("odom_topic", cfg.odom_topic);
  node.get_parameter("initialpose_topic", cfg.initialpose_topic);
  node.get_parameter("predict_rate", cfg.predict_rate);
  if (!(cfg.predict_rate > 0.0)) {
    RCLCPP_WARN(logger, "predict_rate must be positive (got %lf). Using 50.0.", cfg.predict_rate);
    cfg.predict_rate = 50.0;
  }
  node.get_parameter("history_length", cfg.history_length);
  if (!(cfg.history_length > 0.0)) {
    RCLCPP_WARN(logger, "history_length must be positive (got %lf). Using 1.0.", cfg.history_length);
    cfg.history_length = 1.0;
  }
  node.get_parameter("publish_tf", cfg.publish_tf);
  node.get_parameter("map_frame_id", cfg.map_frame_id);
  node.get_parameter("base_frame_id", cfg.base_frame_id);
  node.get_parameter("lidar_frame_id", cfg.lidar_frame_id);
  choose("odom_covariance_source", cfg.odom_covariance_source, {"param", "message"});

  std::vector<double> q, p_init, r_ndt, r_odom;
  // Q と P_init は状態と同じ [x, y, z, roll, pitch, yaw, v, omega] の 8 要素
  if (load("Q", EkfCore::kN, q)) {
    cfg.core.q = Eigen::Map<const EkfCore::StateVector>(q.data());
  }
  if (load("P_init", EkfCore::kN, p_init)) {
    cfg.core.p_init = Eigen::Map<const EkfCore::StateVector>(p_init.data());
  }
  node.get_parameter("max_predict_dt", cfg.core.max_predict_dt);

  NdtModel::Params & nprm = cfg.ndt;
  if (load("R_ndt", 6, r_ndt)) {
    nprm.r = Eigen::Map<const NdtModel::Vector6d>(r_ndt.data());
  }
  node.get_parameter("gate_horizontal", nprm.gate_horizontal);
  node.get_parameter("gate_1d", nprm.gate_1d);
  node.get_parameter("lockout_count", nprm.lockout_count);
  node.get_parameter("ndt_delay", cfg.ndt_delay);
  if (!(cfg.ndt_delay >= 0.0) || cfg.ndt_delay >= cfg.history_length) {
    RCLCPP_WARN(
      logger, "ndt_delay must be in [0, history_length) (got %lf). Using 0.0.", cfg.ndt_delay);
    cfg.ndt_delay = 0.0;
  }

  OdomModel::Params & oprm = cfg.odom;
  if (load("R_odom", 2, r_odom)) {
    oprm.r = Eigen::Map<const Eigen::Vector2d>(r_odom.data());
  }
  node.get_parameter("gate_odom", oprm.gate);
  node.get_parameter("odom_timeout_init", oprm.timeout_init);
  node.get_parameter("odom_delay", cfg.odom_delay);
  if (!(cfg.odom_delay >= 0.0) || cfg.odom_delay >= cfg.history_length) {
    RCLCPP_WARN(
      logger, "odom_delay must be in [0, history_length) (got %lf). Using 0.0.", cfg.odom_delay);
    cfg.odom_delay = 0.0;
  }

  return cfg;
}

void logEkfConfig(const rclcpp::Logger & logger, const EkfConfig & cfg)
{
  const EkfCore::Params & cprm = cfg.core;
  const NdtModel::Params & nprm = cfg.ndt;
  RCLCPP_INFO(
    logger, "ekf model: %s 8-state EKF [x,y,z,roll,pitch,yaw,v,omega]%s",
    cfg.base_frame_id.c_str(), cfg.use_odom ? " + odom observation [v,omega]" : " (odom unused)");
  RCLCPP_INFO(
    logger, "ekf topics: ndt_pose: %s, odom: %s, initialpose: %s",
    cfg.ndt_pose_topic.c_str(), cfg.odom_topic.c_str(), cfg.initialpose_topic.c_str());
  RCLCPP_INFO(
    logger, "ekf frames: %s -> %s (lidar: %s), publish_tf: %d",
    cfg.map_frame_id.c_str(), cfg.base_frame_id.c_str(), cfg.lidar_frame_id.c_str(),
    cfg.publish_tf);
  RCLCPP_INFO(
    logger, "ekf predict_rate: %lf Hz, history_length: %lf s",
    cfg.predict_rate, cfg.history_length);
  RCLCPP_INFO(
    logger, "ekf gate_horizontal: %lf, gate_1d: %lf, lockout_count: %d",
    nprm.gate_horizontal, nprm.gate_1d, nprm.lockout_count);
  RCLCPP_INFO(logger, "ekf max_predict_dt: %lf", cprm.max_predict_dt);
  RCLCPP_INFO(logger, "ekf ndt_delay: %lf s", cfg.ndt_delay);
  RCLCPP_INFO(
    logger, "ekf Q(x,y,z,roll,pitch,yaw,v,omega): [%g, %g, %g, %g, %g, %g, %g, %g]",
    cprm.q(0), cprm.q(1), cprm.q(2), cprm.q(3), cprm.q(4), cprm.q(5), cprm.q(6), cprm.q(7));
  RCLCPP_INFO(
    logger, "ekf R_ndt: [%g, %g, %g, %g, %g, %g]",
    nprm.r(0), nprm.r(1), nprm.r(2), nprm.r(3), nprm.r(4), nprm.r(5));
  RCLCPP_INFO(
    logger, "ekf P_init(x,y,z,roll,pitch,yaw,v,omega): [%g, %g, %g, %g, %g, %g, %g, %g]",
    cprm.p_init(0), cprm.p_init(1), cprm.p_init(2), cprm.p_init(3), cprm.p_init(4),
    cprm.p_init(5), cprm.p_init(6), cprm.p_init(7));
  if (cfg.use_odom) {
    const OdomModel::Params & oprm = cfg.odom;
    RCLCPP_INFO(
      logger, "ekf R_odom: [%g, %g] (source: %s), gate_odom: %lf, odom_timeout_init: %lf",
      oprm.r(0), oprm.r(1), cfg.odom_covariance_source.c_str(), oprm.gate, oprm.timeout_init);
    RCLCPP_INFO(logger, "ekf odom_delay: %lf s", cfg.odom_delay);
  }
}

EkfFilter createFilter(const EkfConfig & cfg, const Eigen::Affine3d & mount)
{
  // 観測モデルは作るときに core にゲートを確保するので、LaggedEkf より先に作る
  EkfCore core(cfg.core);
  EkfFilter f;
  f.ndt = std::make_shared<NdtModel>(cfg.ndt, mount, core);
  if (cfg.use_odom) {
    f.odom = std::make_shared<OdomModel>(cfg.odom, core);
  }
  f.lagged = std::make_unique<LaggedEkf>(core, cfg.history_length);
  return f;
}

}  // namespace ekf_localizer
