#include "ekf_localizer/ekf_params.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
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
  node.declare_parameter("tf_future_steps", 0);
  node.declare_parameter("tf_past_steps", 0);
  node.declare_parameter("history_length", 1.0);
  node.declare_parameter("map_frame_id", "map");
  node.declare_parameter("base_frame_id", "base_link");
  node.declare_parameter("lidar_frame_id", "velodyne");
  node.declare_parameter(
    "Q", std::vector<double>{
      1.0e-3, 1.0e-3, 2.0e-3, 5.0e-3, 5.0e-3, 1.0e-3, 0.05, 0.0, 0.0, 0.0, 0.2});
  node.declare_parameter(
    "R_ndt", std::vector<double>{1.0e-4, 1.0e-4, 1.0e-4, 1.0e-6, 1.0e-6, 1.0e-6});
  node.declare_parameter("max_predict_dt", 0.2);
  node.declare_parameter(
    "P_init", std::vector<double>{
      2.0e-4, 2.0e-4, 2.0e-4, 2.0e-6, 2.0e-6, 2.0e-6, 1.0, 0.0, 0.0, 0.0, 1.0});
  node.declare_parameter("gate_ndt", 16.81);
  node.declare_parameter("ndt_delay", 0.0);

  // オドメトリ観測（use_odom: true のみ）
  node.declare_parameter("R_odom", std::vector<double>{6.08e-3, 9.92e-4});
  node.declare_parameter("odom_noise_alpha", std::vector<double>{0.0, 0.0, 0.0, 0.0});
  node.declare_parameter("accel_noise_tau", std::vector<double>{0.0, 0.0});
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
  node.get_parameter("tf_future_steps", cfg.tf_future_steps);
  node.get_parameter("tf_past_steps", cfg.tf_past_steps);
  if (cfg.tf_future_steps < 0) {
    RCLCPP_WARN(logger, "tf_future_steps must be >= 0 (got %d). Using 0.", cfg.tf_future_steps);
    cfg.tf_future_steps = 0;
  }
  if (cfg.tf_past_steps < 0) {
    RCLCPP_WARN(logger, "tf_past_steps must be >= 0 (got %d). Using 0.", cfg.tf_past_steps);
    cfg.tf_past_steps = 0;
  }
  if (cfg.tf_future_steps == 0 && cfg.tf_past_steps > 0) {
    RCLCPP_WARN(logger, "tf_past_steps is used only when tf_future_steps >= 1. Ignoring it.");
    cfg.tf_past_steps = 0;
  }
  // 送り直す刻みは履歴の中に収める（起点より古い刻みの状態は求められない）
  const int max_past = std::max(0, static_cast<int>(std::ceil(cfg.history_length * cfg.predict_rate)) - 1);
  if (cfg.tf_past_steps > max_past) {
    RCLCPP_WARN(
      logger, "tf_past_steps must be shorter than history_length (got %d). Using %d.",
      cfg.tf_past_steps, max_past);
    cfg.tf_past_steps = max_past;
  }
  // 1 つの /tf のメッセージを LAN の 1 フレーム（1500 B、TF 1 個は約 100 B）に収める目安
  if (cfg.tf_future_steps + cfg.tf_past_steps > 10) {
    RCLCPP_WARN(
      logger, "tf_future_steps + tf_past_steps = %d transforms per message. "
      "More than 10 may not fit in one Ethernet frame.", cfg.tf_future_steps + cfg.tf_past_steps);
  }
  node.get_parameter("map_frame_id", cfg.map_frame_id);
  node.get_parameter("base_frame_id", cfg.base_frame_id);
  node.get_parameter("lidar_frame_id", cfg.lidar_frame_id);

  std::vector<double> q, p_init, r_ndt, r_odom, odom_alpha, odom_tau;
  // Q と P_init は状態と同じ
  // [x, y, z, roll, pitch, yaw, v_xy, v_z, omega_roll, omega_pitch, omega_yaw] の 11 要素
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
  node.get_parameter("gate_ndt", nprm.gate);
  node.get_parameter("ndt_delay", cfg.ndt_delay);
  if (!(cfg.ndt_delay >= 0.0) || cfg.ndt_delay >= cfg.history_length) {
    RCLCPP_WARN(
      logger, "ndt_delay must be in [0, history_length) (got %lf). Using 0.0.", cfg.ndt_delay);
    cfg.ndt_delay = 0.0;
  }

  OdomModel::Params & oprm = cfg.odom;
  if (load("R_odom", 2, r_odom)) {
    const Eigen::Map<const Eigen::Vector2d> r(r_odom.data());
    if ((r.array() > 0.0).all()) {
      oprm.r = r;
    } else {
      RCLCPP_WARN(logger, "R_odom must be positive (got [%g, %g]). Using default.", r(0), r(1));
    }
  }
  if (load("odom_noise_alpha", 4, odom_alpha)) {
    const Eigen::Map<const Eigen::Vector4d> a(odom_alpha.data());
    if ((a.array() >= 0.0).all()) {
      oprm.alpha = a;
    } else {
      RCLCPP_WARN(logger, "odom_noise_alpha must not be negative. Using default.");
    }
  }
  if (load("accel_noise_tau", 2, odom_tau)) {
    oprm.tau = Eigen::Map<const Eigen::Vector2d>(odom_tau.data());
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
    logger, "ekf model: %s 11-state EKF "
    "[x,y,z,roll,pitch,yaw,v_xy,v_z,omega_roll,omega_pitch,omega_yaw]%s",
    cfg.base_frame_id.c_str(),
    cfg.use_odom ? " + odom observation [v_xy,omega_yaw]" : " (odom unused)");
  RCLCPP_INFO(
    logger, "ekf topics: ndt_pose: %s, odom: %s, initialpose: %s",
    cfg.ndt_pose_topic.c_str(), cfg.odom_topic.c_str(), cfg.initialpose_topic.c_str());
  RCLCPP_INFO(
    logger, "ekf frames: %s -> %s (lidar: %s)",
    cfg.map_frame_id.c_str(), cfg.base_frame_id.c_str(), cfg.lidar_frame_id.c_str());
  RCLCPP_INFO(
    logger, "ekf publish: %lf Hz (predicted to the current time), history_length: %lf s",
    cfg.predict_rate, cfg.history_length);
  if (cfg.tf_future_steps > 0) {
    RCLCPP_INFO(
      logger, "ekf TF on the %lf s grid: %d future step(s), %d past step(s) resent",
      1.0 / cfg.predict_rate, cfg.tf_future_steps, cfg.tf_past_steps);
  }
  RCLCPP_INFO(
    logger, "ekf gate_ndt: %lf (NDT observed as the base pose, 6 components at once)", nprm.gate);
  RCLCPP_INFO(logger, "ekf max_predict_dt: %lf", cprm.max_predict_dt);
  RCLCPP_INFO(logger, "ekf ndt_delay: %lf s", cfg.ndt_delay);
  const auto join = [](const EkfCore::StateVector & v) {
      std::string out;
      for (int i = 0; i < v.size(); ++i) {
        char buf[32];
        std::snprintf(buf, sizeof(buf), "%s%g", i == 0 ? "" : ", ", v(i));
        out += buf;
      }
      return out;
    };
  RCLCPP_INFO(
    logger, "ekf Q(x,y,z,roll,pitch,yaw,v_xy,v_z,omega_roll,omega_pitch,omega_yaw): [%s]",
    join(cprm.q).c_str());
  RCLCPP_INFO(
    logger, "ekf R_ndt: [%g, %g, %g, %g, %g, %g]",
    nprm.r(0), nprm.r(1), nprm.r(2), nprm.r(3), nprm.r(4), nprm.r(5));
  RCLCPP_INFO(
    logger, "ekf P_init(x,y,z,roll,pitch,yaw,v_xy,v_z,omega_roll,omega_pitch,omega_yaw): [%s]",
    join(cprm.p_init).c_str());
  if (cfg.use_odom) {
    const OdomModel::Params & oprm = cfg.odom;
    RCLCPP_INFO(
      logger, "ekf R_odom: [%g, %g], odom_noise_alpha: [%g, %g, %g, %g], accel_noise_tau: [%g, %g], "
      "gate_odom: %lf, odom_timeout_init: %lf",
      oprm.r(0), oprm.r(1), oprm.alpha(0), oprm.alpha(1), oprm.alpha(2), oprm.alpha(3),
      oprm.tau(0), oprm.tau(1), oprm.gate, oprm.timeout_init);
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
    f.odom_noise = std::make_shared<OdomNoise>(cfg.odom);
  }
  f.lagged = std::make_unique<LaggedEkf>(core, cfg.history_length);
  return f;
}

}  // namespace ekf_localizer
