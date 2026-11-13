#include "ekf_localizer/ekf_params.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <utility>
#include <vector>

namespace ekf_localizer
{

namespace
{

/// Eigen のベクトル → ROS のパラメータの double の配列
template<class Derived>
std::vector<double> toStdVector(const Eigen::MatrixBase<Derived> & v)
{
  return std::vector<double>(v.derived().data(), v.derived().data() + v.size());
}

/// 名前を変えたパラメータ（古い名前, 新しい名前）。2026-11-13
const std::vector<std::pair<std::string, std::string>> kRenamedParameters = {
  {"predict_rate", "publish_rate"},
  {"accel_noise_tau", "odom_noise_tau"},
  {"gate_ndt", "ndt_gate"},
  {"gate_odom", "odom_gate"},
};

}  // namespace

void declareEkfParameters(rclcpp::Node & node)
{
  const EkfParams d;
  node.declare_parameter("use_odom", d.use_odom);
  node.declare_parameter("publish_rate", d.publish_rate);
  node.declare_parameter("tf_future_steps", d.tf_future_steps);
  node.declare_parameter("tf_past_steps", d.tf_past_steps);
  node.declare_parameter("history_length", d.history_length);
  node.declare_parameter("map_frame_id", d.map_frame_id);
  node.declare_parameter("base_frame_id", d.base_frame_id);
  node.declare_parameter("lidar_frame_id", d.lidar_frame_id);
  node.declare_parameter("Q", toStdVector(d.core.q));
  node.declare_parameter("P_init", toStdVector(d.core.p_init));
  node.declare_parameter("max_predict_dt", d.core.max_predict_dt);

  // NDT の観測
  node.declare_parameter("R_ndt", toStdVector(d.ndt.r));
  node.declare_parameter("ndt_gate", d.ndt.gate_threshold);
  node.declare_parameter("ndt_delay", d.ndt_delay);

  // オドメトリの観測（use_odom: true のみ）
  node.declare_parameter("R_odom", toStdVector(d.odom.r));
  node.declare_parameter("odom_noise_alpha", toStdVector(d.odom.alpha));
  node.declare_parameter("odom_noise_tau", toStdVector(d.odom.tau));
  node.declare_parameter("odom_gate", d.odom.gate_threshold);
  node.declare_parameter("odom_timeout_init", d.odom.timeout_init);
  node.declare_parameter("odom_delay", d.odom_delay);
  node.declare_parameter("odom_scale", toStdVector(d.odom_scale));
  node.declare_parameter("odom_stamp_mode", d.odom_stamp_mode);
  node.declare_parameter("odom_stamp_auto_threshold", d.odom_stamp_auto_threshold);
}

EkfParams loadEkfParams(rclcpp::Node & node)
{
  const rclcpp::Logger logger = node.get_logger();
  const EkfParams defaults;
  EkfParams params;

  // 古い名前は宣言していないので、渡されても読まれない（既定値になる）。気づけるように WARN を出す
  const auto & overrides = node.get_node_parameters_interface()->get_parameter_overrides();
  for (const auto & [old_name, new_name] : kRenamedParameters) {
    if (overrides.count(old_name) > 0) {
      RCLCPP_WARN(
        logger, "Parameter \"%s\" was renamed to \"%s\". The old name is ignored.",
        old_name.c_str(), new_name.c_str());
    }
  }

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

  node.get_parameter("use_odom", params.use_odom);
  node.get_parameter("publish_rate", params.publish_rate);
  if (!(params.publish_rate > 0.0)) {
    RCLCPP_WARN(
      logger, "publish_rate must be positive (got %lf). Using %lf.",
      params.publish_rate, defaults.publish_rate);
    params.publish_rate = defaults.publish_rate;
  }
  node.get_parameter("history_length", params.history_length);
  if (!(params.history_length > 0.0)) {
    RCLCPP_WARN(
      logger, "history_length must be positive (got %lf). Using %lf.",
      params.history_length, defaults.history_length);
    params.history_length = defaults.history_length;
  }
  node.get_parameter("tf_future_steps", params.tf_future_steps);
  node.get_parameter("tf_past_steps", params.tf_past_steps);
  if (params.tf_future_steps < 0) {
    RCLCPP_WARN(logger, "tf_future_steps must be >= 0 (got %d). Using 0.", params.tf_future_steps);
    params.tf_future_steps = 0;
  }
  if (params.tf_past_steps < 0) {
    RCLCPP_WARN(logger, "tf_past_steps must be >= 0 (got %d). Using 0.", params.tf_past_steps);
    params.tf_past_steps = 0;
  }
  if (params.tf_future_steps == 0 && params.tf_past_steps > 0) {
    RCLCPP_WARN(logger, "tf_past_steps is used only when tf_future_steps >= 1. Ignoring it.");
    params.tf_past_steps = 0;
  }
  // 送り直す刻みは履歴の中に収める（起点より古い刻みの状態は求められない）
  const int max_past =
    std::max(0, static_cast<int>(std::ceil(params.history_length * params.publish_rate)) - 1);
  if (params.tf_past_steps > max_past) {
    RCLCPP_WARN(
      logger, "tf_past_steps must be shorter than history_length (got %d). Using %d.",
      params.tf_past_steps, max_past);
    params.tf_past_steps = max_past;
  }
  // 1 つの /tf のメッセージを LAN の 1 フレーム（1500 B、TF 1 個は約 100 B）に収める目安
  if (params.tf_future_steps + params.tf_past_steps > 10) {
    RCLCPP_WARN(
      logger, "tf_future_steps + tf_past_steps = %d transforms per message. "
      "More than 10 may not fit in one Ethernet frame.",
      params.tf_future_steps + params.tf_past_steps);
  }
  node.get_parameter("map_frame_id", params.map_frame_id);
  node.get_parameter("base_frame_id", params.base_frame_id);
  node.get_parameter("lidar_frame_id", params.lidar_frame_id);

  std::vector<double> q, p_init, r_ndt, r_odom, odom_alpha, odom_tau, odom_scale;
  // Q と P_init は状態と同じ
  // [x, y, z, roll, pitch, yaw, v_xy, v_z, omega_roll, omega_pitch, omega_yaw] の 11 要素
  EkfCore::Params & core = params.core;
  if (load("Q", EkfCore::kStateSize, q)) {
    core.q = Eigen::Map<const EkfCore::StateVector>(q.data());
  }
  if (load("P_init", EkfCore::kStateSize, p_init)) {
    core.p_init = Eigen::Map<const EkfCore::StateVector>(p_init.data());
  }
  node.get_parameter("max_predict_dt", core.max_predict_dt);

  NdtModel::Params & ndt = params.ndt;
  if (load("R_ndt", 6, r_ndt)) {
    ndt.r = Eigen::Map<const Vector6d>(r_ndt.data());
  }
  node.get_parameter("ndt_gate", ndt.gate_threshold);
  node.get_parameter("ndt_delay", params.ndt_delay);
  if (!(params.ndt_delay >= 0.0) || params.ndt_delay >= params.history_length) {
    RCLCPP_WARN(
      logger, "ndt_delay must be in [0, history_length) (got %lf). Using %lf.",
      params.ndt_delay, defaults.ndt_delay);
    params.ndt_delay = defaults.ndt_delay;
  }

  OdomModel::Params & odom = params.odom;
  if (load("R_odom", 2, r_odom)) {
    const Eigen::Map<const Eigen::Vector2d> r(r_odom.data());
    if ((r.array() > 0.0).all()) {
      odom.r = r;
    } else {
      RCLCPP_WARN(logger, "R_odom must be positive (got [%g, %g]). Using default.", r(0), r(1));
    }
  }
  if (load("odom_noise_alpha", 4, odom_alpha)) {
    const Eigen::Map<const Eigen::Vector4d> a(odom_alpha.data());
    if ((a.array() >= 0.0).all()) {
      odom.alpha = a;
    } else {
      RCLCPP_WARN(logger, "odom_noise_alpha must not be negative. Using default.");
    }
  }
  if (load("odom_noise_tau", 2, odom_tau)) {
    odom.tau = Eigen::Map<const Eigen::Vector2d>(odom_tau.data());
  }
  node.get_parameter("odom_gate", odom.gate_threshold);
  node.get_parameter("odom_timeout_init", odom.timeout_init);
  node.get_parameter("odom_delay", params.odom_delay);
  if (!(params.odom_delay >= 0.0) || params.odom_delay >= params.history_length) {
    RCLCPP_WARN(
      logger, "odom_delay must be in [0, history_length) (got %lf). Using %lf.",
      params.odom_delay, defaults.odom_delay);
    params.odom_delay = defaults.odom_delay;
  }
  if (load("odom_scale", 2, odom_scale)) {
    params.odom_scale = Eigen::Map<const Eigen::Vector2d>(odom_scale.data());
  }
  node.get_parameter("odom_stamp_mode", params.odom_stamp_mode);
  if (params.odom_stamp_mode != "stamp" && params.odom_stamp_mode != "receipt" &&
    params.odom_stamp_mode != "auto")
  {
    RCLCPP_WARN(
      logger, "odom_stamp_mode must be stamp, receipt or auto (got \"%s\"). Using %s.",
      params.odom_stamp_mode.c_str(), defaults.odom_stamp_mode.c_str());
    params.odom_stamp_mode = defaults.odom_stamp_mode;
  }
  node.get_parameter("odom_stamp_auto_threshold", params.odom_stamp_auto_threshold);

  return params;
}

void logEkfParams(const rclcpp::Logger & logger, const EkfParams & params)
{
  const EkfCore::Params & core = params.core;
  const NdtModel::Params & ndt = params.ndt;
  RCLCPP_INFO(
    logger, "ekf model: %s 11-state EKF "
    "[x,y,z,roll,pitch,yaw,v_xy,v_z,omega_roll,omega_pitch,omega_yaw]%s",
    params.base_frame_id.c_str(),
    params.use_odom ? " + odom observation [v_xy,omega_yaw]" : " (odom unused)");
  RCLCPP_INFO(
    logger, "ekf frames: %s -> %s (lidar: %s)",
    params.map_frame_id.c_str(), params.base_frame_id.c_str(), params.lidar_frame_id.c_str());
  RCLCPP_INFO(
    logger, "ekf publish_rate: %lf Hz (predicted to the current time), history_length: %lf s",
    params.publish_rate, params.history_length);
  if (params.tf_future_steps > 0) {
    RCLCPP_INFO(
      logger, "ekf TF on the %lf s grid: %d future step(s), %d past step(s) resent",
      1.0 / params.publish_rate, params.tf_future_steps, params.tf_past_steps);
  }
  RCLCPP_INFO(
    logger, "ekf ndt_gate: %lf (NDT observed as the base pose, 6 components at once)",
    ndt.gate_threshold);
  RCLCPP_INFO(logger, "ekf max_predict_dt: %lf", core.max_predict_dt);
  RCLCPP_INFO(logger, "ekf ndt_delay: %lf s", params.ndt_delay);
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
    join(core.q).c_str());
  RCLCPP_INFO(
    logger, "ekf R_ndt: [%g, %g, %g, %g, %g, %g]",
    ndt.r(0), ndt.r(1), ndt.r(2), ndt.r(3), ndt.r(4), ndt.r(5));
  RCLCPP_INFO(
    logger, "ekf P_init(x,y,z,roll,pitch,yaw,v_xy,v_z,omega_roll,omega_pitch,omega_yaw): [%s]",
    join(core.p_init).c_str());
  if (params.use_odom) {
    const OdomModel::Params & odom = params.odom;
    RCLCPP_INFO(
      logger, "ekf R_odom: [%g, %g], odom_noise_alpha: [%g, %g, %g, %g], odom_noise_tau: [%g, %g], "
      "odom_gate: %lf, odom_timeout_init: %lf",
      odom.r(0), odom.r(1), odom.alpha(0), odom.alpha(1), odom.alpha(2), odom.alpha(3),
      odom.tau(0), odom.tau(1), odom.gate_threshold, odom.timeout_init);
    RCLCPP_INFO(
      logger, "ekf odom_delay: %lf s, odom_scale: [%g, %g], odom_stamp_mode: %s "
      "(odom_stamp_auto_threshold %g s)",
      params.odom_delay, params.odom_scale(0), params.odom_scale(1),
      params.odom_stamp_mode.c_str(), params.odom_stamp_auto_threshold);
  }
}

}  // namespace ekf_localizer
