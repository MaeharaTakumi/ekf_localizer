#include "ekf_localizer/ekf_localizer_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <vector>

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "ekf_localizer/tf_grid.hpp"

namespace ekf_localizer
{

namespace
{

rclcpp::Time toTime(double t)
{
  return rclcpp::Time(static_cast<int64_t>(std::llround(t * 1e9)), RCL_ROS_TIME);
}

/// 観測の時刻 [s]（メッセージの stamp − delay）
double measurementTime(const builtin_interfaces::msg::Time & stamp, double delay)
{
  return rclcpp::Time(stamp, RCL_ROS_TIME).seconds() - delay;
}

using Status = LaggedEkf::Status;

}  // namespace

EkfLocalizer::EkfLocalizer(const rclcpp::NodeOptions & options)
: rclcpp::Node("ekf_localizer", options),
  last_publish_stamp_(0, 0, RCL_ROS_TIME)
{
  declareEkfParameters(*this);
  params_ = loadEkfParams(*this);
  logEkfParams(get_logger(), params_);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // NDT の解は 1 つも落としたくないので reliable
  ndt_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "ndt_pose", rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
    std::bind(&EkfLocalizer::ndtPoseReceived, this, std::placeholders::_1));
  // best effort は reliable / best effort どちらの publisher とも接続できる（Real と Gazebo で共通）。
  // 深さは処理が詰まったときに捨てない程度
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "odom", rclcpp::QoS(rclcpp::KeepLast(50)).best_effort(),
    std::bind(&EkfLocalizer::odomReceived, this, std::placeholders::_1));
  initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", rclcpp::SystemDefaultsQoS(),
    std::bind(&EkfLocalizer::initialPoseReceived, this, std::placeholders::_1));

  // /ekf_pose は map → LiDAR。後から購読したノードも最後の値を受け取れるよう transient_local
  ekf_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "ekf_pose", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  ekf_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("ekf_odom", rclcpp::QoS(10));

  // ROS 時刻のタイマー（use_sim_time ならシミュレーション時刻で回る）。周期は TF の刻みと同じ整数のナノ秒
  period_ns_ = std::max<int64_t>(1, std::llround(1e9 / params_.publish_rate));
  timer_ = rclcpp::create_timer(
    this, get_clock(), std::chrono::nanoseconds(period_ns_),
    std::bind(&EkfLocalizer::timerCallback, this));
}

bool EkfLocalizer::tryCreateFilter()
{
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(params_.base_frame_id, params_.lidar_frame_id, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Waiting for TF %s -> %s: %s",
      params_.base_frame_id.c_str(), params_.lidar_frame_id.c_str(), ex.what());
    return false;
  }
  const Eigen::Affine3d mount = tf2::transformToEigen(tf);
  filter_ = createFilter(params_, mount);

  const Eigen::Vector3d t = mount.translation();
  const Eigen::Vector3d rpy = rpyFromRotation(mount.linear());
  RCLCPP_INFO(
    get_logger(), "mount %s -> %s (from TF): t=[%.3f, %.3f, %.3f], rpy=[%.4f, %.4f, %.4f]",
    params_.base_frame_id.c_str(), params_.lidar_frame_id.c_str(),
    t.x(), t.y(), t.z(), rpy(0), rpy(1), rpy(2));
  return true;
}

builtin_interfaces::msg::Time EkfLocalizer::odomStamp(const builtin_interfaces::msg::Time & stamp)
{
  if (params_.odom_stamp_mode == "stamp") {return stamp;}
  const rclcpp::Time t_receipt = now();
  if (t_receipt.nanoseconds() == 0) {return stamp;}   // sim time で /clock をまだ受け取っていない
  const double offset = t_receipt.seconds() - rclcpp::Time(stamp, RCL_ROS_TIME).seconds();
  const bool use_receipt = params_.odom_stamp_mode == "receipt" ||
    std::abs(offset) >= params_.odom_stamp_auto_threshold;
  if (params_.odom_stamp_mode == "auto") {
    ++odom_count_;
    odom_receipt_count_ += use_receipt ? 1 : 0;
    odom_max_abs_offset_ = std::max(odom_max_abs_offset_, std::abs(offset));
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 10000,
      "auto: receipt time used for %d of %d odometry messages (|receipt - stamp| latest %.1f ms, max %.1f ms)",
      odom_receipt_count_, odom_count_, offset * 1e3, odom_max_abs_offset_ * 1e3);
  }
  return use_receipt ? static_cast<builtin_interfaces::msg::Time>(t_receipt) : stamp;
}

void EkfLocalizer::addMeasurement(const MeasurementPtr & m)
{
  const ObservationModel & model = *m->model;
  const LaggedEkf::AddResult r = filter_.ekf->add(m);

  if (r.status == Status::kTooOld) {
    const rclcpp::Time t_now = now();
    const auto it = too_old_warned_.find(&model);
    if (it == too_old_warned_.end() || (t_now - it->second).seconds() >= 5.0) {
      RCLCPP_WARN(
        get_logger(), "%s is older than the history (%.3f s behind the latest). "
        "Increase history_length.", model.name(), filter_.ekf->latestTime() - m->time);
      too_old_warned_.insert_or_assign(&model, t_now);
    }
    return;
  }

  const ApplyResult & o = r.apply_result;
  if (o.initialized_now) {
    const EkfCore & ekf = filter_.ekf->current();
    const MeasurementPtr & aid = ekf.initAid();
    RCLCPP_INFO(
      get_logger(), "EKF initialized by %s (v_xy, omega_yaw = %.3f, %.3f from %s).", model.name(),
      ekf.vXy(), ekf.omegaYaw(), o.aided ? aid->model->name() : "zero");
    return;
  }

  using S = StageResult::Status;
  for (const StageResult & st : o.stages) {
    if (st.status == S::kRejected) {
      RCLCPP_WARN(
        get_logger(), "%s %s rejected by gate (d2 = %lf, %d times in a row).",
        model.name(), st.name, st.d2, st.reject_count);
    }
  }
}

void EkfLocalizer::ndtPoseReceived(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
{
  if (!filter_.ekf) {return;}   // EKF はタイマーで作る（取付が TF から読めてから）

  // NDT の解は map → LiDAR の姿勢。共分散は使わない（観測ノイズは R_ndt）
  const auto & p = msg->pose.pose;
  double roll, pitch, yaw;
  tf2::Matrix3x3(
    tf2::Quaternion(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w))
  .getRPY(roll, pitch, yaw);
  NdtModel::Vector6d z;
  z << p.position.x, p.position.y, p.position.z, roll, pitch, yaw;

  // 点群の取得時刻が stamp より ndt_delay だけ前のときに補う
  const double t = measurementTime(msg->header.stamp, params_.ndt_delay);
  addMeasurement(filter_.ndt->measurement(z, t));
}

void EkfLocalizer::odomReceived(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  if (!filter_.ekf || !filter_.odom) {return;}

  // twist は child_frame_id（base）座標なので linear.x, angular.z を v_xy, omega_yaw の観測にする（倍率 odom_scale を掛ける）
  const double v = msg->twist.twist.linear.x * params_.odom_scale(0);
  const double omega = msg->twist.twist.angular.z * params_.odom_scale(1);
  // 時刻は odom_stamp_mode で stamp か受け取った時刻
  const builtin_interfaces::msg::Time stamp = odomStamp(msg->header.stamp);
  // 観測ノイズは R_odom と速さ・加速度から計算する（メッセージの共分散は使わない）
  const Eigen::Vector2d r =
    filter_.odom_noise->variance(rclcpp::Time(stamp, RCL_ROS_TIME).seconds(), v, omega);
  // twist が表す動きは stamp より odom_delay だけ前のもの（diff_drive_controller の移動平均など）
  const double t = measurementTime(stamp, params_.odom_delay);
  addMeasurement(filter_.odom->measurement(v, omega, r, t));
}

void EkfLocalizer::initialPoseReceived(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
{
  if (msg->header.frame_id != params_.map_frame_id) {return;}
  if (!filter_.ekf) {return;}
  // 姿勢を与え直したので、次の NDT 観測で初期化し直す（ゲートに引っかかり続けるのを防ぐ）
  filter_.ekf->reset();
  RCLCPP_INFO(get_logger(), "initialpose received: EKF reset.");
}

void EkfLocalizer::timerCallback()
{
  if (!filter_.ekf && !tryCreateFilter()) {return;}
  if (!filter_.ekf->initialized()) {return;}

  // 現在時刻まで予測して出す。オドメトリの stamp が現在時刻より先のこともある
  // （sim time の /clock が粗い場合）ので、そのときは最新の観測の時刻で出す
  const double t = std::max(now().seconds(), filter_.ekf->current().time());
  const rclcpp::Time stamp = toTime(t);
  if (stamp <= last_publish_stamp_) {return;}
  last_publish_stamp_ = stamp;

  const EkfCore ekf = filter_.ekf->predicted(t);
  if (params_.tf_future_steps > 0) {
    publishTfGrid(stamp.nanoseconds());
  } else {
    tf_broadcaster_->sendTransform(baseTransformMsg(ekf, stamp));
  }
  publishPoseAndOdom(ekf, stamp);
}

void EkfLocalizer::publishTfGrid(int64_t t_ns)
{
  // 刻み k·T の TF を、今より後の tf_future_steps 個と今以前の tf_past_steps 個、1 つのメッセージで送る。
  // 同じ刻みは周期ごとに新しい値で送り直し、購読側の tf2 で置き換わる。値は t 以前の観測だけを入れた状態を
  // その刻みまで予測したもの（未来の刻みは最新の状態の予測、過ぎた刻みは遅れて届いた観測も入れた値）
  std::vector<geometry_msgs::msg::TransformStamped> tfs;
  for (const int64_t k_ns : tfGridStamps(
      t_ns, period_ns_, params_.tf_future_steps, params_.tf_past_steps))
  {
    const double tk = static_cast<double>(k_ns) * 1e-9;
    const EkfCore ekf = filter_.ekf->at(tk);
    // 初期化より前の刻みと、履歴の起点より前の刻み（起点の時刻のまま進まない）は送らない
    if (!ekf.initialized() || ekf.time() > tk) {continue;}
    tfs.push_back(baseTransformMsg(ekf, rclcpp::Time(k_ns, RCL_ROS_TIME)));
  }
  if (!tfs.empty()) {tf_broadcaster_->sendTransform(tfs);}
}

geometry_msgs::msg::TransformStamped EkfLocalizer::baseTransformMsg(
  const EkfCore & ekf, const rclcpp::Time & stamp) const
{
  geometry_msgs::msg::TransformStamped tf = tf2::eigenToTransform(ekf.basePose());
  tf.header.stamp = stamp;
  tf.header.frame_id = params_.map_frame_id;
  tf.child_frame_id = params_.base_frame_id;
  return tf;
}

geometry_msgs::msg::PoseWithCovarianceStamped EkfLocalizer::basePoseMsg(
  const EkfCore & ekf, const rclcpp::Time & stamp) const
{
  geometry_msgs::msg::PoseWithCovarianceStamped pose;
  pose.header.stamp = stamp;
  pose.header.frame_id = params_.map_frame_id;
  pose.pose.pose = tf2::toMsg(ekf.basePose());
  Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(pose.pose.covariance.data()) =
    ekf.covariance().topLeftCorner<6, 6>();
  return pose;
}

geometry_msgs::msg::PoseWithCovarianceStamped EkfLocalizer::lidarPoseMsg(
  const EkfCore & ekf, const rclcpp::Time & stamp) const
{
  // map → LiDAR（NDT と同じ基準）。共分散は LiDAR の姿勢の状態に対するヤコビアン J で J P J^T。
  // 取付オフセットのレバーアームで roll・pitch・yaw の不確かさが位置に漏れる分が入る
  geometry_msgs::msg::PoseWithCovarianceStamped pose;
  pose.header.stamp = stamp;
  pose.header.frame_id = params_.map_frame_id;
  pose.pose.pose = tf2::toMsg(filter_.ndt->lidarPose(ekf.state()));
  const Eigen::Matrix<double, 6, 6> J =
    filter_.ndt->lidarPoseJacobian(ekf.state()).leftCols<6>();
  Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(pose.pose.covariance.data()) =
    J * ekf.covariance().topLeftCorner<6, 6>() * J.transpose();
  return pose;
}

void EkfLocalizer::publishPoseAndOdom(const EkfCore & ekf, const rclcpp::Time & stamp)
{
  // /ekf_pose：map → LiDAR
  ekf_pose_pub_->publish(lidarPoseMsg(ekf, stamp));

  // /ekf_odom：map → base と、v_xy, omega_yaw（linear.x, angular.z）。
  // v_z, omega_roll, omega_pitch は地図の z とオイラー角の変化率で base 座標の twist と合わないので出さない
  const geometry_msgs::msg::PoseWithCovarianceStamped base = basePoseMsg(ekf, stamp);
  nav_msgs::msg::Odometry odom;
  odom.header = base.header;
  odom.child_frame_id = params_.base_frame_id;
  odom.pose = base.pose;
  odom.twist.twist.linear.x = ekf.vXy();
  odom.twist.twist.angular.z = ekf.omegaYaw();
  const auto & P = ekf.covariance();
  odom.twist.covariance[0] = P(EkfCore::kVxy, EkfCore::kVxy);
  odom.twist.covariance[5] = P(EkfCore::kVxy, EkfCore::kOmegaYaw);
  odom.twist.covariance[30] = P(EkfCore::kOmegaYaw, EkfCore::kVxy);
  odom.twist.covariance[35] = P(EkfCore::kOmegaYaw, EkfCore::kOmegaYaw);
  ekf_odom_pub_->publish(odom);
}

}  // namespace ekf_localizer
