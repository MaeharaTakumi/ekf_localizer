#include "ekf_localizer/ekf_localizer_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace ekf_localizer
{

namespace
{

rclcpp::Time toTime(double t)
{
  return rclcpp::Time(static_cast<int64_t>(std::llround(t * 1e9)), RCL_ROS_TIME);
}

using Status = LaggedEkf::Status;

}  // namespace

EkfLocalizer::EkfLocalizer(const rclcpp::NodeOptions & options)
: rclcpp::Node("ekf_localizer", options),
  last_publish_stamp_(0, 0, RCL_ROS_TIME)
{
  declareEkfParameters(*this);
  cfg_ = loadEkfConfig(*this);
  logEkfConfig(get_logger(), cfg_);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // NDT の解は 1 つも落としたくないので reliable
  ndt_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    cfg_.ndt_pose_topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
    std::bind(&EkfLocalizer::ndtPoseReceived, this, std::placeholders::_1));
  // best effort は reliable / best effort どちらの publisher とも接続できる（Real と Gazebo で共通）。
  // 深さは処理が詰まったときに捨てない程度
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    cfg_.odom_topic, rclcpp::QoS(rclcpp::KeepLast(50)).best_effort(),
    std::bind(&EkfLocalizer::odomReceived, this, std::placeholders::_1));
  initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    cfg_.initialpose_topic, rclcpp::SystemDefaultsQoS(),
    std::bind(&EkfLocalizer::initialPoseReceived, this, std::placeholders::_1));

  // /ekf_pose は map → LiDAR。後から購読したノードも最後の値を受け取れるよう transient_local
  ekf_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "ekf_pose", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  ekf_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("ekf_odom", rclcpp::QoS(10));

  if (cfg_.publish_trigger == "cloud") {
    // 点群は stamp しか使わない。best effort は reliable / best effort どちらの publisher とも接続できる
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cfg_.points_topic, rclcpp::SensorDataQoS(),
      std::bind(&EkfLocalizer::cloudReceived, this, std::placeholders::_1));
    return;
  }

  // ROS 時刻のタイマー（use_sim_time ならシミュレーション時刻で回る）
  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / cfg_.predict_rate));
  timer_ = rclcpp::create_timer(
    this, get_clock(), period, std::bind(&EkfLocalizer::timerCallback, this));
}

bool EkfLocalizer::tryCreateFilter()
{
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(cfg_.base_frame_id, cfg_.lidar_frame_id, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Waiting for TF %s -> %s: %s",
      cfg_.base_frame_id.c_str(), cfg_.lidar_frame_id.c_str(), ex.what());
    return false;
  }
  const Eigen::Affine3d mount = tf2::transformToEigen(tf);
  filter_ = createFilter(cfg_, mount);

  const Eigen::Vector3d t = mount.translation();
  const Eigen::Vector3d rpy = rpyFromRotation(mount.linear());
  RCLCPP_INFO(
    get_logger(), "mount %s -> %s (from TF): t=[%.3f, %.3f, %.3f], rpy=[%.4f, %.4f, %.4f]",
    cfg_.base_frame_id.c_str(), cfg_.lidar_frame_id.c_str(),
    t.x(), t.y(), t.z(), rpy(0), rpy(1), rpy(2));
  return true;
}

void EkfLocalizer::addMeasurement(const MeasurementPtr & m)
{
  const ObservationModel & model = *m->model;
  const LaggedEkf::Result r = filter_.lagged->add(m);

  if (r.status == Status::kTooOld) {
    const rclcpp::Time t_now = now();
    const auto it = too_old_warned_.find(&model);
    if (it == too_old_warned_.end() || (t_now - it->second).seconds() >= 5.0) {
      RCLCPP_WARN(
        get_logger(), "%s is older than the history (%.3f s behind the latest). "
        "Increase history_length.", model.name(), filter_.lagged->latestStamp() - m->stamp);
      too_old_warned_.insert_or_assign(&model, t_now);
    }
    return;
  }

  const Outcome & o = r.outcome;
  if (o.initialized_now) {
    const EkfCore & ekf = filter_.lagged->current();
    const MeasurementPtr & aid = ekf.initAid();
    RCLCPP_INFO(
      get_logger(), "EKF initialized by %s (v, omega = %.3f, %.3f from %s).", model.name(),
      ekf.velocity(), ekf.angularVelocity(), o.aided ? aid->model->name() : "zero");
    return;
  }

  using S = StageResult::Status;
  for (const StageResult & st : o.stages) {
    if (st.status == S::kRejected) {
      RCLCPP_WARN(
        get_logger(), "%s %s rejected by gate (d2 = %lf, %d times in a row).",
        model.name(), st.name, st.d2, st.reject_count);
    } else if (st.status == S::kReinitialized) {
      RCLCPP_WARN(
        get_logger(), "%s %s: lockout -> reinitialized to the observation (%lf).",
        model.name(), st.name, st.reinitialized_to);
    }
  }
}

void EkfLocalizer::ndtPoseReceived(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
{
  if (!filter_.lagged) {return;}

  // NDT の解は map → LiDAR の姿勢。共分散は使わない（観測ノイズは R_ndt）
  const auto & p = msg->pose.pose;
  double roll, pitch, yaw;
  tf2::Matrix3x3(
    tf2::Quaternion(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w))
  .getRPY(roll, pitch, yaw);
  NdtModel::Vector6d z;
  z << p.position.x, p.position.y, p.position.z, roll, pitch, yaw;

  // 点群の取得時刻が stamp より ndt_delay だけ前のときに補う
  const double stamp =
    rclcpp::Time(msg->header.stamp, RCL_ROS_TIME).seconds() - cfg_.ndt_delay;
  addMeasurement(filter_.ndt->measurement(z, stamp));
}

void EkfLocalizer::odomReceived(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  if (!filter_.lagged || !filter_.odom) {return;}

  Eigen::Vector2d r = cfg_.odom.r;
  if (cfg_.odom_covariance_source == "message") {
    // twist.covariance は 6x6 行優先。[0] が v_x、[35] が omega_z の分散
    const Eigen::Vector2d r_msg(msg->twist.covariance[0], msg->twist.covariance[35]);
    if (r_msg.allFinite() && (r_msg.array() > 0.0).all()) {
      r = r_msg;
    } else {
      RCLCPP_WARN_ONCE(
        get_logger(), "Odometry twist covariance is not positive ([0]=%g, [35]=%g). Using R_odom.",
        r_msg(0), r_msg(1));
    }
  }

  // twist は child_frame_id（base）座標なので linear.x, angular.z がそのまま v, omega
  const double v = msg->twist.twist.linear.x;
  const double omega = msg->twist.twist.angular.z;
  // twist が表す動きは stamp より odom_delay だけ前のもの（diff_drive_controller の移動平均など）
  const double stamp =
    rclcpp::Time(msg->header.stamp, RCL_ROS_TIME).seconds() - cfg_.odom_delay;
  addMeasurement(filter_.odom->measurement(v, omega, r, stamp));
}

void EkfLocalizer::initialPoseReceived(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
{
  if (msg->header.frame_id != cfg_.map_frame_id) {return;}
  if (!filter_.lagged) {return;}
  // 姿勢を与え直したので、次の NDT 観測で初期化し直す（ゲートに引っかかり続けるのを防ぐ）
  filter_.lagged->reset();
  RCLCPP_INFO(get_logger(), "initialpose received: EKF reset.");
}

void EkfLocalizer::cloudReceived(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  if (!filter_.lagged && !tryCreateFilter()) {return;}
  if (!filter_.lagged->initialized()) {return;}

  // 点群の取得時刻（stamp）の予測を出す。この点群の NDT はまだ届いていないので、
  // stamp 以前の観測（前の NDT とオドメトリ）からの予測になる。NDT の初期値にもこの TF が使われる
  const rclcpp::Time stamp(msg->header.stamp, RCL_ROS_TIME);
  if (stamp <= last_publish_stamp_) {return;}
  last_publish_stamp_ = stamp;

  publish(filter_.lagged->at(stamp.seconds()), stamp);
}

void EkfLocalizer::timerCallback()
{
  if (!filter_.lagged && !tryCreateFilter()) {return;}
  if (!filter_.lagged->initialized()) {return;}

  // 現在時刻まで予測して出す。オドメトリの stamp が現在時刻より先のこともある
  // （sim time の /clock が粗い場合）ので、そのときは最新の観測の時刻で出す
  const double t = std::max(now().seconds(), filter_.lagged->current().lastStamp());
  const rclcpp::Time stamp = toTime(t);
  if (stamp <= last_publish_stamp_) {return;}
  last_publish_stamp_ = stamp;

  publish(filter_.lagged->predicted(t), stamp);
}

void EkfLocalizer::publish(const EkfCore & ekf, const rclcpp::Time & stamp)
{
  const Eigen::Affine3d T_mb = ekf.basePose();
  const auto P6 = ekf.covariance().topLeftCorner<6, 6>();

  if (cfg_.publish_tf) {
    geometry_msgs::msg::TransformStamped tf = tf2::eigenToTransform(T_mb);
    tf.header.stamp = stamp;
    tf.header.frame_id = cfg_.map_frame_id;
    tf.child_frame_id = cfg_.base_frame_id;
    tf_broadcaster_->sendTransform(tf);
  }

  // /ekf_pose：map → LiDAR（NDT と同じ基準）。共分散は観測モデル h のヤコビアン J で J P J^T。
  // 取付オフセットのレバーアームで roll・pitch・yaw の不確かさが位置に漏れる分が入る
  geometry_msgs::msg::PoseWithCovarianceStamped pose;
  pose.header.stamp = stamp;
  pose.header.frame_id = cfg_.map_frame_id;
  pose.pose.pose = tf2::toMsg(filter_.ndt->lidarPose(ekf.state()));
  const Eigen::Matrix<double, 6, 6> J =
    filter_.ndt->observationJacobian(ekf.state()).leftCols<6>();
  Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(pose.pose.covariance.data()) =
    J * P6 * J.transpose();
  ekf_pose_pub_->publish(pose);

  // /ekf_odom：map → base と、base 座標の v, omega
  nav_msgs::msg::Odometry odom;
  odom.header = pose.header;
  odom.child_frame_id = cfg_.base_frame_id;
  odom.pose.pose = tf2::toMsg(T_mb);
  Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(odom.pose.covariance.data()) = P6;
  odom.twist.twist.linear.x = ekf.velocity();
  odom.twist.twist.angular.z = ekf.angularVelocity();
  const auto & P = ekf.covariance();
  odom.twist.covariance[0] = P(EkfCore::kV, EkfCore::kV);
  odom.twist.covariance[5] = P(EkfCore::kV, EkfCore::kOmega);
  odom.twist.covariance[30] = P(EkfCore::kOmega, EkfCore::kV);
  odom.twist.covariance[35] = P(EkfCore::kOmega, EkfCore::kOmega);
  ekf_odom_pub_->publish(odom);
}

}  // namespace ekf_localizer
