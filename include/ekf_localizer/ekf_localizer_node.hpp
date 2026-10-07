#ifndef EKF_LOCALIZER__EKF_LOCALIZER_NODE_HPP_
#define EKF_LOCALIZER__EKF_LOCALIZER_NODE_HPP_

#include <memory>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include "ekf_localizer/ekf_params.hpp"
#include "ekf_localizer/lagged_ekf.hpp"

namespace ekf_localizer
{

/// NDT とオドメトリを融合する EKF ノード
///
///   入力  ndt_pose（map → LiDAR、stamp は点群の取得時刻）、odom（v, omega）、initialpose（リセット）
///         トピック名は ndt_pose_topic / odom_topic / initialpose_topic で変えられる
///   出力  ekf_pose（map → LiDAR）、ekf_odom（map → base、v, omega）、TF map → base
///
///   観測は届いた時点で LaggedEkf に入れる。NDT は遅れて届くので、点群の取得時刻まで巻き戻して
///   それ以降のオドメトリを適用し直す。出力は predict_rate の周期で、最新の状態を現在時刻まで予測して出す。
///   取付 base → LiDAR は TF（静的）から読む。読めるまでは観測を捨てる。
class EkfLocalizer : public rclcpp::Node
{
public:
  explicit EkfLocalizer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void ndtPoseReceived(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg);
  void odomReceived(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void initialPoseReceived(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg);
  void timerCallback();
  /// 取付 base → LiDAR を TF から読んで EKF を作る。作れたら true
  bool tryCreateFilter();
  /// ekf の推定値を stamp の時刻として配信する
  void publish(const VehicleEkf & ekf, const rclcpp::Time & stamp);

  EkfConfig cfg_;
  /// 取付が TF から読めるまで null
  std::unique_ptr<LaggedEkf> filter_;
  /// 最後に配信した時刻（同じ時刻で 2 回出さない）
  rclcpp::Time last_publish_stamp_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr ndt_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr ekf_pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ekf_odom_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace ekf_localizer

#endif  // EKF_LOCALIZER__EKF_LOCALIZER_NODE_HPP_
