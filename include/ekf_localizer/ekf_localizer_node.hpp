#ifndef EKF_LOCALIZER__EKF_LOCALIZER_NODE_HPP_
#define EKF_LOCALIZER__EKF_LOCALIZER_NODE_HPP_

#include <map>
#include <memory>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include "ekf_localizer/ekf_params.hpp"

namespace ekf_localizer
{

/// NDT とオドメトリを融合する EKF ノード
///
///   入力  ndt_pose（map → LiDAR、stamp は点群の取得時刻）、odom（v, omega）、initialpose（リセット）
///         トピック名は ndt_pose_topic / odom_topic / initialpose_topic で変えられる
///   出力  predict_rate の周期で、最新の状態を現在時刻まで予測した TF map → base、ekf_pose（map → LiDAR）、
///         ekf_odom（map → base、v, omega）。NDT は点群の stamp の時刻の TF を初期値にし、MATLAB などは
///         最新の TF（今の姿勢）と点群の stamp の時刻の TF（点群の座標変換）を引く。
///         tf_future_steps >= 1 なら TF は周期の刻みの stamp で、今より先の刻みも送る（購読側が点群の stamp の
///         TF を待たずに引けるように。同じ刻みは周期ごとに送り直して置き換える）
///
///   観測は届いた時点で、観測モデルで観測にして LaggedEkf に入れる。観測の時刻は time_source で選ぶ。
///     stamp    メッセージの stamp。NDT は遅れて届くので、点群の取得時刻まで巻き戻してそれ以降のオドメトリを適用し直す
///     receipt  受信した時刻。オドメトリはこのノードが受信した時刻、NDT はその点群（points_topic）を受信した時刻
///              （NDT のノードに入っていたころと同じ。NDT は計算の分だけ遅れて届くので、点群の時刻まで巻き戻して入れる）
///   取付 base → LiDAR は TF（静的）から読む。読めるまでは観測を捨てる。
class EkfLocalizer : public rclcpp::Node
{
public:
  explicit EkfLocalizer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void ndtPoseReceived(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg);
  void odomReceived(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void initialPoseReceived(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg);
  void cloudReceived(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void timerCallback();
  /// 取付 base → LiDAR を TF から読んで EKF を作る（タイマーから呼ぶ）。作れたら true
  bool tryCreateFilter();
  /// オドメトリの観測の時刻 [s]（time_source に従う。delay を引く）
  double measurementTime(const builtin_interfaces::msg::Time & stamp, double delay);
  /// NDT の観測の時刻 [s]。receipt では、同じ stamp の点群を受信した時刻（無ければ NDT を受信した時刻）
  double ndtTime(const builtin_interfaces::msg::Time & stamp);
  /// 観測を LaggedEkf に入れ、結果をログに出す（どのセンサも共通）
  void addMeasurement(const MeasurementPtr & m);
  /// ekf の推定値を stamp の時刻として ekf_pose・ekf_odom に配信する（TF は timerCallback が送る）
  void publish(const EkfCore & ekf, const rclcpp::Time & stamp);
  /// tf_future_steps >= 1 のとき：時刻 t_ns の前後の刻みの TF map → base をまとめて送る
  void publishTfGrid(int64_t t_ns);
  /// TF map → base
  geometry_msgs::msg::TransformStamped tfMsg(const EkfCore & ekf, const rclcpp::Time & stamp) const;
  /// map → base の姿勢と共分散（base の 6x6）
  geometry_msgs::msg::PoseWithCovarianceStamped basePoseMsg(
    const EkfCore & ekf, const rclcpp::Time & stamp) const;
  /// map → LiDAR の姿勢と共分散（LiDAR の姿勢の状態に対するヤコビアン J で J P J^T）
  geometry_msgs::msg::PoseWithCovarianceStamped lidarPoseMsg(
    const EkfCore & ekf, const rclcpp::Time & stamp) const;

  EkfConfig cfg_;
  /// EKF 一式（filter_.lagged は取付が TF から読めるまで null）
  EkfFilter filter_;
  /// 観測モデルごとの、「古すぎる」WARN を最後に出した時刻（5 秒に 1 回にする）
  std::map<const ObservationModel *, rclcpp::Time> too_old_warned_;
  /// 最後に配信した時刻（同じ時刻で 2 回出さない）
  rclcpp::Time last_publish_stamp_;
  /// タイマーの周期と TF の刻みの幅 [ns]（1 / predict_rate）
  int64_t period_ns_{20000000};
  /// time_source: receipt で、点群の stamp [ns] → 受信した時刻 [s]（history_length より古いものは捨てる）
  std::map<int64_t, double> cloud_receipt_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr ndt_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr ekf_pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ekf_odom_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace ekf_localizer

#endif  // EKF_LOCALIZER__EKF_LOCALIZER_NODE_HPP_
