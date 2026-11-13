#include "ekf_localizer/pose_utils.hpp"

#include <algorithm>
#include <cmath>

namespace ekf_localizer
{

double normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

Eigen::Vector3d rpyFromRotation(const Eigen::Matrix3d & R)
{
  return Eigen::Vector3d(
    std::atan2(R(2, 1), R(2, 2)),
    std::asin(std::max(-1.0, std::min(1.0, -R(2, 0)))),
    std::atan2(R(1, 0), R(0, 0)));
}

Eigen::Matrix3d rotationFromRpy(double roll, double pitch, double yaw)
{
  return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
}

Vector6d poseToVector(const Eigen::Affine3d & T)
{
  Vector6d v;
  v.head<3>() = T.translation();
  v.tail<3>() = rpyFromRotation(T.linear());
  return v;
}

Eigen::Affine3d poseFromVector(const Vector6d & v)
{
  Eigen::Affine3d T = Eigen::Affine3d::Identity();
  T.linear() = rotationFromRpy(v(3), v(4), v(5));
  T.translation() = v.head<3>();
  return T;
}

void wrapPoseAngles(Vector6d & d)
{
  for (int i = 3; i < 6; ++i) {d(i) = normalizeAngle(d(i));}
}

}  // namespace ekf_localizer
