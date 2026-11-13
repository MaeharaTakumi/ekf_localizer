#ifndef EKF_LOCALIZER__POSE_UTILS_HPP_
#define EKF_LOCALIZER__POSE_UTILS_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ekf_localizer
{

/// 6 自由度の姿勢 [x, y, z, roll, pitch, yaw]（回転は ZYX）
using Vector6d = Eigen::Matrix<double, 6, 1>;

/// 角度を (-pi, pi] に正規化する
double normalizeAngle(double angle);
/// 回転行列 → [roll, pitch, yaw]（tf2 の getRPY と同じ ZYX 規約）
Eigen::Vector3d rpyFromRotation(const Eigen::Matrix3d & R);
/// [roll, pitch, yaw] → 回転行列（ZYX）
Eigen::Matrix3d rotationFromRpy(double roll, double pitch, double yaw);

/// 姿勢の行列 → [x, y, z, roll, pitch, yaw]
Vector6d poseToVector(const Eigen::Affine3d & T);
/// [x, y, z, roll, pitch, yaw] → 姿勢の行列
Eigen::Affine3d poseFromVector(const Vector6d & v);
/// 姿勢の差 a − b の角度成分（roll, pitch, yaw）を (-pi, pi] に正規化する
void wrapPoseAngles(Vector6d & d);

}  // namespace ekf_localizer

#endif  // EKF_LOCALIZER__POSE_UTILS_HPP_
