#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>

#include "rocket_types.h"

namespace rocket
{

// Conversions from Eigen matrices

inline Position toPosition(const Eigen::Vector3d& position)
{
  return Position{ position(0), position(1), position(2) };
}

inline Vector3 toVector3(const Eigen::Vector3d& vec)
{
  return Vector3{ vec(0), vec(1), vec(2) };
}

inline Quaternion toQuaternion(const Eigen::Vector4d& quat)
{
  return Quaternion{ quat.x(), quat.y(), quat.z(), quat.w() };
}

inline Quaternion toQuaternion(const Eigen::Quaterniond& quat)
{
  return Quaternion{ quat.x(), quat.y(), quat.z(), quat.w() };
}

inline Pose toPose(const Eigen::Matrix<double, 7, 1>& vec)
{
  return Pose{ toPosition(vec.segment(0, 3)), toQuaternion(vec.segment(3, 4)) };
}

inline RocketState toRocketState(const Eigen::Matrix<double, 14, 1>& vec)
{
  return RocketState{ toPosition(vec.segment(0, 3)), toVector3(vec.segment(3, 3)), toQuaternion(vec.segment(6, 4)),
                      toVector3(vec.segment(10, 3)), vec(13) };
}

// Conversions to Eigen matrices

inline Eigen::Vector3d toEigen(const Position& position)
{
  return Eigen::Vector3d{ position.x, position.y, position.z };
}

inline Eigen::Vector3d toEigen(const Vector3& vec)
{
  return Eigen::Vector3d{ vec.x, vec.y, vec.z };
}

inline Eigen::Vector4d toEigen(const Quaternion& quat)
{
  return Eigen::Vector4d{ quat.x, quat.y, quat.z, quat.w };
}

inline Eigen::Quaterniond toEigenQuaternion(const Quaternion& quat)
{
  return Eigen::Quaterniond{ quat.w, quat.x, quat.y, quat.z };
}

inline Eigen::Matrix<double, 7, 1> toEigen(const Pose& pose)
{
  Eigen::Matrix<double, 7, 1> pose_eigen;
  pose_eigen << toEigen(pose.position), toEigen(pose.orientation);
  return pose_eigen;
}

inline Eigen::Matrix<double, 6, 1> toEigen(const Twist& twist)
{
  Eigen::Matrix<double, 6, 1> twist_eigen;
  twist_eigen << toEigen(twist.velocity), toEigen(twist.angular_velocity);
  return twist_eigen;
}

inline Eigen::Vector3d toEigen(const RocketGimbalControl& control)
{
  return Eigen::Vector3d{ control.outer_angle, control.inner_angle, control.thrust };
}

inline Eigen::Matrix<double, 14, 1> toEigen(const RocketState& state)
{
  Eigen::Matrix<double, 14, 1> state_eigen;
  state_eigen << toEigen(state.position), toEigen(state.velocity), toEigen(state.orientation),
      toEigen(state.angular_velocity);
  return state_eigen;
}
}  // namespace rocket