#pragma once

#include "rocket_types.h"

#include "ros/ros.h"
#include <rocket_utils/FSM.h>
#include <rocket_utils/State.h>
#include <rocket_utils/GimbalControl.h>
#include <rocket_utils/ControlMomentGyro.h>

namespace rocket
{

// Conversions to ROS messages

inline geometry_msgs::Point toROS(const Position& position)
{
  geometry_msgs::Point position_ros;
  position_ros.x = position.x;
  position_ros.y = position.y;
  position_ros.z = position.z;
  return position_ros;
}

inline geometry_msgs::Vector3 toROS(const Vector3& vec)
{
  geometry_msgs::Vector3 vec_ros;
  vec_ros.x = vec.x;
  vec_ros.y = vec.y;
  vec_ros.z = vec.z;
  return vec_ros;
}

inline geometry_msgs::Quaternion toROS(const Quaternion& quat)
{
  geometry_msgs::Quaternion quat_ros;
  quat_ros.x = quat.x;
  quat_ros.y = quat.y;
  quat_ros.z = quat.z;
  quat_ros.w = quat.w;
  return quat_ros;
}

inline geometry_msgs::Pose toROS(const Pose& pose)
{
  geometry_msgs::Pose pose_ros;
  pose_ros.position = toROS(pose.position);
  pose_ros.orientation = toROS(pose.orientation);
  return pose_ros;
}

inline geometry_msgs::Twist toROS(const Twist& pose)
{
  geometry_msgs::Twist pose_ros;
  pose_ros.linear = toROS(pose.velocity);
  pose_ros.angular = toROS(pose.angular_velocity);
  return pose_ros;
}

inline rocket_utils::State toROS(const RocketState& state)
{
  rocket_utils::State state_ros;
  state_ros.pose = toROS(state.getPose());
  state_ros.twist = toROS(state.getTwist());
  return state_ros;
}

inline rocket_utils::GimbalControl toROS(const RocketGimbalControl& control)
{
  rocket_utils::GimbalControl control_ros;
  control_ros.outer_angle = control.outer_angle;
  control_ros.inner_angle = control.inner_angle;
  ;
  control_ros.thrust = control.thrust;
  return control_ros;
}

inline rocket_utils::ControlMomentGyro toROS(const RocketControlMomentGyro& control)
{
  rocket_utils::ControlMomentGyro control_ros;
  control_ros.outer_angle = control.outer_angle;
  control_ros.inner_angle = control.inner_angle;
  ;
  control_ros.torque = control.torque;
  return control_ros;
}

inline rocket_utils::FSM toROS(const RocketFSMState& fsm)
{
  rocket_utils::FSM fsm_msg;
  if (fsm == RocketFSMState::IDLE)
    fsm_msg.state_machine = rocket_utils::FSM::IDLE;
  else if (fsm == RocketFSMState::RAIL)
    fsm_msg.state_machine = rocket_utils::FSM::RAIL;
  else if (fsm == RocketFSMState::LAUNCH)
    fsm_msg.state_machine = rocket_utils::FSM::LAUNCH;
  else if (fsm == RocketFSMState::ASCENT)
    fsm_msg.state_machine = rocket_utils::FSM::ASCENT;
  else if (fsm == RocketFSMState::DESCENT)
    fsm_msg.state_machine = rocket_utils::FSM::DESCENT;
  else if (fsm == RocketFSMState::COAST)
    fsm_msg.state_machine = rocket_utils::FSM::COAST;
  else if (fsm == RocketFSMState::STOP)
    fsm_msg.state_machine = rocket_utils::FSM::STOP;
  else
    throw std::runtime_error("Invalid FSM state");
  return fsm_msg;
}

// Conversions from ROS messages

inline Position fromROS(const geometry_msgs::Point& position)
{
  return Position{ position.x, position.y, position.z };
}

inline Vector3 fromROS(const geometry_msgs::Vector3& vec)
{
  return Vector3{ vec.x, vec.y, vec.z };
}

inline Quaternion fromROS(const geometry_msgs::Quaternion& quat)
{
  return Quaternion{ quat.x, quat.y, quat.z, quat.w };
}

inline Pose fromROS(const geometry_msgs::Pose& pose)
{
  return Pose{ fromROS(pose.position), fromROS(pose.orientation) };
}

inline Twist fromROS(const geometry_msgs::Twist& twist)
{
  return Twist{ fromROS(twist.linear), fromROS(twist.angular) };
}

inline RocketState fromROS(const rocket_utils::State& state)
{
  return RocketState{ fromROS(state.pose.position), fromROS(state.twist.linear), fromROS(state.pose.orientation),
                      fromROS(state.twist.angular), state.propeller_mass };
}

inline RocketGimbalControl fromROS(const rocket_utils::GimbalControl& control)
{
  return RocketGimbalControl{ control.outer_angle, control.inner_angle, control.thrust };
}

inline RocketControlMomentGyro fromROS(const rocket_utils::ControlMomentGyro& control)
{
  return RocketControlMomentGyro{ control.outer_angle, control.inner_angle, control.torque };
}

inline RocketFSMState fromROS(const rocket_utils::FSM& fsm_msg)
{
  std::string fsm = fsm_msg.state_machine;
  if (fsm == rocket_utils::FSM::IDLE)
    return RocketFSMState::IDLE;
  else if (fsm == rocket_utils::FSM::RAIL)
    return RocketFSMState::RAIL;
  else if (fsm == rocket_utils::FSM::LAUNCH)
    return RocketFSMState::LAUNCH;
  else if (fsm == rocket_utils::FSM::ASCENT)
    return RocketFSMState::ASCENT;
  else if (fsm == rocket_utils::FSM::DESCENT)
    return RocketFSMState::DESCENT;
  else if (fsm == rocket_utils::FSM::COAST)
    return RocketFSMState::COAST;
  else if (fsm == rocket_utils::FSM::STOP)
    return RocketFSMState::STOP;
  else
    throw std::runtime_error("Invalid FSM state");
}
}  // namespace rocket
