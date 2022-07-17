#pragma once

namespace rocket
{
struct Position
{
  double x;
  double y;
  double z;
};

struct Vector3
{
  double x;
  double y;
  double z;
};

struct Quaternion
{
  double x;
  double y;
  double z;
  double w;
};

struct Pose
{
  Position position;
  Quaternion orientation;
};

struct Twist
{
  Vector3 velocity;
  Vector3 angular_velocity;
};

struct RocketState
{
  Position position;
  Vector3 velocity;
  Quaternion orientation;
  Vector3 angular_velocity;
  double propeller_mass;

public:
  inline Pose getPose() const
  {
    return Pose{ position, orientation };
  }

  inline Twist getTwist() const
  {
    return Twist{ velocity, angular_velocity };
  }
};

struct Wrench
{
  Vector3 force;
  Vector3 torque;
};

struct RocketGimbalControl
{
  double outer_angle;
  double inner_angle;
  double thrust;
};

struct RocketControlMomentGyro
{
  double outer_angle;
  double inner_angle;
  double torque;
};

enum RocketFSMState
{
  CALIBRATION,
  IDLE,
  LAUNCH,
  RAIL,
  ASCENT,
  COAST,
  LANDING,
  STOP
};
}  // namespace rocket