#pragma once

#include <time.h>
#include <sstream>
#include <string>
#include <iostream>
#include <fstream>

#include <boost/numeric/odeint.hpp>
#include <boost/numeric/odeint/stepper/runge_kutta_dopri5.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <chrono>
#include <random>

struct RocketProps
{
  double dry_mass;
  double Isp;
  double minTorque;
  double maxTorque;

  std::vector<double> maxThrust{ 0, 0, 0 };
  std::vector<double> minThrust{ 0, 0, 0 };

  double dry_CM;
  double propellant_CM;
  double initial_propellant_mass;

  double total_length;

  double initial_speed;

  double h0;

  std::vector<double> target_apogee = { 0, 0, 0 };
  std::vector<double> Cd = { 0, 0, 0 };
  std::vector<double> surface = { 0, 0, 0 };
  std::vector<double> drag_coeff = { 0, 0, 0 };

  std::vector<double> dry_Inertia{ 0, 0, 0 };

  double acc_noise, acc_bias;
  double gyro_noise, gyro_bias;
  double baro_noise, baro_bias;

  std::vector<double> diameter = { 0, 0, 0 };
  std::vector<double> length = { 0, 0, 0 };

  int nStage;

  std::vector<double> full_thrust;
  std::vector<double> full_thrust_time;

  void init()
  {
    total_length = length[nStage - 1];

    surface[0] = diameter[1] * total_length;
    surface[1] = surface[0];
    surface[2] = diameter[1] * diameter[1] / 4 * 3.14159;

    double rho_air = 1.225;
    drag_coeff[0] = 0.5 * rho_air * surface[0] * Cd[0];
    drag_coeff[1] = 0.5 * rho_air * surface[1] * Cd[1];
    drag_coeff[2] = 0.5 * rho_air * surface[2] * Cd[2];
  }
};

class RocketModel
{
public:
  RocketProps props;

  RocketModel(RocketProps& props_) : props(props_)
  {
    props.init();

    total_Inertia[2] = props.dry_Inertia[2];
    propellant_mass = props.initial_propellant_mass;

    updateCm(propellant_mass);
  }

  double getFullThrust(double time_thrust)
  {
    int i;
    for (i = 0; i < (int)props.full_thrust_time.size(); i++)
    {
      if (time_thrust < props.full_thrust_time[i])
      {
        break;
      }
    }
    i--;

    double interp_thrust;
    if (time_thrust < props.full_thrust_time.back())
      interp_thrust = (props.full_thrust[i] + (time_thrust - props.full_thrust_time[i]) /
                                                  (props.full_thrust_time[i + 1] - props.full_thrust_time[i]) *
                                                  (props.full_thrust[i + 1] - props.full_thrust[i]));

    else
      interp_thrust = props.full_thrust_time.back();

    if (time_thrust < 0)
      interp_thrust = 0;

    return interp_thrust;
  }

private:
  double total_CM;  // Current Cm of rocket, in real time
  std::vector<double> total_Inertia{ 0, 0, 0 };
  std::vector<double> J_inv{ 0, 0, 0 };
  double propellant_mass;

  void updateCm(double current_prop_mass)
  {
    total_CM = props.total_length - (props.dry_CM * props.dry_mass + props.propellant_CM * current_prop_mass) /
                                        (props.dry_mass + current_prop_mass);  // From aft of rocket

    double new_inertia =
        props.dry_Inertia[0] + pow(total_CM - (props.total_length - props.propellant_CM), 2) * current_prop_mass;

    total_Inertia[0] = new_inertia;
    total_Inertia[1] = new_inertia;

    J_inv[0] = total_CM / total_Inertia[0];
    J_inv[1] = total_CM / total_Inertia[1];
    J_inv[2] = 1 / total_Inertia[2];
  }
};
