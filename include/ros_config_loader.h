#pragma once

#include <ros/package.h>
#include "ros/ros.h"
#include "rocket_model.hpp"

// TODO use yamlcpp instead?

inline RocketProps loadRocketProps(ros::NodeHandle& nh)
{
  RocketProps props;
  nh.getParam("/rocket/minTorque", props.minTorque);
  nh.getParam("/rocket/maxTorque", props.maxTorque);
  nh.getParam("/rocket/maxThrust", props.maxThrust);
  nh.getParam("/rocket/minThrust", props.minThrust);
  nh.getParam("/rocket/Isp", props.Isp);
  nh.getParam("/rocket/dry_mass", props.dry_mass);
  nh.getParam("/rocket/propellant_mass", props.initial_propellant_mass);
  nh.getParam("/rocket/Cd", props.Cd);
  nh.getParam("/rocket/dry_I", props.dry_Inertia);
  nh.getParam("/rocket/dry_CM", props.dry_CM);
  nh.getParam("/rocket/propellant_CM", props.propellant_CM);
  nh.getParam("/environment/apogee", props.target_apogee);
  nh.getParam("/rocket/initial_speed", props.initial_speed);
  nh.getParam("/rocket/diameters", props.diameter);
  nh.getParam("/rocket/stage_z", props.length);
  nh.getParam("/rocket/stages", props.nStage);
  nh.getParam("/perturbation/acc_noise", props.acc_noise);
  nh.getParam("/perturbation/acc_bias", props.acc_bias);
  nh.getParam("/perturbation/gyro_noise", props.gyro_noise);
  nh.getParam("/perturbation/gyro_bias", props.gyro_bias);
  nh.getParam("/perturbation/baro_noise", props.baro_noise);
  nh.getParam("/perturbation/baro_bias", props.baro_bias);
  nh.getParam("/environment/ground_altitude", props.h0);

  std::string path = ros::package::getPath("rocket_utils") + "/config/motor_file.txt";

  std::string line;
  std::ifstream myfile(path);
  if (myfile.is_open())
  {
    while (getline(myfile, line))
    {
      int separator = line.find("\t");
      std::string time_string = line.substr(0, separator);
      std::string thrust_string = line.substr(separator + 1, line.length() - separator);

      props.full_thrust_time.push_back(std::stof(time_string));
      props.full_thrust.push_back(std::stof(thrust_string));
    }
    myfile.close();
  }
  else
  {
    ROS_WARN("Didn't find motor file");
  }

  return props;
}