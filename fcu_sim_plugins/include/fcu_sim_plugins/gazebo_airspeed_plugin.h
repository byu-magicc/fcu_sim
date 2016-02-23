/*
 * Copyright 2015 James Jackson BYU Provo, UT
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef fcu_sim_PLUGINS_AIRSPEED_PLUGIN_H
#define fcu_sim_PLUGINS_AIRSPEED_PLUGIN_H

#include <random>

#include <Eigen/Core>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <sensor_msgs/FluidPressure.h>
#include <geometry_msgs/Vector3.h>

#include <chrono>
#include <cmath>
#include <iostream>
#include <stdio.h>

#include <boost/bind.hpp>

#include "fcu_sim_plugins/common.h"

namespace gazebo {

class GazeboAirspeedPlugin : public ModelPlugin {
 public:

  GazeboAirspeedPlugin();
  ~GazeboAirspeedPlugin();

  void InitializeParams();
  void Publish();

 protected:

  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void OnUpdate(const common::UpdateInfo&);

 private:
  std::string namespace_;
  std::string airspeed_topic_;
  ros::NodeHandle* nh_;
  ros::Publisher airspeed_pub_;
  std::string frame_id_;
  std::string link_name_;

  std::default_random_engine random_generator_;
  std::normal_distribution<double> standard_normal_distribution_;

  // Gazebo connections
  physics::WorldPtr world_;
  physics::ModelPtr model_;
  physics::LinkPtr link_;
  event::ConnectionPtr updateConnection_;

  common::Time last_time_;

  // Wind Connection
  struct Wind{ double N;  double E;  double D; } wind_;
  ros::Subscriber wind_speed_sub_;
  void WindSpeedCallback(const geometry_msgs::Vector3& wind);

  sensor_msgs::FluidPressure airspeed_message_;

  double pressure_bias_;
  double pressure_noise_sigma_;
  double max_pressure_;
  double min_pressure_;
  double rho_;
};
}

#endif // fcu_sim_PLUGINS_AIRSPEED_PLUGIN_H
