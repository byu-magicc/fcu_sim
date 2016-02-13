/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
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


#ifndef fcu_sim_PLUGINS_AIRCRAFT_FORCES_AND_MOMENTS_H
#define fcu_sim_PLUGINS_AIRCRAFT_FORCES_AND_MOMENTS_H

#include <stdio.h>

#include <boost/bind.hpp>
#include <Eigen/Eigen>

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <fcu_common/Command.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3.h>

#include "fcu_sim_plugins/common.h"

namespace gazebo {
static const std::string kDefaultWindSpeedSubTopic = "gazebo/wind_speed";


class GazeboAircraftForcesAndMoments : public ModelPlugin {
 public:
  GazeboAircraftForcesAndMoments();

  ~GazeboAircraftForcesAndMoments();

  void InitializeParams();
  void SendForces();
  

 protected:
  void UpdateForcesAndMoments();
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void OnUpdate(const common::UpdateInfo & /*_info*/);

 private:
  std::string command_topic_;
  std::string wind_speed_topic_;
  std::string joint_name_;
  std::string link_name_;
  std::string parent_frame_id_;
  std::string motor_speed_pub_topic_;
  std::string namespace_;

  physics::WorldPtr world_;
  physics::ModelPtr model_;
  physics::LinkPtr link_;
  physics::JointPtr joint_;
  physics::EntityPtr parent_link_;
  event::ConnectionPtr updateConnection_; // Pointer to the update event connection.

  // physical parameters
  double mass_;
  double Jx_;
  double Jy_;
  double Jz_;
  double Jxz_;
  double rho_;

  // aerodynamic coefficients
  struct WingCoeff{
    double S;
    double b;
    double c;
    double M;
    double epsilon;
    double alpha0;
  } wing_;

  // Propeller Coefficients
  struct PropCoeff{
    double k_motor;
    double k_T_P;
    double k_Omega;
    double e;
    double S;
    double C;
  } prop_;

  // Lift Coefficients
  struct LiftCoeff{
    double O;
    double alpha;
    double beta;
    double p;
    double q;
    double r;
    double delta_a;
    double delta_e;
    double delta_r;
  };

  LiftCoeff CL_;
  LiftCoeff CD_;
  LiftCoeff Cm_;
  LiftCoeff CY_;
  LiftCoeff Cell_;
  LiftCoeff Cn_;

  // not constants
  // actuators
  struct Actuators{
    double e;
    double a;
    double r;
    double t;
  } delta_;

    // wind
  struct Wind{
    double N;
    double E;
    double D;
  } wind_;

  // container for forces
  struct ForcesAndTorques{
    double Fx;
    double Fy;
    double Fz;
    double l;
    double m;
    double n;
  } forces_;

  // Time Counters
  double sampling_time_;
  double prev_sim_time_;

  ros::NodeHandle* node_handle_;
  ros::Subscriber command_sub_;
  ros::Subscriber wind_speed_sub_;

  boost::thread callback_queue_thread_;
  void QueueThread();
  void WindSpeedCallback(const geometry_msgs::Vector3& wind);
  void CommandCallback(const fcu_common::CommandConstPtr& msg);

  std::unique_ptr<FirstOrderFilter<double>>  rotor_velocity_filter_;
  math::Vector3 wind_speed_W_;
};
}

#endif // fcu_sim_PLUGINS_AIRCRAFT_FORCES_AND_MOMENTS_H
