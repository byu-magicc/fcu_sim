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


#ifndef fcu_sim_PLUGINS_NAZE_HIL_MULTIROTOR_H
#define fcu_sim_PLUGINS_NAZE_HIL_MULTIROTOR_H

#include <stdio.h>

#include <boost/bind.hpp>
#include <Eigen/Eigen>

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <fcu_common/ServoOutputRaw.h>
#include <std_msgs/Float32MultiArray.h>
#include "fcu_sim_plugins/common.h"
#include <geometry_msgs/Vector3.h>

namespace gazebo {
static const std::string kDefaultWindSpeedSubTopic = "gazebo/wind_speed";


class GazeboMultirotorHIL : public ModelPlugin {
 public:
  GazeboMultirotorHIL();

  ~GazeboMultirotorHIL();

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

  struct Rotor{
    double max;
    double F1;  // constants in quadratic fit for force
    double F2;
    double F3;
    double T1;  // constants in quadratic fit for torque
    double T2;
    double T3;
    double tau_up; // time constants for response
    double tau_down;
  };

  struct Motor{
    Rotor rotor;
    double angle;  // angle from forward (-PI <-> PI)
    double distance; // distance from center to center of motor
    int direction; // 1 for CW -1 for CCW
  };

  int num_rotors_;
  std::vector<Motor> motors_;

  double mu_;

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
  void CommandCallback(const fcu_common::ServoOutputRaw& msg);
  double sat(double x, double max, double min);
  math::Vector3 W_wind_speed_;

  Eigen::MatrixXd force_allocation_matrix_;
  Eigen::MatrixXd torque_allocation_matrix_;
  Eigen::VectorXd motor_signals_;
  Eigen::VectorXd desired_forces_;
  Eigen::VectorXd desired_torques_;
  Eigen::VectorXd actual_forces_;
  Eigen::VectorXd actual_torques_;
};
}

#endif // fcu_sim_PLUGINS_NAZE_HIL_MULTIROTOR_H
