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


#ifndef fcu_sim_PLUGINS_MOTOR_MODELS_H
#define fcu_sim_PLUGINS_MOTOR_MODELS_H

#include <stdio.h>

#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Vector3.h>

#include "fcu_sim_plugins/common.h"
#include "fcu_sim_plugins/motor_model.hpp"

namespace turning_direction {
const static int CCW = 1;
const static int CW = -1;
}

namespace gazebo {

class GazeboMotorModel : public MotorModel, public ModelPlugin {
 public:
  GazeboMotorModel() : ModelPlugin(), MotorModel(){}

  virtual ~GazeboMotorModel();

  virtual void InitializeParams();
  virtual void Publish();

 protected:
  virtual void UpdateForcesAndMoments();
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void OnUpdate(const common::UpdateInfo & /*_info*/);

 private:
  std::string command_sub_topic_;
  std::string wind_speed_sub_topic_;
  std::string joint_name_;
  std::string link_name_;
  std::string motor_speed_pub_topic_;
  std::string namespace_;

  int motor_number_;
  int turning_direction_;

  double max_force_;
  double max_rot_velocity_;
  double moment_constant_;
  double motor_constant_;
  double ref_motor_rot_vel_;
  double rolling_moment_coefficient_;
  double rotor_drag_coefficient_;
  double rotor_velocity_slowdown_sim_;
  double time_constant_down_;
  double time_constant_up_;

  ros::NodeHandle* node_handle_;
  ros::Publisher motor_velocity_pub_;
  ros::Subscriber command_sub_;
  ros::Subscriber wind_speed_sub_;

  physics::ModelPtr model_;
  physics::JointPtr joint_;
  physics::LinkPtr link_;
  /// \brief Pointer to the update event connection.
  event::ConnectionPtr updateConnection_;

  boost::thread callback_queue_thread_;
  void QueueThread();
  std_msgs::Float32 turning_velocity_msg_;
  void VelocityCallback(const std_msgs::Float64MultiArrayConstPtr& rot_velocities);
  void WindSpeedCallback(const geometry_msgs::Vector3ConstPtr& wind_speed);

  std::unique_ptr<FirstOrderFilter<double>>  rotor_velocity_filter_;
  math::Vector3 wind_speed_W_;
};
}

#endif // fcu_sim_PLUGINS_MOTOR_MODELS_H
