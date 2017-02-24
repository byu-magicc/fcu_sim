/*
 * Copyright 2016 James Jackson, Brigham Young University, Provo, UT
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


#ifndef fcu_sim_PLUGINS_MULTIROTOR_FORCES_AND_MOMENTS_H
#define fcu_sim_PLUGINS_MULTIROTOR_FORCES_AND_MOMENTS_H

#include <stdio.h>

#include <vector>
#include <boost/bind.hpp>
#include <Eigen/Eigen>

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <fcu_common/Command.h>
#include <fcu_common/simple_pid.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3.h>

#include "fcu_sim_plugins/common.h"

namespace gazebo {


class MultiRotorForcesAndMoments : public ModelPlugin {
public:
  MultiRotorForcesAndMoments();

  ~MultiRotorForcesAndMoments();

  void InitializeParams();
  void SendForces();

protected:
  void UpdateForcesAndMoments();
  void Reset();
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

  // So we can reset to the initial position
  math::Pose initial_pose_;

  // physical parameters
  double linear_mu_;
  double angular_mu_;
  struct GE_constants{
    double a;
    double b;
    double c;
    double d;
    double e;
  } ground_effect_;
  double mass_; // for static thrust offset when in altitude mode (kg)

  // Container for an Actuator
  struct Actuator{
    double max;
    double tau_up;
    double tau_down;
  };

  // Struct of Actuators
  // This organizes the physical limitations of the abstract torques and Force
  struct Actuators{
    Actuator l;
    Actuator m;
    Actuator n;
    Actuator F;
  } actuators_;

  // container for forces
  struct ForcesAndTorques{
    double Fx;
    double Fy;
    double Fz;
    double l;
    double m;
    double n;
  } applied_forces_, actual_forces_, desired_forces_;

  // container for PID controller
  fcu_common::SimplePID roll_controller_;
  fcu_common::SimplePID pitch_controller_;
  fcu_common::SimplePID yaw_controller_;
  fcu_common::SimplePID alt_controller_;

  fcu_common::Command command_;

  // Time Counters
  double sampling_time_;
  double prev_sim_time_ = 0;
  double prev_control_time_ = 0;

  ros::NodeHandle* nh_;
  ros::Subscriber command_sub_;
  ros::Subscriber wind_speed_sub_;
  ros::Publisher debug_;

  boost::thread callback_queue_thread_;
  void QueueThread();
  void WindSpeedCallback(const geometry_msgs::Vector3& wind);
  void CommandCallback(const fcu_common::Command msg);
  void ComputeControl(void);
  double sat(double x, double max, double min);
  double max(double x, double y);

  std::unique_ptr<FirstOrderFilter<double>>  rotor_velocity_filter_;
  math::Vector3 W_wind_speed_;
};
}

#endif // fcu_sim_PLUGINS_MULTIROTOR_FORCES_AND_MOMENTS_H
