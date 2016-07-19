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

#include <fcu_common/ExtendedCommand.h>
#include <fcu_common/ServoOutputRaw.h>
#include <std_msgs/Float32MultiArray.h>
#include <fcu_common/Attitude.h>
#include "fcu_sim_plugins/common.h"
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
#include <fcu_common/simple_pid.h>

#include "param.h"

namespace gazebo {
static const std::string kDefaultWindSpeedSubTopic = "gazebo/wind_speed";


class GazeboROSflightSIL : public ModelPlugin {
public:
  GazeboROSflightSIL();

  ~GazeboROSflightSIL();

  void InitializeParams();
  void SendForces();


protected:
  void UpdateForcesAndMoments();
  void UpdateEstimator();
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void OnUpdate(const common::UpdateInfo & /*_info*/);

private:
  std::string command_topic_;
  std::string wind_speed_topic_;
  std::string imu_topic_;
  std::string estimate_topic_;
  std::string joint_name_;
  std::string link_name_;
  std::string parent_frame_id_;
  std::string signals_topic_;
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

  double linear_mu_;
  double angular_mu_;
  struct GE_constants{
    double a;
    double b;
    double c;
    double d;
    double e;
  } ground_effect_;

  double mass_;

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

  // container for PID controller
  fcu_common::SimplePID roll_controller_;
  fcu_common::SimplePID pitch_controller_;
  fcu_common::SimplePID yaw_controller_;
  fcu_common::SimplePID alt_controller_;

  // container for forces
  struct ForcesAndTorques{
    double Fx;
    double Fy;
    double Fz;
    double l;
    double m;
    double n;
  } forces_, applied_forces_;

  // Time Counters
  double sampling_time_;
  double prev_sim_time_;

  ros::NodeHandle* node_handle_;
  ros::Subscriber command_sub_;
  ros::Subscriber wind_speed_sub_;
  ros::Subscriber imu_sub_;
  ros::Publisher estimate_pub_;
  ros::Publisher signals_pub_;
  ros::Publisher alt_pub_, angle_pub_, rate_pub_, passthrough_pub_;

  fcu_common::ExtendedCommand command_;

  boost::thread callback_queue_thread_;
  void QueueThread();
  void WindSpeedCallback(const geometry_msgs::Vector3& wind);
  void CommandCallback(const fcu_common::ExtendedCommand& msg);
  void imuCallback(const sensor_msgs::Imu& msg);
  double sat(double x, double max, double min);
  double max(double x, double y);
  void initialize_params();
  void init_param_int(param_id_t id, char name[PARAMS_NAME_LENGTH], int32_t value);
  void init_param_float(param_id_t id, char name[PARAMS_NAME_LENGTH], float value);
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
