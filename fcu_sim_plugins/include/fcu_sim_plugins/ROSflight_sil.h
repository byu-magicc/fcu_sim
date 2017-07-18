/*
 * Copyright 2016 James Jackson, Brigham Young University, Provo UT
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
#include <yaml-cpp/yaml.h>

#include <rosflight_msgs/Command.h>
#include <rosflight_msgs/OutputRaw.h>
#include <rosflight_msgs/RCRaw.h>
#include <std_msgs/Float32MultiArray.h>
#include <rosflight_msgs/Attitude.h>
#include "fcu_sim_plugins/common.h"
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>
#include <std_srvs/Trigger.h>
#include <rosflight_utils/simple_pid.h>


namespace gazebo {
static const std::string kDefaultWindSpeedSubTopic = "gazebo/wind_speed";


class ROSflightSIL : public ModelPlugin {
public:
  ROSflightSIL();

  ~ROSflightSIL();

  void SendForces();


protected:
  void UpdateForcesAndMoments();
  void UpdateEstimator();
  void Reset();
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void OnUpdate(const common::UpdateInfo & /*_info*/);

private:
  std::string command_topic_;
  std::string rc_topic_;
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
    std::vector<double> F_poly;
    std::vector<double> T_poly;
    double tau_up; // time constants for response
    double tau_down;
  };

  struct Motor{
    Rotor rotor;
    Eigen::Vector3d position;
    Eigen::Vector3d normal;
    int direction; // 1 for CW -1 for CCW
  };

  int num_rotors_;
  std::vector<Motor> motors_;

  double linear_mu_;
  double angular_mu_;
  std::vector<double> ground_effect_;

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
  rosflight_utils::SimplePID roll_controller_;
  rosflight_utils::SimplePID pitch_controller_;
  rosflight_utils::SimplePID yaw_controller_;
  rosflight_utils::SimplePID alt_controller_;

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
  uint64_t start_time_us_;

  ros::NodeHandle* nh_;
  ros::Subscriber command_sub_;
  ros::Subscriber rc_sub_;
  ros::Subscriber wind_speed_sub_;
  ros::Subscriber imu_sub_;
  ros::Publisher estimate_pub_, euler_pub_;
  ros::Publisher signals_pub_;
  ros::Publisher alt_pub_, angle_pub_, command_pub_, passthrough_pub_;
  ros::ServiceServer calibrate_imu_srv_;

  rosflight_msgs::Command command_;

  boost::thread callback_queue_thread_;
  void WindSpeedCallback(const geometry_msgs::Vector3& wind);
  void CommandCallback(const rosflight_msgs::Command& msg);
  void RCCallback(const rosflight_msgs::OutputRaw& msg);
  void imuCallback(const sensor_msgs::Imu& msg);

  bool calibrateImuBiasSrvCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  double sat(double x, double max, double min);
  double max(double x, double y);
  math::Vector3 W_wind_speed_;

  Eigen::MatrixXd rotor_position_;
  Eigen::MatrixXd rotor_plane_normal_;
  Eigen::VectorXd rotor_rotation_direction_;

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
