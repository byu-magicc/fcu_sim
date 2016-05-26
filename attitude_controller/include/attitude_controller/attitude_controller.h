#ifndef ATTITUDE_CONTROLLER_H
#define ATTITUDE_CONTROLLER_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>
#include <fcu_common/Command.h>
#include <fcu_common/simple_pid.h>
#include <tf/tf.h>
#include <attitude_controller/multicopter.h>
#include <attitude_controller/h_inf_controller.h>
#include <dynamic_reconfigure/server.h>
#include <attitude_controller/GainConfig.h>

#define PID_CONTROL 0
#define HINF_CONTROL 1

namespace attitude_controller
{

class attitudeController
{

public:

  attitudeController();

private:

  // Node handles, publishers, subscribers
  ros::NodeHandle nh_;         //!< public node handle for subscribing, publishing, etc.
  ros::NodeHandle nh_private_; //!< private node handle for pulling parameter values from the parameter server

  // Publishers and Subscribers
  ros::Subscriber odometry_subscriber_;
  ros::Subscriber command_subscriber_;
  ros::Publisher actuators_publisher_;

  // Parameters
  std::string odometry_topic_, motor_speed_command_topic_, command_topic_;

  // Controller Type
  int controller_type_;

  // Class Variables
  double dt_;
  double time_of_last_control_;

  // incoming commands
  double roll_c_;
  double pitch_c_;
  double yaw_c_;
  double phidot_c_;
  double thetadot_c_;
  double psidot_c_;
  double phiddot_c_;
  double thetaddot_c_;
  double psiddot_c_;
  double thrust_c_;

  // current attitude
  double phi_;
  double theta_;
  double psi_;
  double p_;
  double q_;
  double r_;

  // un-normalize commands
  double max_roll_;
  double max_pitch_;
  double max_yaw_rate_;
  double max_throttle_;

  // outgoing commands
  Eigen::Vector4d desired_forces_;
  Eigen::VectorXd rotor_velocities_;

  // PID controllers
  fcu_common::SimplePID pid_roll_;
  fcu_common::SimplePID pid_pitch_;
  fcu_common::SimplePID pid_yaw_rate_;

  // H-Infinity Controller
  HInfController h_inf_;

  // Multicopter Configuration
  multicopter multicopter_;

  // Functions
  Eigen::Vector4d updatePIDControl();
  Eigen::Vector4d updateHInfControl();
  void calculateRotorVelocities();
  void calculateRotorMapping();

  // Dynamic Gain Configuration
  dynamic_reconfigure::Server<attitude_controller::GainConfig> server_;
  dynamic_reconfigure::Server<attitude_controller::GainConfig>::CallbackType func_;
  void gainCallback(attitude_controller::GainConfig &config, uint32_t level);

  // Message Callbacks
  void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
  void commandCallback(const fcu_common::CommandConstPtr& msg);
};

} // namespace attitude_controller

#endif // attitudeController_H
