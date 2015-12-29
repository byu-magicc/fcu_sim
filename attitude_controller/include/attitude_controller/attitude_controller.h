#ifndef ATTITUDE_CONTROLLER_H
#define ATTITUDE_CONTROLLER_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <rotor_gazebo/Actuators.h>
#include <relative_nav/Command.h>
#include <relative_nav/simple_pid.h>
#include <tf/tf.h>
#include <attitude_controller/multicopter.h>
#include <attitude_controller/h_inf_controller.h>

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

  // outgoing commands
  Eigen::Vector4d desired_forces_;
  Eigen::VectorXd rotor_velocities_;

  // PID controllers
  relative_nav::SimplePID pid_roll_;
  relative_nav::SimplePID pid_pitch_;
  relative_nav::SimplePID pid_yaw_rate_;

  // H-Infinity Controller
  HInfController h_inf_;

  // Multicopter Configuration
  multicopter multicopter_;

  // Functions
  Eigen::Vector4d updatePIDControl();
  Eigen::Vector4d updateHInfControl();
  void calculateRotorVelocities();
  void calculateRotorMapping();

  // Message Callbacks
  void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
  void commandCallback(const relative_nav::CommandConstPtr& msg);
};

} // namespace attitude_controller

#endif // attitudeController_H
