#ifndef ATTITUDE_CONTROLLER_H
#define ATTITUDE_CONTROLLER_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <rotor_gazebo/Actuators.h>
#include <relative_nav_msgs/Command.h>
#include <relative_nav_common/simple_pid.h>
#include <tf/tf.h>
#include <attitude_controller/multicopter.h>

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

  // Class Variables
  double dt_;
  double time_of_last_control_;

  // incoming commands
  double roll_c_;
  double pitch_c_;
  double yaw_rate_c_;
  double thrust_c_;

  // current attitude
  double phi_;
  double theta_;
  double r_;

  // outgoing commands
  Eigen::Vector4d desired_forces_;
  Eigen::VectorXd rotor_velocities_;

  relative_nav_common::SimplePID pid_roll_;
  relative_nav_common::SimplePID pid_pitch_;
  relative_nav_common::SimplePID pid_yaw_rate_;
  multicopter multicopter_;
  Eigen::MatrixX4d forces_to_omegas_mapping_;

  // Functions
  void updatePIDLoops();
  void calculateRotorVelocities();
  void calculateRotorMapping();

  void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
  void commandCallback(const relative_nav_msgs::CommandConstPtr& msg);
};

} // namespace attitude_controller

#endif // attitudeController_H
