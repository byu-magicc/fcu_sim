#ifndef SIM_RELAY_H
#define SIM_RELAY_H

#include <ros/ros.h>
#include <relative_nav_msgs/Command.h>
#include <relative_nav_msgs/FilterState.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <mav_msgs/RollPitchYawrateThrust.h>

namespace sim_relay
{

class simRelay
{

public:

  simRelay();

private:

  // Node handles, publishers, subscribers
  ros::NodeHandle nh_;         //!< public node handle for subscribing, publishing, etc.
  ros::NodeHandle nh_private_; //!< private node handle for pulling parameter values from the parameter server

  // Publishers and Subscribers
  ros::Subscriber command_sub_;
  ros::Subscriber odometry_sub_;
  ros::Publisher imu_pub_;
  ros::Publisher alt_pub_;
  ros::Publisher relative_state_pub_;
  ros::Publisher mocap_pub_;
  ros::Publisher command_pub_;

  // Parameters
  bool noise_;
  double accel_stdev_;
  double gyro_bias_;
  double gyro_stdev_;
  double att_stdev_;
  double trans_stdev_;

  // Local Variables
  relative_nav_msgs::FilterState current_state_;

  // Functions
  void commandCallback(const relative_nav_msgs::CommandConstPtr &msg);
  void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
  void publishSensors();
  void publishStates();
};

} // namespace sim_relay

#endif // simRelay_H
