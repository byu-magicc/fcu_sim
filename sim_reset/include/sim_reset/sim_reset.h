#ifndef SIM_RESET_H
#define SIM_RESET_H

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <gazebo_msgs/ModelState.h>
#include <relative_nav_msgs/DesiredState.h>

namespace sim_reset
{

struct Button{
  int index;
  bool prev_value;
};

struct Buttons {
  Button reset;
};

class simReset
{

public:

  simReset();

private:
  // Node handles, publishers, subscribers
  ros::NodeHandle nh_;         //!< public node handle for subscribing, publishing, etc.
  ros::NodeHandle nh_private_; //!< private node handle for pulling parameter values from the parameter server

  // Publishers and Subscribers
  ros::Subscriber Joy_subscriber_;
  ros::Subscriber desired_state_subscriber_;
  ros::Publisher ModelState_publisher_;

  // Parameters
  bool reset_;
  std::string model_name_;

  // Local Variables
  Buttons buttons_;
  gazebo_msgs::ModelState reset_msg_;
  double alt_;

  // Functions
  void JoyCallback(const sensor_msgs::JoyConstPtr &msg);
  void desiredStateCallback(const relative_nav_msgs::DesiredStateConstPtr &msg);
};

} // namespace sim_reset

#endif // simReset_H
