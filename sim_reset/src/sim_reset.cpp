#include "sim_reset/sim_reset.h"

namespace sim_reset
{

simReset::simReset() :
  nh_(ros::NodeHandle()),
  nh_private_(ros::NodeHandle("~"))
{
  // retrieve params
  nh_private_.param<int>("reset_button", buttons_.reset.index, 9);
  nh_private_.param<std::string>("model_name", model_name_, "shredder");

  // Setup publishers and subscribers
  Joy_subscriber_ = nh_.subscribe("joy", 1, &simReset::JoyCallback, this);
  desired_state_subscriber_ = nh_.subscribe("desired_state",1,&simReset::desiredStateCallback,this);
  ModelState_publisher_ = nh_.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 1);

  // fill static members of reset_msg_
  reset_msg_.model_name = model_name_;
  reset_msg_.pose.orientation.x = 0.0;
  reset_msg_.pose.orientation.y = 0.0;
  reset_msg_.pose.orientation.z = 0.0;
  reset_msg_.pose.orientation.w = 1.0;
  reset_msg_.pose.position.x = 0.0;
  reset_msg_.pose.position.y = 0.0;
  reset_msg_.pose.position.z = 0.0;
  reset_msg_.twist.angular.x = 0.0;
  reset_msg_.twist.angular.y = 0.0;
  reset_msg_.twist.angular.z = 0.0;
  reset_msg_.twist.linear.x = 0.0;
  reset_msg_.twist.linear.y = 0.0;
  reset_msg_.twist.linear.z = 0.0;
  reset_msg_.reference_frame = "world";

}

void simReset::JoyCallback(const sensor_msgs::JoyConstPtr &msg){
  if(msg->buttons[buttons_.reset.index]==0 && buttons_.reset.prev_value==1){ // button release
    ModelState_publisher_.publish(reset_msg_);
  }
  buttons_.reset.prev_value = msg->buttons[buttons_.reset.index];

}

void simReset::desiredStateCallback(const relative_nav_msgs::DesiredStateConstPtr &msg)
{
  alt_ = msg->pose.z;
  reset_msg_.pose.position.z = -1.0*alt_;

}

} // namespace sim_reset
