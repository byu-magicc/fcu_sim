#include "sim_relay/sim_relay.h"

namespace sim_relay
{

simRelay::simRelay() :
  nh_(ros::NodeHandle()),
  nh_private_(ros::NodeHandle("~"))
{
  // retrieve params
  nh_private_.param<bool>("sensor_noise", noise_, true);
  nh_private_.param<double>("accel_stdev" , accel_stdev_, 0.0025);
  nh_private_.param<double>("gyro_stdev" , gyro_stdev_, 0.002269);
  nh_private_.param<double>("gyro_bias" , gyro_bias_, 0.0000);
  nh_private_.param<double>("attitude_stdev", att_stdev_, 0.0043633);
  nh_private_.param<double>("translation_stdev", trans_stdev_, 0.015);


  // Setup publishers and subscribers
  command_sub_ = nh_.subscribe("desired_state", 1, &simRelay::commandCallback, this);
  odometry_sub_ = nh_.subscribe("odometry", 1, &simRelay::odometryCallback, this);

  imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu/data", 1);
  alt_pub_ = nh_.advertise<sensor_msgs::Range>("altimter", 1);
  relative_state_pub_ = nh_.advertise<relative_nav_msgs::FilterState>("relative_state", 1);
  mocap_pub_= nh_.advertise<geometry_msgs::TransformStamped>("mocap", 1);
  command_pub_ = nh_.advertise<mav_msgs::RollPitchYawrateThrust>("command/roll_pitch_yawrate_thrust",1);
}

void simRelay::commandCallback(const relative_nav_msgs::CommandConstPtr &msg){
  mav_msgs::RollPitchYawrateThrust out_msg;
  out_msg.roll = msg->roll;
  out_msg.pitch = msg->pitch;
  out_msg.yaw_rate = msg->yaw_rate;
  out_msg.thrust.x = 0.0;
  out_msg.thrust.y = 0.0;
  out_msg.thrust.z = msg->thrust;
  out_msg.header.stamp = ros::Time::now();
  command_pub_.publish(out_msg);
}

void simRelay::odometryCallback(const nav_msgs::OdometryConstPtr &msg){
  current_state_.transform.translation.x = msg->pose.pose.position.x;
  current_state_.transform.translation.y = msg->pose.pose.position.y;
  current_state_.transform.translation.z = msg->pose.pose.position.z;
  current_state_.transform.rotation = msg->pose.pose.orientation;
  current_state_.velocity.x = msg->twist.twist.linear.x;
  current_state_.velocity.y = msg->twist.twist.linear.y;
  current_state_.velocity.z = msg->twist.twist.linear.z;
  // currently, this does not output relative nodes.  This can be done later
}

void simRelay::publishSensors(){}

void simRelay::publishStates(){}

} // namespace sim_relay
