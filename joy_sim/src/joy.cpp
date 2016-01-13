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


#include "joy_sim/joy.h"

// #include <rotor_gazebo/default_topics.h>


Joy::Joy() {
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  pnh.param<std::string>("command_topic", command_topic_, "command");

  pnh.param<int>("x_axis", axes_.roll, 2);
  pnh.param<int>("y_axis", axes_.pitch, 3);
  pnh.param<int>("axis_thrust_", axes_.thrust, 1);
  pnh.param<int>("yaw_axis", axes_.yaw, 0);

  pnh.param<int>("x_sign", axes_.roll_direction, -1);
  pnh.param<int>("y_sign", axes_.pitch_direction, -1);
  pnh.param<int>("axis_direction_thrust", axes_.thrust_direction, 1);
  pnh.param<int>("yaw_sign", axes_.yaw_direction, -1);

  pnh.param<double>("max_roll", max_.roll, 25.0 * M_PI / 180.0);  // [rad]
  pnh.param<double>("max_pitch", max_.pitch, 25.0 * M_PI / 180.0);  // [rad]
  pnh.param<double>("max_yaw_rate", max_.rate_yaw, 180.0 * M_PI / 180.0);  // [rad/s]
  pnh.param<double>("max_thrust", max_.thrust, 74.676);  // [N]
  pnh.param<double>("mass", mass_, 3.81);  // [N]

  pnh.param<double>("max_aileron", max_.aileron, 15.0*M_PI/180.0);
  pnh.param<double>("max_elevator", max_.elevator, 25.0*M_PI/180.0);
  pnh.param<double>("max_rudder", max_.rudder, 15.0*M_PI/180.0);

  pnh.param<int>("button_takeoff_", buttons_.fly.index, 0);

  command_pub_ = nh_.advertise<relative_nav::Command>(command_topic_,10);
  ctrl_pub_ = nh_.advertise<rotor_gazebo::RollPitchYawrateThrust> ("command/roll_pitch_yawrate_thrust", 10);
  fw_pub_ = nh_.advertise<rotor_gazebo::FWCommand>("command/FWCommand", 10);

  ROS_ERROR_STREAM("mass = " << mass_ <<" max_thrust = " << max_.thrust);

  command_msg_.roll = 0;
  command_msg_.pitch = 0;
  command_msg_.yaw_rate = 0;
  command_msg_.thrust = 0;

  control_msg_.roll = 0;
  control_msg_.pitch = 0;
  control_msg_.yaw_rate = 0;
  control_msg_.thrust.x = 0;
  control_msg_.thrust.y = 0;
  control_msg_.thrust.z = 0;
  current_yaw_vel_ = 0;

  fw_msg_.thrust = 0;
  fw_msg_.delta_a = 0;
  fw_msg_.delta_e = 0;
  fw_msg_.delta_r = 0;

  namespace_ = nh_.getNamespace();
  joy_sub_ = nh_.subscribe("joy", 10, &Joy::JoyCallback, this);
  fly_mav_ = false;
}

void Joy::StopMav() {
  control_msg_.roll = 0;
  control_msg_.pitch = 0;
  control_msg_.yaw_rate = 0;
  control_msg_.thrust.x = 0;
  control_msg_.thrust.y = 0;
  control_msg_.thrust.z = 0;

  command_msg_.roll = 0;
  command_msg_.pitch = 0;
  command_msg_.yaw_rate = 0;
  command_msg_.thrust = 0;

  fw_msg_.thrust = 0;
  fw_msg_.delta_a = 0;
  fw_msg_.delta_e = 0;
  fw_msg_.delta_r = 0;
}

void Joy::JoyCallback(const sensor_msgs::JoyConstPtr& msg) {
  if(fly_mav_){
    current_joy_ = *msg;
    control_msg_.roll = msg->axes[axes_.roll] * max_.roll * axes_.roll_direction;
    control_msg_.pitch = msg->axes[axes_.pitch] * max_.pitch * axes_.pitch_direction;
    control_msg_.yaw_rate = msg->axes[axes_.yaw] * max_.rate_yaw * axes_.yaw_direction;
    if(msg->axes[axes_.thrust]*axes_.thrust_direction < 0){
      // some fraction of the mass
      control_msg_.thrust.z = (msg->axes[axes_.thrust]+1)*mass_*9.81; 
    }else{
      // some fraction of remaining thrust
      control_msg_.thrust.z = msg->axes[axes_.thrust]*(max_.thrust-mass_*9.81)+mass_*9.81; 
    }

    command_msg_.roll = control_msg_.roll;
    command_msg_.pitch = control_msg_.pitch;
    command_msg_.yaw_rate = control_msg_.yaw_rate;
    command_msg_.thrust = control_msg_.thrust.z;

    fw_msg_.thrust = 0.5*(msg->axes[axes_.thrust] + 1.0);
    fw_msg_.delta_a = -1.0*msg->axes[axes_.roll] * max_.aileron * axes_.pitch_direction;
    fw_msg_.delta_e = -1.0*msg->axes[axes_.pitch] * max_.elevator * axes_.pitch_direction;
    fw_msg_.delta_r = msg->axes[axes_.yaw] * max_.rudder * axes_.yaw_direction;

  } else{
    StopMav();
  }
  if(msg->buttons[buttons_.fly.index]==0 && buttons_.fly.prev_value==1){ // button release
    fly_mav_ = !fly_mav_;
  }
  buttons_.fly.prev_value = msg->buttons[buttons_.fly.index];

  ros::Time update_time = ros::Time::now();
  control_msg_.header.stamp = update_time;
  control_msg_.header.frame_id = "rotors_joy_frame";
  Publish();
}

void Joy::Publish() {
  ctrl_pub_.publish(control_msg_);
  command_pub_.publish(command_msg_);
  fw_pub_.publish(fw_msg_);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "rotor_sim_joy");
  Joy joy;

  ros::spin();

  return 0;
}
