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

  pnh.param("axis_roll_", axes_.roll, 2);
  pnh.param("axis_pitch_", axes_.pitch, 3);
  pnh.param("axis_thrust_", axes_.thrust, 1);
  pnh.param("axis_yaw_", axes_.yaw, 0);

  pnh.param("axis_direction_roll", axes_.roll_direction, -1);
  pnh.param("axis_direction_pitch", axes_.pitch_direction, 1);
  pnh.param("axis_direction_thrust", axes_.thrust_direction, 1);
  pnh.param("axis_direction_yaw", axes_.yaw_direction, -1);

  pnh.param("max_v_xy", max_.v_xy, 1.0);  // [m/s]
  pnh.param("max_roll", max_.roll, 25.0 * M_PI / 180.0);  // [rad]
  pnh.param("max_pitch", max_.pitch, 25.0 * M_PI / 180.0);  // [rad]
  pnh.param("max_yaw_rate", max_.rate_yaw, 180.0 * M_PI / 180.0);  // [rad/s]
  pnh.param("max_thrust", max_.thrust, 74.676);  // [N]

  pnh.param("button_takeoff_", buttons_.fly.index, 0);

  command_pub_ = nh_.advertise<relative_nav_msgs::Command>(command_topic_,10);
  ctrl_pub_ = nh_.advertise<rotor_gazebo::RollPitchYawrateThrust> ("command/roll_pitch_yawrate_thrust", 10);

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
}

void Joy::JoyCallback(const sensor_msgs::JoyConstPtr& msg) {
  if(fly_mav_){
    current_joy_ = *msg;
    control_msg_.roll = msg->axes[axes_.roll] * max_.roll * axes_.roll_direction;
    control_msg_.pitch = msg->axes[axes_.pitch] * max_.pitch * axes_.pitch_direction;
    control_msg_.yaw_rate = msg->axes[axes_.yaw] * max_.rate_yaw * axes_.yaw_direction;
    control_msg_.thrust.z = (msg->axes[axes_.thrust] + 1) * max_.thrust / 2.0 * axes_.thrust_direction;

    command_msg_.roll = control_msg_.roll;
    command_msg_.pitch = control_msg_.pitch;
    command_msg_.yaw_rate = control_msg_.yaw_rate;
    command_msg_.thrust = control_msg_.thrust.z;

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
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "rotor_sim_joy");
  Joy joy;

  ros::spin();

  return 0;
}
