#ifndef cu_sim_PLUGINS_GIMBAL_PLUGIN_H
#define fcu_sim_PLUGINS_GIMBAL_PLUGIN_H


//#include <gazebo/math/gzmath.hh>
//#include <ctime>
//#include <stdio.h>

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"

#include <chrono>
#include <cmath>
#include <iostream>
#include <stdio.h>

#include <boost/bind.hpp>

#define PI 3.141592

namespace gazebo {


class GazeboGimbalPlugin : public ModelPlugin {
public:
  GazeboGimbalPlugin();
  ~GazeboGimbalPlugin();
  void commandCallback(const geometry_msgs::Twist::ConstPtr& msg);

protected:

  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void OnUpdate(const common::UpdateInfo & /*_info*/);


private:
  // ROS variables
  ros::NodeHandle* nh_;
  ros::Subscriber command_sub_;

  // Pointer to the gazebo items.
  physics::JointPtr yaw_joint_;
  physics::JointPtr roll_joint_;
  physics::JointPtr pitch_joint_;
  physics::ModelPtr model_;
  physics::WorldPtr world_;

  std::string namespace_;


  // Pointer to the update event connection
  event::ConnectionPtr updateConnection_;

  // Azimuth Angle
  double yaw_kp_;
  double yaw_kd_;
  double yaw_prev_error_;

  // Elevation Angle
  double pitch_kp_;
  double pitch_kd_;
  double pitch_previous_error_;

  double roll_kp_;
  double roll_kd_;
  double roll_previous_error_;

  // Time
  double previous_time_;
  double current_time_;

  // Commands
  double yaw_desired_;
  double pitch_desired_;
  double roll_desired_;

  double time_constant_;

  int direction_; 		// 0: no direction, 1: up, 2: down, 3: CW, 4: CCW
  double update_rate_;
};
} // namespace gazebo
#endif //fcu_sim_PLUGINS_GIMBAL_PLUGIN_H
