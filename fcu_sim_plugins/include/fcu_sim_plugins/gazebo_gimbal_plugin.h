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

  // Pointer to the model
  private:
      physics::ModelPtr model;
          // brief Pointer to the joint.
      physics::JointPtr joint;
      physics::JointPtr ball_joint;
          // Pointer to the update event connection
      event::ConnectionPtr updateConnection;
          // Azimuth Angle
      float kp;
      float kd;
      float previous_error;
          // Elevation Angle
      float ball_kp;
      float ball_kd;
      float ball_previous_error;
          // Time
      float previous_time;
      float current_time;

      // ROS variables
      ros::NodeHandle* nh;
      ros::Subscriber command_sub;

      float angle_desired;
      float ball_angle_desired;
      int direction; 		// 0: no direction, 1: up, 2: down, 3: CW, 4: CCW
      float update_rate;
  };
} // namespace gazebo
#endif //fcu_sim_PLUGINS_GIMBAL_PLUGIN_H
