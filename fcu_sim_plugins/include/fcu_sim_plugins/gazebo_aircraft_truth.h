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


#ifndef fcu_sim_PLUGINS_AIRCRAFT_TRUTH_H
#define fcu_sim_PLUGINS_AIRCRAFT_TRUTH_H

#include <stdio.h>

#include <boost/bind.hpp>
#include <Eigen/Eigen>

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <fcu_common/Command.h>
#include <fcu_common/FW_State.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3.h>

#include "fcu_sim_plugins/common.h"

namespace gazebo {
static const std::string kDefaultWindSpeedSubTopic = "gazebo/wind_speed";


class GazeboAircraftTruth : public ModelPlugin {
 public:
  GazeboAircraftTruth();

  ~GazeboAircraftTruth();

  void InitializeParams();  

 protected:
  void PublishTruth();
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void OnUpdate(const common::UpdateInfo & /*_info*/);

 private:
  std::string truth_topic_;
  std::string wind_speed_topic_;
  std::string joint_name_;
  std::string link_name_;
  std::string parent_frame_id_;
  std::string motor_speed_pub_topic_;
  std::string namespace_;

  physics::WorldPtr world_;
  physics::ModelPtr model_;
  physics::LinkPtr link_;
  physics::JointPtr joint_;
  physics::EntityPtr parent_link_;
  event::ConnectionPtr updateConnection_; // Pointer to the update event connection.

  // wind
  struct Wind{
    double N;
    double E;
    double D;
  } wind_;

  // Time Counters
  double sampling_time_;
  double prev_sim_time_;

  ros::NodeHandle* node_handle_;
  ros::Subscriber wind_speed_sub_;
  ros::Publisher true_state_pub_;

  boost::thread callback_queue_thread_;
  void QueueThread();
  void WindSpeedCallback(const geometry_msgs::Vector3& wind);

  std::unique_ptr<FirstOrderFilter<double>>  rotor_velocity_filter_;
  math::Vector3 wind_speed_W_;
};
}

#endif // fcu_sim_PLUGINS_AIRCRAFT_TRUTH_H
