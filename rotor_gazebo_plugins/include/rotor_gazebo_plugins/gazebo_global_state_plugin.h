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

#ifndef rotor_gazebo_PLUGINS_RANGE_PLUGIN_H
#define rotor_gazebo_PLUGINS_RANGE_PLUGIN_H

#include <random>

#include <Eigen/Core>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <rotor_gazebo/default_topics.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include "rotor_gazebo_plugins/common.h"
#include <relative_nav_msgs/FilterState.h>
#include <tf/tf.h>

namespace gazebo {

class GazeboGlobalStatePlugin : public ModelPlugin {
 public:

  GazeboGlobalStatePlugin();
  ~GazeboGlobalStatePlugin();

 protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void OnUpdate(const common::UpdateInfo&);
  tf::Transform mathtoTF(math::Pose mathTF);

 private:
  // Ros Stuff
  std::string namespace_;
  ros::NodeHandle* node_handle_;
  ros::Publisher global_state_pub_;
  ros::Publisher mocap_pub_;

  // Topic
  std::string global_state_topic_;
  std::string mocap_topic_;

  // Gazebo Information
  std::string frame_id_;
  std::string link_name_;
  physics::WorldPtr world_;
  physics::ModelPtr model_;
  physics::LinkPtr link_;
  event::ConnectionPtr updateConnection_;
  common::Time last_time_;

  // Memory
  relative_nav_msgs::FilterState global_state_message_;
  geometry_msgs::TransformStamped mocap_message_;
};
}

#endif // rotor_gazebo_PLUGINS_RANGE_PLUGIN_H
