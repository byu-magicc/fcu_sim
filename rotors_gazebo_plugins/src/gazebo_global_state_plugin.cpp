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

#include "rotors_gazebo_plugins/gazebo_global_state_plugin.h"

#include <chrono>
#include <cmath>
#include <iostream>
#include <stdio.h>

#include <boost/bind.hpp>

namespace gazebo {

GazeboGlobalStatePlugin::GazeboGlobalStatePlugin()
    : ModelPlugin(),
      node_handle_(0){}

GazeboGlobalStatePlugin::~GazeboGlobalStatePlugin() {
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
  if (node_handle_) {
    node_handle_->shutdown();
    delete node_handle_;
  }
}


void GazeboGlobalStatePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Configure Gazebo Integration
  model_ = _model;
  world_ = model_->GetWorld();
  namespace_.clear();
  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_altimeter_plugin] Please specify a robotNamespace.\n";
  if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[gazebo_altimeter_plugin] Please specify a linkName.\n";
  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("[gazebo_altimeter_plugin] Couldn't find specified link \"" << link_name_ << "\".");
  // Connect to the Gazebo Update
  this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboGlobalStatePlugin::OnUpdate, this, _1));
  frame_id_ = link_name_;

  // load params from xacro
  getSdfParam<std::string>(_sdf, "globalStateTopic", global_state_topic_, "global_state");
  getSdfParam<std::string>(_sdf, "mocapTopic", mocap_topic_, "mocap");
  last_time_ = world_->GetSimTime();

  // Configure ROS Integration
  node_handle_ = new ros::NodeHandle(namespace_);
  global_state_pub_ = node_handle_->advertise<relative_nav_msgs::FilterState>(global_state_topic_, 10);
  mocap_pub_ = node_handle_->advertise<geometry_msgs::TransformStamped>(mocap_topic_, 10);

  // Fill static members of messages.
  global_state_message_.node_id = 0;
  mocap_message_.header.frame_id = "world";
  mocap_message_.child_frame_id = frame_id_;
}


void GazeboGlobalStatePlugin::OnUpdate(const common::UpdateInfo& _info)
{
  common::Time current_time  = world_->GetSimTime();
  double t = current_time.Double();

  // get global state from gazebo
  math::Pose math_NWU = link_->GetWorldPose();
  tf::Transform tf_NWU = mathtoTF(math_NWU);

  // convert NWU from Gazebo output to NED
  tf::Transform NWU_to_NED(tf::Quaternion(1,0,0,0),tf::Vector3(0,0,0));
  tf::Transform tf_NED = NWU_to_NED*tf_NWU;

  // create geometry_msgs to send
  // mocap is in NWU while global_state is in NED
  geometry_msgs::Transform msg_NED,msg_NWU;
  tf::transformTFToMsg(tf_NED,msg_NED);
  tf::transformTFToMsg(tf_NWU,msg_NWU);

  // pack up and send
  mocap_message_.transform = msg_NWU;
  mocap_message_.header.stamp = ros::Time::now();
  global_state_message_.transform = msg_NED;
  global_state_pub_.publish(global_state_message_);
  mocap_pub_.publish(mocap_message_);
}


tf::Transform GazeboGlobalStatePlugin::mathtoTF(math::Pose mathTF)
{
  tf::Transform tf_TF;
  tf_TF.setOrigin(tf::Vector3(mathTF.pos.x, mathTF.pos.y, mathTF.pos.z));
  tf_TF.setRotation(tf::Quaternion(mathTF.rot.x, mathTF.rot.y,mathTF.rot.z, mathTF.rot.w));
  return tf_TF;
}


GZ_REGISTER_MODEL_PLUGIN(GazeboGlobalStatePlugin);
}
