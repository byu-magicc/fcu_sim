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

#include "fcu_sim_plugins/gazebo_altimeter_plugin.h"

namespace gazebo {

GazeboAltimeterPlugin::GazeboAltimeterPlugin()
    : ModelPlugin(),
      node_handle_(0){}

GazeboAltimeterPlugin::~GazeboAltimeterPlugin() {
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
  if (node_handle_) {
    node_handle_->shutdown();
    delete node_handle_;
  }
}


void GazeboAltimeterPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
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
  this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboAltimeterPlugin::OnUpdate, this, _1));
  frame_id_ = link_name_;

  // load params from xacro
  getSdfParam<std::string>(_sdf, "altimeterTopic", alt_topic_, "altimeter");
  getSdfParam<double>(_sdf, "altimeterMinRange", min_range_, 0.381);
  getSdfParam<double>(_sdf, "altimeterMaxRange", max_range_, 6.45);
  getSdfParam<double>(_sdf, "altimeterErrorStdev", error_stdev_, 0.10);
  getSdfParam<double>(_sdf, "altimeterFOV", field_of_view_, 1.107);
  getSdfParam<double>(_sdf, "altimeterPublishRate", pub_rate_, 20.0);
  getSdfParam<bool>(_sdf, "altimeterNoiseOn", alt_noise_on_, true);
  getSdfParam<bool>(_sdf, "publishFloat32", publish_float_, false);
  last_time_ = world_->GetSimTime();

  // Configure ROS Integration
  node_handle_ = new ros::NodeHandle(namespace_);
  if(publish_float_)
      alt_pub_ = node_handle_->advertise<std_msgs::Float32>(alt_topic_, 10);
  else
      alt_pub_ = node_handle_->advertise<sensor_msgs::Range>(alt_topic_, 10);

  // Fill static members of alt message.
  alt_message_.header.frame_id = frame_id_;
  alt_message_.max_range = max_range_;
  alt_message_.min_range = min_range_;
  alt_message_.field_of_view = field_of_view_;
  alt_message_.radiation_type = sensor_msgs::Range::ULTRASOUND;

  // Configure Noise
  random_generator_= std::default_random_engine(std::chrono::system_clock::now().time_since_epoch().count());
  standard_normal_distribution_ = std::normal_distribution<double>(0.0, error_stdev_);
}


void GazeboAltimeterPlugin::OnUpdate(const common::UpdateInfo& _info)
{
  // check if time to publish
  common::Time current_time  = world_->GetSimTime();
  if((current_time - last_time_).Double() > 1.0/pub_rate_){

    // pull z measurement out of Gazebo
    math::Pose current_state_LFU = link_->GetWorldPose();
    alt_message_.range = current_state_LFU.pos.z;

    // zero measurement when messages are out of range
    if(alt_message_.range > max_range_ || alt_message_.range < min_range_){
      alt_message_.range = 0.0;
    }

    // add noise, if requested
    if(alt_noise_on_){
      alt_message_.range += standard_normal_distribution_(random_generator_);
    }

    // publish message
    alt_message_.header.stamp = ros::Time::now();
    if(publish_float_)
    {
        std_msgs::Float32 msg;
        msg.data = alt_message_.range;
        alt_pub_.publish(msg);
    }
    else
        alt_pub_.publish(alt_message_);
    last_time_ = current_time;
  }
}

GZ_REGISTER_MODEL_PLUGIN(GazeboAltimeterPlugin);
}
