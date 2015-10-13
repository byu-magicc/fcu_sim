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


#include "rotor_gazebo_plugins/gazebo_wind_plugin.h"
#include <geometry_msgs/WrenchStamped.h>
#include <chrono>

namespace gazebo {

GazeboWindPlugin::~GazeboWindPlugin() {
  event::Events::DisconnectWorldUpdateBegin(update_connection_);
  if (node_handle_) {
    node_handle_->shutdown();
    delete node_handle_;
  }
}

void GazeboWindPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  // Store the pointer to the model.
  model_ = _model;
  world_ = model_->GetWorld();

  double wind_gust_start = kDefaultWindGustStart;
  double wind_gust_duration = kDefaultWindGustDuration;

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_wind_plugin] Please specify a robotNamespace.\n";
  node_handle_ = new ros::NodeHandle(namespace_);

  if (_sdf->HasElement("xyzOffset"))
    xyz_offset_ = _sdf->GetElement("xyzOffset")->Get<math::Vector3>();
  else
    gzerr << "[gazebo_wind_plugin] Please specify a xyzOffset.\n";

  getSdfParam<std::string>(_sdf, "windPubTopic", wind_pub_topic_, "/" + namespace_ + wind_pub_topic_);
  getSdfParam<std::string>(_sdf, "frameId", frame_id_, frame_id_);
  getSdfParam<std::string>(_sdf, "linkName", link_name_, link_name_);

  // Get the wind params from SDF.
  getSdfParam<double>(_sdf, "windForceMean", wind_force_mean_, wind_force_mean_);
  getSdfParam<double>(_sdf, "windForceVariance", wind_force_variance_, wind_force_variance_);

  // Get the wind gust params from SDF.
  getSdfParam<double>(_sdf, "windGustStart", wind_gust_start, wind_gust_start);
  getSdfParam<double>(_sdf, "windGustDuration", wind_gust_duration, wind_gust_duration);
  getSdfParam<double>(_sdf, "windGustForceMean", wind_gust_force_mean_, wind_gust_force_mean_);
  getSdfParam<double>(_sdf, "windGustForceVariance", wind_gust_force_variance_, wind_gust_force_variance_);
  getSdfParam<math::Vector3>(_sdf, "windGustDirection", wind_gust_direction_, wind_gust_direction_);

  wind_gust_direction_.Normalize();
  wind_gust_start_ = common::Time(wind_gust_start);
  wind_gust_end_ = common::Time(wind_gust_start + wind_gust_duration);

  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("[gazebo_wind_plugin] Couldn't find specified link \"" << link_name_ << "\".");


  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboWindPlugin::OnUpdate, this, _1));

  wind_pub_ = node_handle_->advertise<geometry_msgs::WrenchStamped>(wind_pub_topic_, 10);
}

// This gets called by the world update start event.
void GazeboWindPlugin::OnUpdate(const common::UpdateInfo& _info) {
  // Get the current simulation time.
  common::Time now = world_->GetSimTime();

  // Establish Random Number Generator
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator (seed);
  std::uniform_real_distribution<double> direction_distribution(-1,1);
  std::normal_distribution<double> force_distribution(wind_force_mean_,sqrt(wind_force_variance_));
  std::normal_distribution<double> gust_force_distribution(wind_gust_force_mean_,sqrt(wind_gust_force_variance_));
  std::normal_distribution<double> wind_change_distribution(1000,500);

  // Calculate the wind force


  if (wind_change_delay==wind_change_value)
  {
      wind_strength = force_distribution(generator);
      wind_x = direction_distribution(generator);
      wind_y = direction_distribution(generator);
      wind_z = direction_distribution(generator);
      wind_change_value = ceil(abs(wind_change_distribution(generator)));
      wind_change_delay=0;
  }
  else
  {
      wind_change_delay++;
  }

  //Vary the wind direction

  wind_direction={wind_x, wind_y, wind_z};
  wind_direction.Normalize();

  math::Vector3 wind = wind_strength * wind_direction;
  // Apply a force from the wind to the link.
  link_->AddForceAtRelativePosition(wind, xyz_offset_);

  //Wind Gust

  math::Vector3 wind_gust(0, 0, 0);
  // Calculate the wind gust force.
  if (now >= wind_gust_start_ && now < wind_gust_end_) {
    double wind_gust_strength = gust_force_distribution(generator);
    wind_gust = wind_gust_strength * wind_gust_direction_;
    // Apply a force from the wind gust to the link.
    link_->AddForceAtRelativePosition(wind_gust, xyz_offset_);
  }

  geometry_msgs::WrenchStamped wrench_msg;

  wrench_msg.header.frame_id = frame_id_;
  wrench_msg.header.stamp.sec = now.sec;
  wrench_msg.header.stamp.nsec = now.nsec;
  wrench_msg.wrench.force.x = wind.x + wind_gust.x;
  wrench_msg.wrench.force.y = wind.y + wind_gust.y;
  wrench_msg.wrench.force.z = wind.z + wind_gust.z;
  wrench_msg.wrench.torque.x = 0;
  wrench_msg.wrench.torque.y = 0;
  wrench_msg.wrench.torque.z = 0;

  wind_pub_.publish(wrench_msg);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboWindPlugin);
}
