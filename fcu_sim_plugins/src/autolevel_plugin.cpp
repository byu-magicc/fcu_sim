/*
 * Copyright 2015 James Jackson MAGICC Lab, BYU, Provo, UT
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

#include "fcu_sim_plugins/autolevel_plugin.h"

namespace gazebo {

AutoLevelPlugin::AutoLevelPlugin() : ModelPlugin() {}

AutoLevelPlugin::~AutoLevelPlugin() {
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
}

void AutoLevelPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Configure Gazebo Integration
  model_ = _model;
  world_ = model_->GetWorld();
  namespace_.clear();

  // Load SDF parameters
  if (_sdf->HasElement("namespace")) {
    namespace_ = _sdf->GetElement("namespace")->Get<std::string>();
  } else {
    gzerr << "[AutoLevelPlugin] Please specify a namespace";
  }

  if (_sdf->HasElement("sensorLink")) {
    sensor_link = model_->GetLink(_sdf->GetElement("sensorLink")->Get<std::string>());
  } else{
    gzerr << "[AutoLevelPlugin] Please specify a sensor link";
  }

  std::string link_name;
  if (_sdf->HasElement("modelLink")){
    model_link = model_->GetLink(_sdf->GetElement("modelLink")->Get<std::string>());
  }else{
    gzerr << "[AutoLevelPlugin] Please specify a linkName of the forces and moments plugin.\n";
  }



  // Connect Gazebo Update
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&AutoLevelPlugin::OnUpdate, this, _1));


  // Set the axes of the gimbal
//  yaw_joint_->SetAxis(0, math::Vector3(0, 0, 1));
//  pitch_joint_->SetAxis(0, math::Vector3(0, 1, 0));
//  roll_joint_->SetAxis(0, math::Vector3(1, 0, 0));

  // Initialize
  this->Reset();
}

void AutoLevelPlugin::Reset()
{

}

// Return the Sign of the argument
void AutoLevelPlugin::OnUpdate(const common::UpdateInfo & _info)
{
    math::Vector3 global_pose = model_link->GetWorldCoGPose().rot.GetAsEuler();
    math::Vector3 relative_pose = model_link->GetRelativePose().rot.GetAsEuler();

    double roll = -global_pose.x;
    double pitch =  -global_pose.y;
    double yaw = -relative_pose.z;

    sensor_link->SetRelativePose(math::Pose(0, 0, -.50, roll, pitch, yaw));

}

GZ_REGISTER_MODEL_PLUGIN(AutoLevelPlugin);

} // namespace
