/*
 * Copyright 2016 Robert Pottorff PCC Lab - BYU - Provo, UT
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

#include "fcu_sim_plugins/world_utilities.h"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/Model.hh"
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

namespace gazebo {

WorldUtilities::WorldUtilities() : WorldPlugin() {}

WorldUtilities::~WorldUtilities() {
  if (nh_) {
    nh_->shutdown();
    delete nh_;
  }
}

void WorldUtilities::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
{
  // Connect ROS
  nh_ = new ros::NodeHandle("~");
  step_command_sub_ = nh_->subscribe("step", 1, &WorldUtilities::stepCommandCallback, this);
  load_models_command_sub_ = nh_->subscribe("load_models", 1, &WorldUtilities::loadModelsCommandCallback, this);

  this->world_ = _parent;
}

void WorldUtilities::loadModelsCommandCallback(const std_msgs::String &msg)
{
//    //   Load the world file
//    sdf::SDFPtr sdf(new sdf::SDF);
//    if (!sdf::init(sdf))
//    {
//        gzerr << "Unable to initialize sdf\n";
//        return;
//    }
//
//    // Find the file.
//    std::string fullFile = gazebo::common::find_file(msg.data);
//
//    if (fullFile.empty())
//    {
//        gzerr << "Unable to find file[" << msg.data << "]\n";
//        return;
//    }
//
//    if (!sdf::readFile(fullFile, sdf))
//    {
//        gzerr << "Unable to read sdf file[" << msg.data  << "]\n";
//        return;
//    }

    this->world_->SetPaused(true);

    for (auto const &m : this->world_->GetModels()){
        if(m->GetName().find("obstacle_") == 0){
            m->SetWorldPose(math::Pose(0,0,0,0,0,0));
            gzerr << m->GetWorldPose() << "\n";
        }
    }
    gzerr << "Message recieved" << "\n";



//    sdf::ElementPtr childElem = sdf->Root()->GetElement("model");
//
//    this->world_->GetByName(this->world_->GetName());
//
//    while (childElem)
//    {
//        sdf::SDF r = sdf::SDF();
//        r.Root(childElem);
//        this->world_->InsertModelString(r.ToString());
//        childElem = childElem->GetNextElement("model");
//    }

}

void WorldUtilities::stepCommandCallback(const std_msgs::Int16 &msg)
{
    this->world_->SetPaused(true);

    # if GAZEBO_MAJOR_VERSION >= 3
        this->world_->Step(msg.data);
    # else
        this->world_->StepWorld(msg.data);
    # endif
}

GZ_REGISTER_WORLD_PLUGIN(WorldUtilities)

} // namespace

