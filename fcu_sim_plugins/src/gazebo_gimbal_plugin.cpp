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

#include "fcu_sim_plugins/gazebo_gimbal_plugin.h"

namespace gazebo {

GazeboGimbalPlugin::GazeboGimbalPlugin() : ModelPlugin() {}

GazeboGimbalPlugin::~GazeboGimbalPlugin() {
    event::Events::DisconnectWorldUpdateBegin(updateConnection_);
    if (nh_) {
      nh_->shutdown();
      delete nh_;
    }
}

void GazeboGimbalPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Configure Gazebo Integration
  model_ = _model;
  world_ = model_->GetWorld();
  namespace_.clear();

  // Load SDF parameters
  if (_sdf->HasElement("namespace")) {
    namespace_ = _sdf->GetElement("namespace")->Get<std::string>();
  } else {
    gzerr << "[gazeboGimbalPlugin] Please specify a namespace";
  }

  if (_sdf->HasElement("yawJoint")) {
    std::string yaw_joint_name = _sdf->GetElement("yawJoint")->Get<std::string>();
    yaw_joint_ = model_->GetLink(yaw_joint_name);
  } else{
    gzerr << gzerr << "[gazeboGimbalPlugin] Please specify a yawJoint";
  }

  if (_sdf->HasElement("pitchJoint")) {
    std::string pitch_joint_name = _sdf->GetElement("pitchJoint")->Get<std::string>();
    pitch_joint_ = model_->GetLink(pitch_joint_name);
  } else{
    gzerr << gzerr << "[gazeboGimbalPlugin] Please specify a pitchJoint";
  }

  if (_sdf->HasElement("rollJoint")) {
    std::string roll_joint_name = _sdf->GetElement("rollJoint")->Get<std::string>();
    roll_joint_ = model_->GetLink(roll_joint_name);
  } else{
    gzerr << gzerr << "[gazeboGimbalPlugin] Please specify a rollJoint";
  }

  if (_sdf->HasElement("rollJoint")) {
    std::string roll_joint_name = _sdf->GetElement("rollJoint")->Get<std::string>();
    time_constant_ = model_->GetLink(roll_joint_name);
  } else{
    gzerr << gzerr << "[gazeboGimbalPlugin] Please specify a rollJoint";
  }









  nh_ = new ros::NodeHandle();
  command_sub_ = nh_->subscribe("cmd_vel", 1, &GazeboGimbalPlugin::commandCallback, this);

  _model->GetName() << "]\n";
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
                              boost::bind(&GazeboGimbalPlugin::OnUpdate, this, _1));

  // Store the model pointer for convenience.
  this->model_ = _model;

  // Get the first joint. We are making an assumption about the model
  // having one joint that is the rotational joint.
  this->yaw_joint_ = _model->GetJoints()[0];
  this->pitch_joint_ = _model->GetJoints()[1];
  // this->ball_joint->SetForce(0,0.1);
  // cout <<	"Initial angle: " << this->joint->GetAngle(0).Degree() << endl;
  math::Angle angle(0);
  math::Angle ball_angle(0);
  this->yaw_joint_->SetAngle(0,angle);
  this->pitch_joint_->SetAngle(0,ball_angle);
  // cout <<	this->joint->GetAngle(0).Degree() << endl;
  // Azimuth Angle
  yaw_desired_ = 0;
  yaw_kp_ = 0.001;
  yaw_kd_ = 0.0001;
  // Elevation Angle
  pitch_desired_ = 90;
  pitch_kp_ = 0.001;
  pitch_kd_ = 0.0001;
  // Initialize Error
  previous_time_ = clock()/(double) CLOCKS_PER_SEC;
  yaw_prev_error_ = 0;

  direction_ = -1;
  update_rate_ = 0.5;
}

void GazeboGimbalPlugin::OnUpdate(const common::UpdateInfo & /*_info*/)
{
    // Update time
    current_time_ = clock()/(double) CLOCKS_PER_SEC;
    float time_step = current_time_ - previous_time_;

      // Azimuth Controller
    float angle_current = this->yaw_joint_->GetAngle(0).Degree();
    if(direction_ == 3){
        yaw_desired_ = angle_current + update_rate_;
        if(yaw_desired_ > 360){
            yaw_desired_ = 360;
        }
    }
    else if(direction_ == 4){
        yaw_desired_ = angle_current - update_rate_;
        if(yaw_desired_ < -360){
            yaw_desired_ = -360;
        }
    }
    float error = yaw_desired_ - angle_current;
    float error_derivative = (error-yaw_prev_error_)/time_step;
    float force_desired = yaw_kp_*error + yaw_kd_*error_derivative;
    yaw_joint_->SetForce(0,force_desired);
      // cout << "Azimuth angle: " << angle_current << endl;

      // Elevation Controller
    float ball_angle_current = this->pitch_joint_->GetAngle(0).Degree();
    if(direction_ == 1){
        pitch_desired_ = ball_angle_current + update_rate_;
        if(pitch_desired_ > 180){
            pitch_desired_ = 180;
        }
    }
    else if(direction_ == 2){
        pitch_desired_ = ball_angle_current - update_rate_;
        if(pitch_desired_ < 0){
            pitch_desired_ = 0;
        }
    }
    float ball_error = pitch_desired_ - ball_angle_current;
    float ball_error_derivative = (ball_error-pitch_previous_error_)/time_step;
    float ball_force_desired = pitch_kp_*ball_error + pitch_kd_*ball_error_derivative;
    pitch_joint_->SetForce(0,ball_force_desired);
      // cout << "Elevation angle: " <<  ball_angle_current << endl;

      // Update error and time
    yaw_prev_error_ = error;
    pitch_previous_error_ = ball_error;
    previous_time_ = current_time_;

    //	cout << "I am updating" << endl;
    //	joint->SetVelocity(0,1000);
    //	joint->SetForce(0,0.0001);
    //	cout << joint->GetVelocity(0) << endl;
}

void GazeboGimbalPlugin::commandCallback(const geometry_msgs::Twist::ConstPtr& msg){

     // cout << "desired azimuth: " << msg->linear.x << endl;
     // cout << "desired elevation: " << msg->angular.z << endl;
   if(msg->linear.x==0.5){
       direction_ = 1;
   }
   else if(msg->linear.x==-0.5){
       direction_ = 2;
   }

   if(msg->angular.z==0.5){
       direction_ = 3;
   }
   else if(msg->angular.z==-0.5){
       direction_ = 4;
   }

   if(msg->linear.x==0 && msg->angular.z==0){
       direction_ = 0;
   }

}

// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
GZ_REGISTER_MODEL_PLUGIN(GazeboGimbalPlugin);

} // namespace
