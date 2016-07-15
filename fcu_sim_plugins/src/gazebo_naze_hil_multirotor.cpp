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

#include "fcu_sim_plugins/gazebo_naze_hil_multirotor.h"
#include <sstream>

namespace gazebo
{

GazeboMultirotorHIL::GazeboMultirotorHIL() :
  ModelPlugin(), node_handle_(nullptr), prev_sim_time_(0)  {}


GazeboMultirotorHIL::~GazeboMultirotorHIL()
{
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
  if (node_handle_) {
    node_handle_->shutdown();
    delete node_handle_;
  }
}


void GazeboMultirotorHIL::SendForces()
{
  // apply the forces and torques to the joint
  link_->AddRelativeForce(math::Vector3(forces_.Fx, -forces_.Fy, forces_.Fz));
  link_->AddRelativeTorque(math::Vector3(forces_.l, -forces_.m, -forces_.n));
}


void GazeboMultirotorHIL::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  model_ = _model;
  world_ = model_->GetWorld();

  namespace_.clear();

  /*
   * Connect the Plugin to the Robot and Save pointers to the various elements in the simulation
   */
  if (_sdf->HasElement("namespace"))
    namespace_ = _sdf->GetElement("namespace")->Get<std::string>();
  else
    gzerr << "[gazebo_multirotor_hil] Please specify a namespace.\n";
  node_handle_ = new ros::NodeHandle(namespace_);

 if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[gazebo_multirotor_hil] Please specify a linkName of the forces and moments plugin.\n";
  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("[gazebo_multirotor_hil] Couldn't find specified link \"" << link_name_ << "\".");

  /* Load Params from Gazebo Server */
  getSdfParam<std::string>(_sdf, "windSpeedTopic", wind_speed_topic_, "wind");
  getSdfParam<std::string>(_sdf, "commandTopic", command_topic_, "command");

  /* Load Rotor Configuration */
  getSdfParam<int>(_sdf, "numRotors", num_rotors_, 4);
  motors_.resize(num_rotors_);

  // For now, just assume all rotors are the same
  Rotor rotor;
  getSdfParam<double>(_sdf, "rotorMaxThrust", rotor.max, 14.961);
  getSdfParam<double>(_sdf, "rotorF1", rotor.F1, -1e-05f);
  getSdfParam<double>(_sdf, "rotorF2", rotor.F2, 0.0452);
  getSdfParam<double>(_sdf, "rotorF3", rotor.F3, -35.117);
  getSdfParam<double>(_sdf, "rotorT1", rotor.T1, -2e-08f);
  getSdfParam<double>(_sdf, "rotorT2", rotor.T2, 8e-05);
  getSdfParam<double>(_sdf, "rotorT3", rotor.T3, -0.0586);
  getSdfParam<double>(_sdf, "rotorTauUp", rotor.tau_up, 0.1644);
  getSdfParam<double>(_sdf, "rotorTauDown", rotor.tau_down, 0.2164);

  force_allocation_matrix_.resize(4,num_rotors_);
  torque_allocation_matrix_.resize(4,num_rotors_);
  for(int i = 0; i<num_rotors_; i++)
  {
    std::stringstream int_strm;
    int_strm << i+1;
    getSdfParam<double>(_sdf, "rotor" + int_strm.str() + "Distance", motors_[i].distance, 0);
    getSdfParam<double>(_sdf, "rotor" + int_strm.str() + "Angle",    motors_[i].angle, 0);
    getSdfParam<int>(_sdf, "rotor" + int_strm.str() + "Direction", motors_[i].direction, 0);
    motors_[i].rotor = rotor;

    // build allocation_matrices
    force_allocation_matrix_(0,i) = -1.0*sin(motors_[i].angle)*motors_[i].distance; // l
    force_allocation_matrix_(1,i) = cos(motors_[i].angle)*motors_[i].distance; // m
    force_allocation_matrix_(2,i) = 0.0; // n
    force_allocation_matrix_(3,i) = 1.0; // F

    torque_allocation_matrix_(0,i) = 0.0; // l
    torque_allocation_matrix_(1,i) = 0.0; // m
    torque_allocation_matrix_(2,i) = motors_[i].direction; //n
    torque_allocation_matrix_(3,i) = 0.0; // F
    gzmsg << "allocation matrices \n" << force_allocation_matrix_ << "\n\n" << torque_allocation_matrix_ << "\n";
  }

  getSdfParam<double> (_sdf, "mu", mu_, 1);

  // Initialize size of dynamic force and torque matrices
  desired_forces_.resize(num_rotors_);
  desired_torques_.resize(num_rotors_);
  actual_forces_.resize(num_rotors_);
  actual_torques_.resize(num_rotors_);
  motor_signals_.resize(num_rotors_);
  for (int i = 0; i < num_rotors_; i++)
  {
    desired_forces_(i)=0.0;
    desired_torques_(i)=0.0;
    actual_forces_(i)=0.0;
    actual_torques_(i)=0.0;
    motor_signals_(i)=1000;
  }

  // Connect the update function to the simulation
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboMultirotorHIL::OnUpdate, this, _1));

  // Connect Subscribers
  command_sub_ = node_handle_->subscribe(command_topic_, 1, &GazeboMultirotorHIL::CommandCallback, this);
  wind_speed_sub_ = node_handle_->subscribe(wind_speed_topic_, 1, &GazeboMultirotorHIL::WindSpeedCallback, this);
}

// This gets called by the world update event.
void GazeboMultirotorHIL::OnUpdate(const common::UpdateInfo& _info) {

  sampling_time_ = _info.simTime.Double() - prev_sim_time_;
  prev_sim_time_ = _info.simTime.Double();
  UpdateForcesAndMoments();
  SendForces();
}

void GazeboMultirotorHIL::WindSpeedCallback(const geometry_msgs::Vector3 &wind){
  W_wind_speed_.x = wind.x;
  W_wind_speed_.y = wind.y;
  W_wind_speed_.z = wind.z;
}

void GazeboMultirotorHIL::CommandCallback(const fcu_common::ServoOutputRaw &msg)
{
  for(int i = 0; i< num_rotors_; i++)
  {
    motor_signals_(i) = msg.values[i];
  }
}


void GazeboMultirotorHIL::UpdateForcesAndMoments()
{
  /* Get state information from Gazebo                          *
   * C denotes child frame, P parent frame, and W world frame.  *
   * Further C_pose_W_P denotes pose of P wrt. W expressed in C.*/
  math::Pose W_pose_W_C = link_->GetWorldCoGPose();
  double pn = W_pose_W_C.pos.x; // We should check to make sure that this is right
  double pe = -W_pose_W_C.pos.y;
  double pd = -W_pose_W_C.pos.z;
  math::Vector3 euler_angles = W_pose_W_C.rot.GetAsEuler();
  double phi = euler_angles.x;
  double theta = -euler_angles.y;
  double psi = -euler_angles.z;
  math::Vector3 C_linear_velocity_W_C = link_->GetRelativeLinearVel();
  double u = C_linear_velocity_W_C.x;
  double v = -C_linear_velocity_W_C.y;
  double w = -C_linear_velocity_W_C.z;
  math::Vector3 C_angular_velocity_W_C = link_->GetRelativeAngularVel();
  double p = C_angular_velocity_W_C.x;
  double q = -C_angular_velocity_W_C.y;
  double r = -C_angular_velocity_W_C.z;

  // wind info is available in the wind_ struct
  // Rotate into body frame and relative velocity
  math::Vector3 C_wind_speed = W_pose_W_C.rot.RotateVector(W_wind_speed_);
  double ur = u - C_wind_speed.x;
  double vr = v - C_wind_speed.y;
  double wr = w - C_wind_speed.z;

  // Calculate Forces
  for (int i = 0; i<num_rotors_; i++)
  {
    // First, figure out the desired force output from passing the signal into the quadratic approximation
    double signal = motor_signals_(i);
    desired_forces_(i,0) = motors_[i].rotor.F1*signal*signal + motors_[i].rotor.F2*signal + motors_[i].rotor.F3;
    desired_torques_(i,0) = motors_[i].rotor.T1*signal*signal + motors_[i].rotor.T2*signal + motors_[i].rotor.T3;

    // Then, Calculate Actual force and torque for each rotor using first-order dynamics
    double tau = (desired_forces_(i,0) > actual_forces_(i,0)) ? motors_[i].rotor.tau_up : motors_[i].rotor.tau_down;
    double alpha = sampling_time_/(tau + sampling_time_);
    actual_forces_(i,0) = sat((1-alpha)*actual_forces_(i) + alpha*desired_forces_(i), motors_[i].rotor.max, 0.0);
    actual_torques_(i,0) = sat((1-alpha)*actual_torques_(i) + alpha*desired_torques_(i), motors_[i].rotor.max, 0.0);
  }
//  gzerr << "\nsignals = \n" << motor_signals_;
//  gzmsg << "\ndesired_forces = \n" << desired_forces_ << "\ndesired torques = " << desired_torques_ << "\n";
//  gzmsg << "\nactual_forces = \n" << actual_forces_ << "\nactual_torques = " << actual_torques_ << "\n";

  Eigen::Vector4d output_forces_and_torques = force_allocation_matrix_*actual_forces_ + torque_allocation_matrix_*actual_torques_;

//  gzmsg << "\noutput_forces and torques\n" << output_forces_and_torques;

  forces_.Fx = -mu_*ur;
  forces_.Fy = -mu_*vr;
  forces_.Fz = -mu_*wr + output_forces_and_torques(3);
  forces_.l = output_forces_and_torques(0);
  forces_.m = output_forces_and_torques(1);
  forces_.n = output_forces_and_torques(2);
}

double GazeboMultirotorHIL::sat(double x, double max, double min)
{
  if(x > max)
    return max;
  else if(x < min)
    return min;
  else
    return x;
}


GZ_REGISTER_MODEL_PLUGIN(GazeboMultirotorHIL);
}
