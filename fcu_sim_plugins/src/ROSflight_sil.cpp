/*
   * Copyright  2016 James Jackson, Brigham Young University, Provo UT
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

#pragma GCC diagnostic ignored "-Wwrite-strings"

#include "fcu_sim_plugins/ROSflight_sil.h"
#include <sstream>
#include <stdint.h>

#include <stdio.h>


// includde the ROSflight headers
extern "C"
{
#include "ROSflight_SIL.h"
#include "board.h"
#include "rosflight.h"
#include "estimator.h"
#include "mixer.h"
#include "mode.h"
#include "controller.h"
#include "sensors.h"
}


namespace gazebo
{

ROSflightSIL::ROSflightSIL() :
  ModelPlugin(), nh_(nullptr), prev_sim_time_(0)  {
}


ROSflightSIL::~ROSflightSIL()
{
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
  if (nh_) {
    nh_->shutdown();
    delete nh_;
  }
}

void ROSflightSIL::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
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
    gzerr << "[ROSflight_SIL] Please specify a namespace.\n";
  nh_ = new ros::NodeHandle(namespace_);

  if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[ROSflight_SIL] Please specify a linkName of the forces and moments plugin.\n";
  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("[ROSflight_SIL] Couldn't find specified link \"" << link_name_ << "\".");

  /* Load Params from Gazebo Server */
  getSdfParam<std::string>(_sdf, "windSpeedTopic", wind_speed_topic_, "wind");
  getSdfParam<std::string>(_sdf, "commandTopic", command_topic_, "command");
  getSdfParam<std::string>(_sdf, "rcTopic", rc_topic_, "rc");
  getSdfParam<std::string>(_sdf, "imuTopic", imu_topic_, "imu/data");
  getSdfParam<std::string>(_sdf, "estimateTopic", estimate_topic_, "attitude");
  getSdfParam<std::string>(_sdf, "signalsTopic", signals_topic_, "motor_signals");

  gzmsg << "loading parameters from " << namespace_ << " ns\n";

  // Pull Parameters off of rosparam server
  num_rotors_ = 0;
  ROS_ASSERT(nh_->getParam("ground_effect", ground_effect_));
  ROS_ASSERT(nh_->getParam("mass", mass_));
  ROS_ASSERT(nh_->getParam("linear_mu", linear_mu_));
  ROS_ASSERT(nh_->getParam("angular_mu", angular_mu_));
  ROS_ASSERT(nh_->getParam("num_rotors", num_rotors_));

  std::vector<double> rotor_positions(3 * num_rotors_);
  std::vector<double> rotor_vector_normal(3 * num_rotors_);
  std::vector<int> rotor_rotation_directions(num_rotors_);

  // For now, just assume all rotors are the same
  Rotor rotor;

  ROS_ASSERT(nh_->getParam("rotor_positions", rotor_positions));
  ROS_ASSERT(nh_->getParam("rotor_vector_normal", rotor_vector_normal));
  ROS_ASSERT(nh_->getParam("rotor_rotation_directions", rotor_rotation_directions));
  ROS_ASSERT(nh_->getParam("rotor_max_thrust", rotor.max));
  ROS_ASSERT(nh_->getParam("rotor_F", rotor.F_poly));
  ROS_ASSERT(nh_->getParam("rotor_T", rotor.T_poly));
  ROS_ASSERT(nh_->getParam("rotor_tau_up", rotor.tau_up));
  ROS_ASSERT(nh_->getParam("rotor_tau_down", rotor.tau_down));

  /* Load Rotor Configuration */
  motors_.resize(num_rotors_);

  force_allocation_matrix_.resize(4,num_rotors_);
  torque_allocation_matrix_.resize(4,num_rotors_);
  for(int i = 0; i < num_rotors_; i++)
  {
    motors_[i].rotor = rotor;
    motors_[i].position.resize(3);
    motors_[i].normal.resize(3);
    for (int j = 0; j < 3; j++)
    {
      motors_[i].position(j) = rotor_positions[3*i + j];
      motors_[i].normal(j) = rotor_vector_normal[3*i + j];
    }
    motors_[i].normal.normalize();
    motors_[i].direction = rotor_rotation_directions[i];

    Eigen::Vector3d moment_from_thrust = motors_[i].position.cross(motors_[i].normal);
    Eigen::Vector3d moment_from_torque = motors_[i].direction * motors_[i].normal;

    // build allocation_matrices
    force_allocation_matrix_(0,i) = moment_from_thrust(0); // l
    force_allocation_matrix_(1,i) = moment_from_thrust(1); // m
    force_allocation_matrix_(2,i) = moment_from_thrust(2); // n
    force_allocation_matrix_(3,i) = motors_[i].normal(2); // F

    torque_allocation_matrix_(0,i) = moment_from_torque(0); // l
    torque_allocation_matrix_(1,i) = moment_from_torque(1); // m
    torque_allocation_matrix_(2,i) = moment_from_torque(2); // n
    torque_allocation_matrix_(3,i) = 0.0; // F
  }

  gzmsg << "allocation matrices:\nFORCE \n" << force_allocation_matrix_ << "\nTORQUE\n" << torque_allocation_matrix_ << "\n";

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
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&ROSflightSIL::OnUpdate, this, _1));

  // Connect Subscribers
  command_sub_ = nh_->subscribe(command_topic_, 1, &ROSflightSIL::CommandCallback, this);
  rc_sub_ = nh_->subscribe(rc_topic_, 1, &ROSflightSIL::RCCallback, this);
  wind_speed_sub_ = nh_->subscribe(wind_speed_topic_, 1, &ROSflightSIL::WindSpeedCallback, this);
  imu_sub_ = nh_->subscribe(imu_topic_, 1, &ROSflightSIL::imuCallback, this);

  // Connect Publishers
  estimate_pub_ = nh_->advertise<rosflight_msgs::Attitude>(estimate_topic_, 1);
  euler_pub_ = nh_->advertise<geometry_msgs::Vector3Stamped>(estimate_topic_ + "/euler", 1);
  signals_pub_ = nh_->advertise<rosflight_msgs::OutputRaw>(signals_topic_, 1);
  command_pub_ = nh_->advertise<rosflight_msgs::Command>("output/command", 1);

  // Connect Services
  calibrate_imu_srv_ = nh_->advertiseService("calibrate_imu_bias", &ROSflightSIL::calibrateImuBiasSrvCallback, this);

  // Initialize ROSflight code
  start_time_us_ = (uint64_t)(world_->GetSimTime().Double() * 1e3);
  gzmsg << "initializing rosflight\n";
  rosflight_init();
  gzmsg << "initialized rosflight\n";
}


// This gets called by the world update event.
void ROSflightSIL::OnUpdate(const common::UpdateInfo& _info)
{
  sampling_time_ = _info.simTime.Double() - prev_sim_time_;
  prev_sim_time_ = _info.simTime.Double();
  UpdateForcesAndMoments();
  SendForces();
}

void ROSflightSIL::WindSpeedCallback(const geometry_msgs::Vector3 &wind)
{
  W_wind_speed_.x = wind.x;
  W_wind_speed_.y = wind.y;
  W_wind_speed_.z = wind.z;
}

void ROSflightSIL::Reset()
{
  start_time_us_ = (uint64_t)(world_->GetSimTime().Double() * 1e3);
  rosflight_init();
}

void ROSflightSIL::RCCallback(const rosflight_msgs::OutputRaw &msg)
{
  for (int i = 0; i < 8; i++)
  {
    _rc_signals[i] = msg.values[i];
  }
}

void ROSflightSIL::SendForces()
{
  // apply the forces and torques to the joint
  link_->AddRelativeForce(math::Vector3(forces_.Fx, -forces_.Fy, -forces_.Fz));
  link_->AddRelativeTorque(math::Vector3(forces_.l, -forces_.m, -forces_.n));
}


void ROSflightSIL::CommandCallback(const rosflight_msgs::Command &msg)
{
  // For now, just arm whenever we get our first command message
  _armed_state = ARMED;

  // Also, notice that we are manually specifying _combined_control
  /// TODO: populate _offboard_control, and use mux_inputs to combine RC and offboard
  _combined_control.F.active = true;
  _combined_control.x.active = true;
  _combined_control.y.active = true;
  _combined_control.z.active = true;

  if (msg.mode == rosflight_msgs::Command::MODE_PASS_THROUGH)
  {
    _combined_control.x.type = PASSTHROUGH;
    _combined_control.y.type = PASSTHROUGH;
    _combined_control.z.type = PASSTHROUGH;
    _combined_control.F.type = PASSTHROUGH;
    _combined_control.x.value = msg.x;
    _combined_control.y.value = msg.y;
    _combined_control.z.value = msg.z;
  }
  else if (msg.mode == rosflight_msgs::Command::MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE)
  {
    _combined_control.x.type = RATE;
    _combined_control.y.type = RATE;
    _combined_control.z.type = RATE;
    _combined_control.F.type = THROTTLE;
    _combined_control.x.value = msg.x;
    _combined_control.y.value = msg.y;
    _combined_control.z.value = msg.z;

  }
  else if (msg.mode == rosflight_msgs::Command::MODE_ROLL_PITCH_YAWRATE_THROTTLE)
  {
    _combined_control.x.type = ANGLE;
    _combined_control.y.type = ANGLE;
    _combined_control.z.type = RATE;
    _combined_control.F.type = THROTTLE;
    _combined_control.x.value = msg.x;
    _combined_control.y.value = msg.y;
    _combined_control.z.value = msg.z;
  }
  else if (msg.mode == rosflight_msgs::Command::MODE_ROLL_PITCH_YAWRATE_ALTITUDE)
  {
    _combined_control.x.type = ANGLE;
    _combined_control.y.type = ANGLE;
    _combined_control.z.type = RATE;
    _combined_control.F.type = ALTITUDE;
  }
  _combined_control.F.value = msg.F;
}

void ROSflightSIL::imuCallback(const sensor_msgs::Imu &msg)
{
  // Update the micros Timer in ROSflight
  SIL_now_us = (uint64_t)(msg.header.stamp.toNSec()*1e-3) - start_time_us_;

  // Load IMU measurements into the read_raw variables to simulate I2C communication
  // Make sure to put measurements in the NWU (the way the IMU is actually mounted)
  accel_read_raw[0] = msg.linear_acceleration.x;
  accel_read_raw[1] = msg.linear_acceleration.y;
  accel_read_raw[2] = msg.linear_acceleration.z;

  gyro_read_raw[0] = msg.angular_velocity.x;
  gyro_read_raw[1] = msg.angular_velocity.y;
  gyro_read_raw[2] = msg.angular_velocity.z;

  temp_read_raw = 25.0;

  // Simulate a read on the IMU
  SIL_call_IMU_ISR();

  // Run the main rosflight loop
  rosflight_run();

  // publish estimate
  rosflight_msgs::Attitude attitude_msg;
  geometry_msgs::Vector3Stamped euler_msg;
  attitude_msg.header.stamp = msg.header.stamp;
  attitude_msg.attitude.w = _current_state.q.w;
  attitude_msg.attitude.x = _current_state.q.x;
  attitude_msg.attitude.y = _current_state.q.y;
  attitude_msg.attitude.z = _current_state.q.z;

  attitude_msg.angular_velocity.x = _current_state.omega.x;
  attitude_msg.angular_velocity.y = _current_state.omega.y;
  attitude_msg.angular_velocity.z = _current_state.omega.z;

  euler_msg.header.stamp = msg.header.stamp;
  euler_msg.vector.x = _current_state.roll;
  euler_msg.vector.y = _current_state.pitch;
  euler_msg.vector.z = _current_state.yaw;

  estimate_pub_.publish(attitude_msg);
  euler_pub_.publish(euler_msg);


  // Run Controller
  rosflight_msgs::Command rate_msg, pt_msg;

  pt_msg.x = _command.x;
  pt_msg.y = _command.y;
  pt_msg.z = _command.z;
  pt_msg.F = _command.F;
  command_pub_.publish(rate_msg);


  rosflight_msgs::OutputRaw ESC_signals;
  ESC_signals.header.stamp.fromSec(world_->GetSimTime().Double());
  for (int i = 0; i < 8 ; i++)
  {
    // Put signal into message for debug
    ESC_signals.values[i] = _outputs[i];

    // Put outputs into vector to calculate forces and torques
    if( i < num_rotors_)
      motor_signals_(i) = _outputs[i];
  }
  signals_pub_.publish(ESC_signals);
}


void ROSflightSIL::UpdateForcesAndMoments()
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
    desired_forces_(i,0) = motors_[i].rotor.F_poly[0]*signal*signal + motors_[i].rotor.F_poly[1]*signal + motors_[i].rotor.F_poly[2];
    desired_torques_(i,0) = motors_[i].rotor.T_poly[0]*signal*signal + motors_[i].rotor.T_poly[1]*signal + motors_[i].rotor.T_poly[2];

    // Then, Calculate Actual force and torque for each rotor using first-order dynamics
    double tau = (desired_forces_(i,0) > actual_forces_(i,0)) ? motors_[i].rotor.tau_up : motors_[i].rotor.tau_down;
    double alpha = sampling_time_/(tau + sampling_time_);
    actual_forces_(i,0) = sat((1-alpha)*actual_forces_(i) + alpha*desired_forces_(i), motors_[i].rotor.max, 0.0);
    actual_torques_(i,0) = sat((1-alpha)*actual_torques_(i) + alpha*desired_torques_(i), motors_[i].rotor.max, 0.0);
  }

  // Use the allocation matrix to calculate the body-fixed force and torques
  Eigen::Vector4d output_forces = force_allocation_matrix_*actual_forces_;
  Eigen::Vector4d output_torques = torque_allocation_matrix_*actual_torques_;
  Eigen::Vector4d output_forces_and_torques = output_forces + output_torques;

  // Calculate Ground Effect
  double z = -pd;
  double ground_effect = max(ground_effect_[0]*z*z*z*z + ground_effect_[1]*z*z*z + ground_effect_[2]*z*z + ground_effect_[3]*z + ground_effect_[4], 0);

  //  // Apply other forces (wind) <- follows "Quadrotors and Accelerometers - State Estimation With an Improved Dynamic Model"
  //  // By Rob Leishman et al.
  forces_.Fx = -linear_mu_*ur;
  forces_.Fy = -linear_mu_*vr;
  forces_.Fz = -linear_mu_*wr - ground_effect + output_forces_and_torques(3);
  forces_.l = -angular_mu_*p + output_forces_and_torques(0);
  forces_.m = -angular_mu_*q + output_forces_and_torques(1);
  forces_.n = -angular_mu_*r + output_forces_and_torques(2);
}


bool ROSflightSIL::calibrateImuBiasSrvCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  return start_imu_calibration();
}

double ROSflightSIL::sat(double x, double max, double min)
{
  if(x > max)
    return max;
  else if(x < min)
    return min;
  else
    return x;
}

double ROSflightSIL::max(double x, double y)
{
  return (x > y) ? x : y;
}


GZ_REGISTER_MODEL_PLUGIN(ROSflightSIL);
}



