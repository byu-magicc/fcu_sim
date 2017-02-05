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

#include "sensors.h"
#include "estimator.h"
#include "param.h"
#include "mode.h"
#include "mixer.h"
#include "mux.h"
#include "controller.h"


namespace gazebo
{

ROSflightSIL::ROSflightSIL() :
  ModelPlugin(), node_handle_(nullptr), prev_sim_time_(0)  {
  printf("STARTING");
}


ROSflightSIL::~ROSflightSIL()
{
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
  if (node_handle_) {
    node_handle_->shutdown();
    delete node_handle_;
  }
}


void ROSflightSIL::SendForces()
{
  // apply the forces and torques to the joint
  link_->AddRelativeForce(math::Vector3(forces_.Fx, -forces_.Fy, forces_.Fz));
  link_->AddRelativeTorque(math::Vector3(forces_.l, -forces_.m, -forces_.n));
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
    gzerr << "[gazebo_multirotor_hil] Please specify a namespace.\n";
  node_handle_ = new ros::NodeHandle(namespace_);

  if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[gazebo_multirotor_hil] Please specify a linkName of the forces and moments plugin.\n";
  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("[gazebo_multirotor_hil] Couldn't find specified link \"" << link_name_ << "\".");

  //  getSdfParam<double>(_sdf, "mass", mass_, 3.856);
  getSdfParam<double> (_sdf, "linear_mu", linear_mu_, 0.8);
  getSdfParam<double> (_sdf, "angular_mu", angular_mu_, 0.5);

  /* Ground Effect Coefficients */
  getSdfParam<double>(_sdf, "ground_effect_a", ground_effect_.a, -55.3516);
  getSdfParam<double>(_sdf, "ground_effect_b", ground_effect_.b, 181.8265);
  getSdfParam<double>(_sdf, "ground_effect_c", ground_effect_.c, -203.9874);
  getSdfParam<double>(_sdf, "ground_effect_d", ground_effect_.d, 85.3735);
  getSdfParam<double>(_sdf, "ground_effect_e", ground_effect_.e, -7.6619);

  /* Load Params from Gazebo Server */
  getSdfParam<std::string>(_sdf, "windSpeedTopic", wind_speed_topic_, "wind");
  getSdfParam<std::string>(_sdf, "commandTopic", command_topic_, "command");
  getSdfParam<std::string>(_sdf, "imuTopic", imu_topic_, "mikey/imu/data");
  getSdfParam<std::string>(_sdf, "estimateTopic", estimate_topic_, "attitude");
  getSdfParam<std::string>(_sdf, "signalsTopic", signals_topic_, "motor_signals");

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
  for(int i = 0; i < num_rotors_; i++)
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
  command_sub_ = node_handle_->subscribe(command_topic_, 1, &ROSflightSIL::CommandCallback, this);
  wind_speed_sub_ = node_handle_->subscribe(wind_speed_topic_, 1, &ROSflightSIL::WindSpeedCallback, this);
  imu_sub_ = node_handle_->subscribe(imu_topic_, 1, &ROSflightSIL::imuCallback, this);

  // Connect Publishers
  estimate_pub_ = node_handle_->advertise<fcu_common::Attitude>(estimate_topic_, 1);
  signals_pub_ = node_handle_->advertise<fcu_common::OutputRaw>(signals_topic_, 1);
  alt_pub_ = node_handle_->advertise<fcu_common::Command>("altitude_command", 1);
  rate_pub_ = node_handle_->advertise<fcu_common::Command>("rate_command", 1);
  angle_pub_ = node_handle_->advertise<fcu_common::Command>("angle_command", 1);
  passthrough_pub_ = node_handle_->advertise<fcu_common::Command>("passthrough_command", 1);

  // Initialize ROSflight code
  init_params();
  init_mode();
  init_estimator(false, false, true);
  init_mixing();
}


// This gets called by the world update event.
void ROSflightSIL::OnUpdate(const common::UpdateInfo& _info) {

  sampling_time_ = _info.simTime.Double() - prev_sim_time_;
  prev_sim_time_ = _info.simTime.Double();
  UpdateForcesAndMoments();
  SendForces();
}

void ROSflightSIL::WindSpeedCallback(const geometry_msgs::Vector3 &wind){
  W_wind_speed_.x = wind.x;
  W_wind_speed_.y = wind.y;
  W_wind_speed_.z = wind.z;
}

void ROSflightSIL::CommandCallback(const fcu_common::Command &msg)
{
  _combined_control.F.active = true;
  _combined_control.x.active = true;
  _combined_control.y.active = true;
  _combined_control.z.active = true;

  if (msg.mode == fcu_common::Command::MODE_PASS_THROUGH)
  {
    _combined_control.x.type = PASSTHROUGH;
    _combined_control.y.type = PASSTHROUGH;
    _combined_control.z.type = PASSTHROUGH;
    _combined_control.F.type = PASSTHROUGH;
    _combined_control.x.value = msg.x;
    _combined_control.y.value = msg.y;
    _combined_control.z.value = msg.z;
  }
  else if (msg.mode == fcu_common::Command::MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE)
  {
    _combined_control.x.type = RATE;
    _combined_control.y.type = RATE;
    _combined_control.z.type = RATE;
    _combined_control.F.type = THROTTLE;
    _combined_control.x.value = msg.x;
    _combined_control.y.value = msg.y;
    _combined_control.z.value = msg.z;

  }
  else if (msg.mode == fcu_common::Command::MODE_ROLL_PITCH_YAWRATE_THROTTLE)
  {
    _combined_control.x.type = ANGLE;
    _combined_control.y.type = ANGLE;
    _combined_control.z.type = RATE;
    _combined_control.F.type = THROTTLE;
    _combined_control.x.value = msg.x;
    _combined_control.y.value = msg.y;
    _combined_control.z.value = msg.z;
  }
  else if (msg.mode == fcu_common::Command::MODE_ROLL_PITCH_YAWRATE_ALTITUDE)
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
  uint32_t now = (uint32_t)(msg.header.stamp.toSec()*1e6);
  // update IMU measurements
  _accel.x = msg.linear_acceleration.x;
  _accel.y = msg.linear_acceleration.y;
  _accel.z = msg.linear_acceleration.z;

  _gyro.x = msg.angular_velocity.x;
  _gyro.y = msg.angular_velocity.y;
  _gyro.z = msg.angular_velocity.z;

  _imu_time = now;

  // update estimate
  run_estimator();

  // publish estimate
  fcu_common::Attitude attitude_msg;
  attitude_msg.header.stamp = ros::Time::now();
  attitude_msg.roll = _current_state.phi;
  attitude_msg.pitch = _current_state.theta;
  attitude_msg.yaw = _current_state.psi;
  attitude_msg.p = _current_state.p;
  attitude_msg.q = _current_state.q;
  attitude_msg.r = _current_state.r;
  estimate_pub_.publish(attitude_msg);

  // Run Controller
  fcu_common::Command alt_msg, angle_msg, rate_msg, pt_msg;
  run_controller();
  alt_msg.mode = fcu_common::Command::MODE_ROLL_PITCH_YAWRATE_ALTITUDE;
  angle_msg.mode = fcu_common::Command::MODE_ROLL_PITCH_YAWRATE_THROTTLE;
  rate_msg.mode = fcu_common::Command::MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE;
  pt_msg.mode = fcu_common::Command::MODE_PASS_THROUGH;

  rate_msg.x = _combined_control.x.value;
  rate_msg.y = _combined_control.y.value;
  rate_msg.z = _combined_control.z.value;
  rate_msg.F = _combined_control.F.value;

  pt_msg.x = _command.x;
  pt_msg.y = _command.y;
  pt_msg.z = _command.z;
  pt_msg.F = _command.F;

  rate_pub_.publish(rate_msg);
  passthrough_pub_.publish(pt_msg);

  // Mix Outputs
  mix_output();
  fcu_common::ServoOutputRaw ESC_signals;
  ESC_signals.header.stamp = ros::Time::now();
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
    desired_forces_(i,0) = motors_[i].rotor.F1*signal*signal + motors_[i].rotor.F2*signal + motors_[i].rotor.F3;
    desired_torques_(i,0) = -1.0*motors_[i].rotor.T1*signal*signal + motors_[i].rotor.T2*signal + motors_[i].rotor.T3;

    // Then, Calculate Actual force and torque for each rotor using first-order dynamics
    double tau = (desired_forces_(i,0) > actual_forces_(i,0)) ? motors_[i].rotor.tau_up : motors_[i].rotor.tau_down;
    double alpha = sampling_time_/(tau + sampling_time_);
    actual_forces_(i,0) = sat((1-alpha)*actual_forces_(i) + alpha*desired_forces_(i), motors_[i].rotor.max, 0.0);
    actual_torques_(i,0) = sat((1-alpha)*actual_torques_(i) + alpha*desired_torques_(i), motors_[i].rotor.max, 0.0);
  }

  // Use the allocation matrix to calculate the body-fixed force and torques
  Eigen::Vector4d output_forces_and_torques = force_allocation_matrix_*actual_forces_ + torque_allocation_matrix_*actual_torques_;

  // Calculate Ground Effect
  double z = -pd;
  double ground_effect = max(ground_effect_.a*z*z*z*z + ground_effect_.b*z*z*z + ground_effect_.c*z*z + ground_effect_.d*z + ground_effect_.e, 0);

  //  // Apply other forces (wind) <- follows "Quadrotors and Accelerometers - State Estimation With an Improved Dynamic Model"
  //  // By Rob Leishman et al.
  forces_.Fx = -linear_mu_*ur;
  forces_.Fy = -linear_mu_*vr;
  forces_.Fz = linear_mu_*wr + ground_effect + output_forces_and_torques(3);
  forces_.l = -angular_mu_*p + output_forces_and_torques(0);
  forces_.m = -angular_mu_*q + output_forces_and_torques(1);
  forces_.n = -angular_mu_*r + output_forces_and_torques(2);
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

void ROSflightSIL::init_param_int(param_id_t id, char name[PARAMS_NAME_LENGTH], int32_t value)
{
  memcpy(_params.names[id], name, PARAMS_NAME_LENGTH);
  _params.values[id] = value;
  _params.types[id] = PARAM_TYPE_INT32;
}

void ROSflightSIL::init_param_float(param_id_t id, char name[PARAMS_NAME_LENGTH], float value)
{
  memcpy(_params.names[id], name, PARAMS_NAME_LENGTH);
  _params.values[id] = *((int32_t *) &value);
  _params.types[id] = PARAM_TYPE_FLOAT;
}



GZ_REGISTER_MODEL_PLUGIN(ROSflightSIL);
}


