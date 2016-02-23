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

#include "fcu_sim_plugins/gazebo_aircraft_forces_and_moments.h"

namespace gazebo
{

GazeboAircraftForcesAndMoments::GazeboAircraftForcesAndMoments() :
  ModelPlugin(), node_handle_(nullptr), prev_sim_time_(0)  {}


GazeboAircraftForcesAndMoments::~GazeboAircraftForcesAndMoments()
{
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
  if (node_handle_) {
    node_handle_->shutdown();
    delete node_handle_;
  }
}


void GazeboAircraftForcesAndMoments::SendForces()
{
  // apply the forces and torques to the joint
  link_->AddRelativeForce(math::Vector3(forces_.Fx, -forces_.Fy, -forces_.Fz));
  link_->AddRelativeTorque(math::Vector3(forces_.l, -forces_.m, -forces_.n));
}


void GazeboAircraftForcesAndMoments::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
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
    gzerr << "[gazebo_aircraft_forces_and_moments] Please specify a namespace.\n";
  node_handle_ = new ros::NodeHandle(namespace_);

 if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[gazebo_aircraft_forces_and_moments] Please specify a linkName of the forces and moments plugin.\n";
  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("[gazebo_aircraft_forces_and_moments] Couldn't find specified link \"" << link_name_ << "\".");

  /* Load Params from Gazebo Server */
  getSdfParam<std::string>(_sdf, "windSpeedTopic", wind_speed_topic_, "wind");
  getSdfParam<std::string>(_sdf, "commandTopic", command_topic_, "command");

  // physical parameters
  getSdfParam<double>(_sdf, "mass", mass_, 13.5);
  getSdfParam<double>(_sdf, "Jx", Jx_, 0.8244);
  getSdfParam<double>(_sdf, "Jy", Jy_, 1.135);
  getSdfParam<double>(_sdf, "Jz", Jz_, 1.759);
  getSdfParam<double>(_sdf, "Jxz", Jxz_, .1204);
  getSdfParam<double>(_sdf, "rho", rho_, 1.2682);

  // Wing Geometry
  getSdfParam<double>(_sdf, "wing_s", wing_.S, 0.55);
  getSdfParam<double>(_sdf, "wing_b", wing_.b, 2.8956);
  getSdfParam<double>(_sdf, "wing_c", wing_.c, 0.18994);
  getSdfParam<double>(_sdf, "wing_M", wing_.M, 0.55);
  getSdfParam<double>(_sdf, "wing_epsilon", wing_.epsilon, 2.8956);
  getSdfParam<double>(_sdf, "wing_alpha0", wing_.alpha0, 0.18994);

  // Propeller Coefficients
  getSdfParam<double>(_sdf, "k_motor",  prop_.k_motor, 80.0);
  getSdfParam<double>(_sdf, "k_T_P",  prop_.k_T_P, 0.0);
  getSdfParam<double>(_sdf, "k_Omega",  prop_.k_Omega, 0.0);
  getSdfParam<double>(_sdf, "prop_e",  prop_.e, 0.9);
  getSdfParam<double>(_sdf, "prop_S",  prop_.S, 0.202);
  getSdfParam<double>(_sdf, "prop_C",  prop_.C, 1.);

  // Lift Params
  getSdfParam<double>(_sdf, "C_L_O", CL_.O, 0.28);
  getSdfParam<double>(_sdf, "C_L_alpha", CL_.alpha, 3.45);
  getSdfParam<double>(_sdf, "C_L_beta", CL_.beta, 0.0);
  getSdfParam<double>(_sdf, "C_L_p", CL_.p, 0.0);
  getSdfParam<double>(_sdf, "C_L_q", CL_.q, 0.0);
  getSdfParam<double>(_sdf, "C_L_r", CL_.r, 0.0);
  getSdfParam<double>(_sdf, "C_L_delta_a", CL_.delta_a, 0.0);
  getSdfParam<double>(_sdf, "C_L_delta_e", CL_.delta_e, -0.36);
  getSdfParam<double>(_sdf, "C_L_delta_r", CL_.delta_r, 0.0);

  // Drag Params
  getSdfParam<double>(_sdf, "C_D_O", CD_.O, 0.03);
  getSdfParam<double>(_sdf, "C_D_alpha", CD_.alpha, 0.30);
  getSdfParam<double>(_sdf, "C_D_beta", CD_.beta, 0.0);
  getSdfParam<double>(_sdf, "C_D_p", CD_.p, 0.0437);
  getSdfParam<double>(_sdf, "C_D_q", CD_.q, 0.0);
  getSdfParam<double>(_sdf, "C_D_r", CD_.r, 0.0);
  getSdfParam<double>(_sdf, "C_D_delta_a", CD_.delta_a, 0.0);
  getSdfParam<double>(_sdf, "C_D_delta_e", CD_.delta_e, 0.0);
  getSdfParam<double>(_sdf, "C_D_delta_r", CD_.delta_r, 0.0);

  // ell Params (x axis moment)
  getSdfParam<double>(_sdf, "C_ell_O", Cell_.O, 0.0);
  getSdfParam<double>(_sdf, "C_ell_alpha", Cell_.alpha, 0.00);
  getSdfParam<double>(_sdf, "C_ell_beta", Cell_.beta, -0.12);
  getSdfParam<double>(_sdf, "C_ell_p", Cell_.p, -0.26);
  getSdfParam<double>(_sdf, "C_ell_q", Cell_.q, 0.0);
  getSdfParam<double>(_sdf, "C_ell_r", Cell_.r, 0.14);
  getSdfParam<double>(_sdf, "C_ell_delta_a", Cell_.delta_a, 0.08);
  getSdfParam<double>(_sdf, "C_ell_delta_e", Cell_.delta_e, 0.0);
  getSdfParam<double>(_sdf, "C_ell_delta_r", Cell_.delta_r, 0.105);

  // m Params (y axis moment)
  getSdfParam<double>(_sdf, "C_m_O", Cm_.O, -0.02338);
  getSdfParam<double>(_sdf, "C_m_alpha", Cm_.alpha, -0.38);
  getSdfParam<double>(_sdf, "C_m_beta", Cm_.beta, 0.0);
  getSdfParam<double>(_sdf, "C_m_p", Cm_.p, 0.0);
  getSdfParam<double>(_sdf, "C_m_q", Cm_.q, -3.6);
  getSdfParam<double>(_sdf, "C_m_r", Cm_.r, 0.0);
  getSdfParam<double>(_sdf, "C_m_delta_a", Cm_.delta_a, 0.0);
  getSdfParam<double>(_sdf, "C_m_delta_e", Cm_.delta_e, -0.5);
  getSdfParam<double>(_sdf, "C_m_delta_r", Cm_.delta_r, 0.0);

  // n Params (z axis moment)
  getSdfParam<double>(_sdf, "C_n_O", Cn_.O, 0.0);
  getSdfParam<double>(_sdf, "C_n_alpha", Cn_.alpha, 0.0);
  getSdfParam<double>(_sdf, "C_n_beta", Cn_.beta, 0.25);
  getSdfParam<double>(_sdf, "C_n_p", Cn_.p, 0.022);
  getSdfParam<double>(_sdf, "C_n_q", Cn_.q, 0.0);
  getSdfParam<double>(_sdf, "C_n_r", Cn_.r, -0.35);
  getSdfParam<double>(_sdf, "C_n_delta_a", Cn_.delta_a, 0.06);
  getSdfParam<double>(_sdf, "C_n_delta_e", Cn_.delta_e, 0.0);
  getSdfParam<double>(_sdf, "C_n_delta_r", Cn_.delta_r, -0.032);

  // Y Params (Sideslip Forces)
  getSdfParam<double>(_sdf, "C_Y_O", CY_.O, 0.0);
  getSdfParam<double>(_sdf, "C_Y_alpha", CY_.alpha, 0.00);
  getSdfParam<double>(_sdf, "C_Y_beta", CY_.beta, -0.98);
  getSdfParam<double>(_sdf, "C_Y_p", CY_.p, 0.0);
  getSdfParam<double>(_sdf, "C_Y_q", CY_.q, 0.0);
  getSdfParam<double>(_sdf, "C_Y_r", CY_.r, 0.0);
  getSdfParam<double>(_sdf, "C_Y_delta_a", CY_.delta_a, 0.0);
  getSdfParam<double>(_sdf, "C_Y_delta_e", CY_.delta_e, 0.0);
  getSdfParam<double>(_sdf, "C_Y_delta_r", CY_.delta_r, -0.017);


  // Connect the update function to the simulation
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboAircraftForcesAndMoments::OnUpdate, this, _1));

  // Connect Subscribers
  command_sub_ = node_handle_->subscribe(command_topic_, 1, &GazeboAircraftForcesAndMoments::CommandCallback, this);
  wind_speed_sub_ = node_handle_->subscribe(wind_speed_topic_, 1, &GazeboAircraftForcesAndMoments::WindSpeedCallback, this);
}

// This gets called by the world update event.
void GazeboAircraftForcesAndMoments::OnUpdate(const common::UpdateInfo& _info) {

  sampling_time_ = _info.simTime.Double() - prev_sim_time_;
  prev_sim_time_ = _info.simTime.Double();
  UpdateForcesAndMoments();
  SendForces();
}

void GazeboAircraftForcesAndMoments::WindSpeedCallback(const geometry_msgs::Vector3 &wind){
  wind_.N = wind.x;
  wind_.E = wind.y;
  wind_.D = wind.z;
}

void GazeboAircraftForcesAndMoments::CommandCallback(const fcu_common::CommandConstPtr &msg)
{
  delta_.t = msg->normalized_throttle;
  delta_.e = msg->normalized_pitch;
  delta_.a = msg->normalized_roll;
  delta_.r = msg->normalized_yaw;
}


void GazeboAircraftForcesAndMoments::UpdateForcesAndMoments()
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
  double ur = u - wind_.N;
  double vr = v - wind_.E;
  double wr = w - wind_.D;

  double Va = sqrt(pow(ur,2.0) + pow(vr,2.0) + pow(wr,2.0));
  double alpha = atan2(wr , ur);
  double beta = asin(vr/Va);

  double sign = (alpha >= 0? 1: -1);//Sigmoid function
  double sigma_a = (1 + exp(-(wing_.M*(alpha - wing_.alpha0))) + exp((wing_.M*(alpha + wing_.alpha0))))/((1 + exp(-(wing_.M*(alpha - wing_.alpha0))))*(1 + exp((wing_.M*(alpha + wing_.alpha0)))));
  double CL_a = (1 - sigma_a)*(CL_.O + CL_.alpha*alpha) + sigma_a*(2*sign*pow(sin(alpha),2.0)*cos(alpha));
  double AR = (pow(wing_.b, 2.0))/wing_.S;
  double CD_a = CD_.p + ((pow((CL_.O + CL_.alpha*(alpha)),2.0))/(3.14159*0.9*AR));//the const 0.9 in this equation replaces the e (Oswald Factor) variable and may be inaccurate

  double CX_a = -CD_a*cos(alpha) + CL_a*sin(alpha);
  double CX_q_a = -CD_.q*cos(alpha) + CL_.q*sin(alpha);
  double CX_deltaE_a = -CD_.delta_e*cos(alpha) + CL_.delta_e*sin(alpha);

  double CZ_a = -CD_a*sin(alpha) - CL_a*cos(alpha);
  double CZ_q_a = -CD_.q*sin(alpha) - CL_.q*cos(alpha);
  double CZ_deltaE_a = -CD_.delta_e*sin(alpha) - CL_.delta_e*cos(alpha);



  // calculate forces

  /*
   * Pack Forces and Moments into the forces_ member for publishing in
   * SendForces()
   */
  if(Va < 0.1){ // not moving (we need this check to make sure we don't divide by zero)
    forces_.Fx = 0.5*rho_*prop_.S*prop_.C*(pow((prop_.k_motor*delta_.t),2.0) - pow(Va,2.0));
    forces_.Fy = 0.0;
    forces_.Fz = 0.0;
    forces_.l = 0.0;
    forces_.m = 0.0;
    forces_.n = 0.0;
  }else{
    forces_.Fx = 0.5*(rho_)*pow(Va,2.0)*wing_.S*(CX_a + (CX_q_a*wing_.c*q)/(2.0*Va) + CX_deltaE_a * delta_.e) + 0.5*rho_*prop_.S*prop_.C*(pow((prop_.k_motor*delta_.t),2.0) - pow(Va,2.0));
    forces_.Fy = 0.5*(rho_)*pow(Va,2.0)*wing_.S*(CY_.O + CY_.beta*beta + ((CY_.p*wing_.b*p)/(2.0*Va)) + ((CY_.r*wing_.b*r)/(2.0*Va)) + CY_.delta_a*delta_.a + CY_.delta_r*delta_.r);
    forces_.Fz = 0.5*(rho_)*pow(Va,2.0)*wing_.S*(CZ_a + (CZ_q_a*wing_.c*q)/(2.0*Va) + CZ_deltaE_a * delta_.e);

    forces_.l = 0.5*(rho_)*pow(Va,2.0)*wing_.S*wing_.b*(Cell_.O + Cell_.beta*beta + (Cell_.p*wing_.b*p)/(2.0*Va) + (Cell_.r*wing_.b*r)/(2.0*Va) + Cell_.delta_a*delta_.a + Cell_.delta_r*delta_.r) - prop_.k_T_P*pow((prop_.k_Omega*delta_.t),2.0);
    forces_.m = 0.5*(rho_)*pow(Va,2.0)*wing_.S*wing_.c*(Cm_.O + Cm_.alpha*alpha + (Cm_.q*wing_.c*q)/(2.0*Va) + Cm_.delta_e*delta_.e);
    forces_.n = 0.5*(rho_)*pow(Va,2.0)*wing_.S*wing_.b*(Cn_.O + Cn_.beta*beta + (Cn_.p*wing_.b*p)/(2.0*Va) + (Cn_.r*wing_.b*r)/(2.0*Va) + Cn_.delta_a*delta_.a + Cn_.delta_r*delta_.r);
  }
}

GZ_REGISTER_MODEL_PLUGIN(GazeboAircraftForcesAndMoments);
}
