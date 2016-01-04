#include "attitude_controller/attitude_controller.h"

namespace attitude_controller
{

attitudeController::attitudeController() :
  nh_(ros::NodeHandle()),
  nh_private_(ros::NodeHandle("~")),
  multicopter_()
{
  // retrieve frame params
  multicopter_.loadfromParam(nh_private_);

  // dynamic reconfigure
  func_ = boost::bind(&attitudeController::gainCallback, this, _1, _2);
  server_.setCallback(func_);

  // retrieve gain params for PID control
//  double kp_roll, ki_roll, kd_roll, kp_pitch, ki_pitch, kd_pitch, kp_yaw, ki_yaw, kd_yaw;
//  double w1, w2, w3, wu;
//  nh_private_.param<double>("PID_gains/kp_roll", kp_roll, 0.55);
//  nh_private_.param<double>("PID_gains/ki_roll", ki_roll, 0);
//  nh_private_.param<double>("PID_gains/kd_roll", kd_roll, 0.35);
//  nh_private_.param<double>("PID_gains/kp_pitch", kp_pitch, 0.55);
//  nh_private_.param<double>("PID_gains/ki_pitch", ki_pitch, 0);
//  nh_private_.param<double>("PID_gains/kd_pitch", kd_pitch, 0.35);
//  nh_private_.param<double>("PID_gains/kp_yaw", kp_yaw, 0.1);
//  nh_private_.param<double>("PID_gains/ki_yaw", ki_yaw, 0);
//  nh_private_.param<double>("PID_gains/kd_yaw", kd_yaw, 0);
//  nh_private_.param<double>("HInf_gains/w1", w1, 0.1);
//  nh_private_.param<double>("HInf_gains/w2", w2, 3);
//  nh_private_.param<double>("HInf_gains/w3", w3, 9);
//  nh_private_.param<double>("HInf_gains/wu", wu, 1.5);
//  nh_private_.param<int>("controller_type", controller_type_, PID_CONTROL);

  // retrieve topic names
  nh_private_.param<std::string>("odometry_topic", odometry_topic_, "odometry");
  nh_private_.param<std::string>("command_topic", command_topic_, "command");
  nh_private_.param<std::string>("motor_speed_command_topic", motor_speed_command_topic_, "command/motor_speed");

  // Setup publishers and subscribers
  odometry_subscriber_ = nh_.subscribe(odometry_topic_, 1, &attitudeController::odometryCallback, this);
  command_subscriber_ = nh_.subscribe(command_topic_, 1, &attitudeController::commandCallback, this);
  actuators_publisher_ = nh_.advertise<rotor_gazebo::Actuators>(motor_speed_command_topic_, 1);

  // intialize proper controller
//  if(controller_type_ == PID_CONTROL){
//    ROS_WARN("using PID Control");
//    // set PID gains
//    pid_roll_.setGains(kp_roll, ki_roll, kd_roll);
//    pid_pitch_.setGains(kp_pitch, ki_pitch, kd_pitch);
//    pid_yaw_rate_.setGains(kp_yaw, ki_yaw, kd_yaw);
//  }else if(controller_type_ == HINF_CONTROL){
//    ROS_WARN("using H-Inf Control");
//    h_inf_.setOmega(w1,w2,w3,wu);
//    h_inf_.setMassMatrix(multicopter_.inertia_matrix);
//    h_inf_.resetIntegrator();
//  }else{
//    ROS_ERROR_STREAM("Improper controller type: " << controller_type_);
//  }

  // set time and outputs to zero
  dt_ = 0;
  time_of_last_control_ = 0;
  thrust_c_ = 0.0;
  roll_c_ = 0.0;
  pitch_c_ = 0.0;
  psidot_c_ = 0.0;

  // initialize command vectors
  rotor_velocities_.resize(multicopter_.num_rotors);
  desired_forces_.resize(4);
}


void attitudeController::commandCallback(const relative_nav::CommandConstPtr& msg){
  roll_c_ = msg->roll;
  pitch_c_ = msg->pitch;
  psidot_c_ = msg->yaw_rate;
  thrust_c_ = msg->thrust;
}


void attitudeController::odometryCallback(const nav_msgs::OdometryConstPtr &msg){
  tf::Quaternion current_orientation;
  tf::quaternionMsgToTF(msg->pose.pose.orientation,current_orientation);
  tf::Matrix3x3(current_orientation).getRPY(phi_, theta_,psi_);

  theta_ *= -1.0; // NWU to NED
  psi_ *= -1.0; // NWU to NED

  p_ = msg->twist.twist.angular.x;
  q_ = -1.0*msg->twist.twist.angular.y; // NWU to NED
  r_ = -1.0*msg->twist.twist.angular.z; // NWU to NED

  // update loop time
  double current_time = ros::Time::now().toSec();
  if (time_of_last_control_ != 0){
    dt_ = current_time - time_of_last_control_;
  }

  // update control
  if(controller_type_ == PID_CONTROL){
    desired_forces_ = updatePIDControl();
  }else if(controller_type_ == HINF_CONTROL){
    desired_forces_ = updateHInfControl();
  }

  // save loop time
  time_of_last_control_ = ros::Time::now().toSec();

  // mix output
  multicopter_.mixOutput(&rotor_velocities_, &desired_forces_);
  rotor_gazebo::Actuators command;
  for(int i=0; i<multicopter_.num_rotors; i++){
    // saturate command
    rotor_velocities_[i] = sqrt((rotor_velocities_[i]<0.0)?0.0:rotor_velocities_[i]);
    rotor_velocities_[i] = (rotor_velocities_[i]>multicopter_.rotors[i].max_rotor_speed)?multicopter_.rotors[i].max_rotor_speed:rotor_velocities_[i];
    command.angular_velocities.push_back(rotor_velocities_[i]);
  }
  command.header.stamp = msg->header.stamp;
  command.header.frame_id = msg->header.frame_id;
  actuators_publisher_.publish(command);
}


Eigen::Vector4d attitudeController::updatePIDControl(){
  Eigen::Vector4d desired_forces;
  desired_forces(0) = pid_roll_.computePID(roll_c_, phi_, dt_); // l
  desired_forces(1) = pid_pitch_.computePID(pitch_c_, theta_, dt_);// m
  desired_forces(2) = pid_yaw_rate_.computePID(psidot_c_, r_, dt_); // n
  desired_forces(3) = thrust_c_; // fz
  return desired_forces;
}


Eigen::Vector4d attitudeController::updateHInfControl(){
  Eigen::Vector3d eta, eta_r, etadot, etadot_r, etaddot_r, moments, omega;
  Eigen::Vector4d desired_forces;
  eta << phi_, theta_, psi_;
  eta_r << roll_c_, pitch_c_, yaw_c_;
  Eigen::Matrix3d R;
  R << 1, sin(phi_)*tan(theta_), cos(phi_)*tan(theta_),
       0, cos(phi_), -sin(phi_),
       0, sin(phi_)*1/cos(theta_), cos(phi_)*1/cos(theta_);
  omega << p_, q_, r_;
  etadot = R*omega;
  etadot_r << phidot_c_, thetadot_c_, psidot_c_;
  etaddot_r << phiddot_c_, thetaddot_c_, psiddot_c_;
  moments = h_inf_.computeHInfinityControl(eta, eta_r, etadot, etadot_r, etaddot_r,dt_);
  desired_forces << moments(0), moments(1), moments(2), thrust_c_;
  ROS_INFO_STREAM("desired_forces " << desired_forces);
  return desired_forces;
}

void attitudeController::gainCallback(attitude_controller::GainConfig &config, uint32_t level)
{
  controller_type_ = config.controller;
  // intialize proper controller
  if(controller_type_ == PID_CONTROL){
    ROS_WARN("using PID Control");
    ROS_WARN_STREAM("New Gains => Roll: { P: = " << config.roll_P << " I: = " << config.roll_I << " D: = " << config.roll_D);
    ROS_WARN_STREAM("New Gains => Pitch: { P: = " << config.pitch_P << " I: = " << config.pitch_I << " D: = " << config.pitch_D);
    ROS_WARN_STREAM("New Gains => yaw: { P: = " << config.yaw_rate_P << " I: = " << config.yaw_rate_I << " D: = " << config.yaw_rate_D);
    // set PID gains
    pid_roll_.setGains(config.roll_P, config.roll_I, config.roll_D);
    pid_pitch_.setGains(config.pitch_P, config.pitch_I, config.pitch_D);
    pid_yaw_rate_.setGains(config.yaw_rate_P, config.yaw_rate_I, config.yaw_rate_D);
    pid_roll_.clearIntegrator();
    pid_pitch_.clearIntegrator();
    pid_yaw_rate_.clearIntegrator();
  }else if(controller_type_ == HINF_CONTROL){
    ROS_WARN("using H-Inf Control");
    h_inf_.setOmega(config.w1,config.w2,config.w3,config.wu);
    h_inf_.setMassMatrix(multicopter_.inertia_matrix);
    h_inf_.resetIntegrator();
  }else{
    ROS_ERROR_STREAM("Improper controller type: " << controller_type_);
  }
}



























} // namespace attitude_controller
