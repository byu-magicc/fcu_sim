#include "attitude_controller/attitude_controller.h"

namespace attitude_controller
{

attitudeController::attitudeController() :
  nh_(ros::NodeHandle()),
  nh_private_(ros::NodeHandle("~")),
  multicopter_()
{
  ROS_WARN("1");
  // retrieve params
  double kp_roll, ki_roll, kd_roll, kp_pitch, ki_pitch, kd_pitch, kp_yaw, ki_yaw, kd_yaw;
  int frame_type;
  double frame_radius, force_constant, moment_constant, mass, Jxx, Jyy, Jzz,max_rotor_speed;
  nh_private_.param<double>("kp_roll", kp_roll, 7.6394);
  nh_private_.param<double>("ki_roll", ki_roll, 0);
  nh_private_.param<double>("kd_roll", kd_roll, 0.9592);
  nh_private_.param<double>("kp_pitch", kp_pitch, 7.6394);
  nh_private_.param<double>("ki_pitch", ki_pitch, 0);
  nh_private_.param<double>("kd_pitch", kd_pitch, 1.5596);
  nh_private_.param<double>("kp_yaw", kp_yaw, 2.8648);
  nh_private_.param<double>("ki_yaw", ki_yaw, 0);
  nh_private_.param<double>("kd_yaw", kd_yaw, 0);
  nh_private_.param<double>("mass", mass, 3.81);
  nh_private_.param<double>("Jxx", Jxx, 0.060224);
  nh_private_.param<double>("Jyy", Jyy, 0.122198);
  nh_private_.param<double>("Jzz", Jzz, 0.0132166);
  nh_private_.param<double>("max_rotor_speed", max_rotor_speed, 903.2);
  nh_private_.param<int>("frame_type", frame_type, I6);
  nh_private_.param<double>("frame_radius", frame_radius, .3429);
  nh_private_.param<double>("force_constant", force_constant, 1.3e-6);
  nh_private_.param<double>("moment_constant", moment_constant, 0.25);
  nh_private_.param<std::string>("odometry_topic", odometry_topic_, "odometry");
  nh_private_.param<std::string>("command_topic", command_topic_, "command/roll_pitch_yawrate_thrust");
  nh_private_.param<std::string>("motor_speed_command_topic", motor_speed_command_topic_, "command/motor_speed");
  ROS_WARN("2");
  // Setup publishers and subscribers
  odometry_subscriber_ = nh_.subscribe(odometry_topic_, 1, &attitudeController::odometryCallback, this);
  command_subscriber_ = nh_.subscribe(command_topic_, 1, &attitudeController::commandCallback, this);
  actuators_publisher_ = nh_.advertise<mav_msgs::Actuators>(motor_speed_command_topic_, 1);
  ROS_WARN("3");
  // set PID gains
  pid_roll_.setGains(kp_roll, ki_roll, kd_roll);
  pid_pitch_.setGains(kp_pitch, ki_pitch, kd_pitch);
  pid_yaw_rate_.setGains(kp_yaw, ki_yaw, kd_yaw);

  // set times to zero
  dt_ = 0;
  time_of_last_control_ = 0;
  thrust_c_ = 0.0;
  roll_c_ = 0.0;
  pitch_c_ = 0.0;
  yaw_rate_c_ = 0.0;

  // initialize frame parameters
  multicopter_.initialize(frame_type, frame_radius, force_constant, moment_constant, Jxx, Jyy, Jzz, mass,max_rotor_speed);

  // initialize command vectors
  rotor_velocities_.resize(multicopter_.num_rotors);
  desired_forces_.resize(4);
}

void attitudeController::odometryCallback(const nav_msgs::OdometryConstPtr &msg){
  ROS_INFO("Odometry CB");
  tf::Quaternion current_orientation;
  tf::quaternionMsgToTF(msg->pose.pose.orientation,current_orientation);
  double yaw;
  tf::Matrix3x3(current_orientation).getRPY(phi_, theta_,yaw);
  r_ = -msg->twist.twist.angular.z;
  updatePIDLoops();
  multicopter_.mixOutput(&rotor_velocities_, &desired_forces_);
  mav_msgs::Actuators command;
  for(int i=0; i<multicopter_.num_rotors; i++){
    // saturate command
    //ROS_INFO_STREAM("rotor " << i << " unsaturated velocity = " << rotor_velocities_[i]);
    rotor_velocities_[i] = (rotor_velocities_[i]<0.0)?0.0:rotor_velocities_[i];
    rotor_velocities_[i] = (rotor_velocities_[i]>multicopter_.max_rotor_speed)?multicopter_.max_rotor_speed:rotor_velocities_[i];
    //ROS_INFO_STREAM("rotor " << i << " saturated velocity = " << rotor_velocities_[i]);
    command.angular_velocities.push_back(rotor_velocities_[i]);
  }
  command.header.stamp = msg->header.stamp;
  command.header.frame_id = msg->header.frame_id;
  actuators_publisher_.publish(command);
}

void attitudeController::commandCallback(const mav_msgs::RollPitchYawrateThrustConstPtr& msg){
  roll_c_ = msg->roll;
  pitch_c_ = msg->pitch;
  yaw_rate_c_ = -msg->yaw_rate;
  thrust_c_ = msg->thrust.z;
}

void attitudeController::updatePIDLoops(){
  double current_time = ros::Time::now().toSec();
  if (time_of_last_control_ != 0){
    dt_ = current_time - time_of_last_control_;
  }
  desired_forces_(0) = pid_roll_.computePID(roll_c_, phi_, dt_); // l
  desired_forces_(1) = pid_pitch_.computePID(pitch_c_, theta_, dt_);// m
  desired_forces_(2) = pid_yaw_rate_.computePID(yaw_rate_c_, r_, dt_); // n

  desired_forces_(3) = thrust_c_; // fz
  time_of_last_control_ = current_time;

  ROS_WARN_STREAM(" yr_c " << yaw_rate_c_ << " r_ " << r_);
  ROS_INFO_STREAM("l_c = " << desired_forces_(0) << " m_c = " << desired_forces_(1) << " n_c " << desired_forces_(2) << " thrust_c " << desired_forces_(3));

}

} // namespace attitude_controller
