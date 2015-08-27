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

  // retrieve gain params
  double kp_roll, ki_roll, kd_roll, kp_pitch, ki_pitch, kd_pitch, kp_yaw, ki_yaw, kd_yaw;
  nh_private_.param<double>("control_gains/kp_roll", kp_roll, 0.55);
  nh_private_.param<double>("control_gains/ki_roll", ki_roll, 0);
  nh_private_.param<double>("control_gains/kd_roll", kd_roll, 0.35);
  nh_private_.param<double>("control_gains/kp_pitch", kp_pitch, 0.55);
  nh_private_.param<double>("control_gains/ki_pitch", ki_pitch, 0);
  nh_private_.param<double>("control_gains/kd_pitch", kd_pitch, 0.35);
  nh_private_.param<double>("control_gains/kp_yaw", kp_yaw, 5.0);
  nh_private_.param<double>("control_gains/ki_yaw", ki_yaw, 0);
  nh_private_.param<double>("control_gains/kd_yaw", kd_yaw, 0);

  // retrieve topic names
  nh_private_.param<std::string>("odometry_topic", odometry_topic_, "odometry");
  nh_private_.param<std::string>("command_topic", command_topic_, "command");
  nh_private_.param<std::string>("motor_speed_command_topic", motor_speed_command_topic_, "command/motor_speed");

  // Setup publishers and subscribers
  odometry_subscriber_ = nh_.subscribe(odometry_topic_, 1, &attitudeController::odometryCallback, this);
  command_subscriber_ = nh_.subscribe(command_topic_, 1, &attitudeController::commandCallback, this);
  actuators_publisher_ = nh_.advertise<rotor_gazebo::Actuators>(motor_speed_command_topic_, 1);

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

  // initialize command vectors
  rotor_velocities_.resize(multicopter_.num_rotors);
  desired_forces_.resize(4);
}

void attitudeController::commandCallback(const relative_nav_msgs::CommandConstPtr& msg){
  roll_c_ = msg->roll;
  pitch_c_ = msg->pitch;
  yaw_rate_c_ = msg->yaw_rate;
  thrust_c_ = msg->thrust;
}

void attitudeController::odometryCallback(const nav_msgs::OdometryConstPtr &msg){
  tf::Quaternion current_orientation;
  tf::quaternionMsgToTF(msg->pose.pose.orientation,current_orientation);
  double yaw;
  tf::Matrix3x3(current_orientation).getRPY(phi_, theta_,yaw);
  r_ = -1.0*msg->twist.twist.angular.z; // NWU to NED
  updatePIDLoops();
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

  //ROS_WARN_STREAM("yaw_c = " << yaw_rate_c_ << " r = " << r_ << " n = " << desired_forces_(2));

}

} // namespace attitude_controller
