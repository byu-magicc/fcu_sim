#ifndef rotor_gazebo_SIMPLE_ROTOR_H
#define rotor_gazebo_SIMPLE_ROTOR_H

#include <gazebo/physics/physics.hh>
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <rotor_gazebo/Actuators.h>
#include "rotor_gazebo_plugins/common.h"

namespace gazebo {

class SimpleRotor: public ModelPlugin {
public:
  SimpleRotor();
  ~SimpleRotor();
  void InitializeParams();

protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void commandCallback(const rotor_gazebo::ActuatorsConstPtr& msg);
  void SendForces();
  void OnUpdate(const common::UpdateInfo & /*_info*/);

private:
  std::string command_topic_;
  std::string link_name_;
  std::string joint_name_;
  std::string namespace_;

  physics::WorldPtr world_;
  physics::ModelPtr model_;
  physics::LinkPtr link_;
  physics::JointPtr joint_;
  event::ConnectionPtr updateConnection_;

  ros::NodeHandle* nh_;
  ros::Subscriber command_sub_;

  int motor_number_;
  int direction_;
  double k_force_;
  double k_torque_;
  double force_;
  double torque_;
};


}

#endif
