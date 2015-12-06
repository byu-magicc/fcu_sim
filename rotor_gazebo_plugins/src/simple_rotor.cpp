#include <rotor_gazebo_plugins/simple_rotor.h>


namespace gazebo{

SimpleRotor::SimpleRotor() :
  ModelPlugin(), nh_(nullptr) {}


SimpleRotor::~SimpleRotor()
{
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
  if (nh_){
    nh_->shutdown();
    delete nh_;
  }
}

void SimpleRotor::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
  model_ = _model;
  world_ = model_->GetWorld();

  namespace_.clear();

  if (_sdf->HasElement("namespace"))
    namespace_ = _sdf->GetElement("namespace")->Get<std::string>();
  else
    gzerr << "[simple_rotor] Please specify a namespace.\n";
  nh_ = new ros::NodeHandle(namespace_);

 if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[simple_rotor] Please specify a linkName of the forces and moments plugin.\n";
  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("[simple_rotor] Couldn't find specified link \"" << link_name_ << "\".");
  if (_sdf->HasElement("jointName"))
    joint_name_ = _sdf->GetElement("jointName")->Get<std::string>();
  else
    gzerr << "[gazebo_motor_model] Please specify a jointName, where the rotor is attached.\n";
  // Get the pointer to the joint.
  joint_ = model_->GetJoint(joint_name_);
  if (joint_ == NULL)
    gzthrow("[gazebo_motor_model] Couldn't find specified joint \"" << joint_name_ << "\".");


  if(_sdf->HasElement("motorNumber"))
    motor_number_ = _sdf->GetElement("motorNumber")->Get<double>();
  else
    gzthrow("[simple_rotor] please spcify motor number.\n");
  std::string motor_direction;
  if(_sdf->HasElement("motorDirection"))
    motor_direction = _sdf->GetElement("motorDirection")->Get<std::string>();
  if(motor_direction == "cw"){
    direction_ = -1;
    gzerr << "CW";
  }else if(motor_direction == "ccw"){
    direction_ = 1;
    gzerr << "CCW";
  }else{
    gzthrow("[simple_rotor] specify motor direction: cw or ccw \n");
  }

  getSdfParam<double>(_sdf, "k_force", k_force_, 1.0);
  getSdfParam<double>(_sdf, "k_torque", k_torque_, 1.0);
  getSdfParam<std::string>(_sdf, "commandTopic", command_topic_, "command");


  // Connect the update function to the simulation
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&SimpleRotor::OnUpdate, this, _1));

  command_sub_ = nh_->subscribe(command_topic_, 1, &SimpleRotor::commandCallback, this);
  force_ = 0;
  torque_ = 0;
}

void SimpleRotor::InitializeParams(){}

void SimpleRotor::commandCallback(const rotor_gazebo::ActuatorsConstPtr& msg)
{
  force_ = msg->angular_velocities[motor_number_]*k_force_;
  torque_ = msg->angular_velocities[motor_number_]*k_torque_*direction_;
}

void SimpleRotor::SendForces()
{
  link_->AddRelativeForce(math::Vector3(0, 0, 1));
  link_->AddRelativeTorque(math::Vector3(0,0,direction_*0.1));
  joint_->SetVelocity(0, direction_*M_PI);
}

void SimpleRotor::OnUpdate(const common::UpdateInfo& _info)
{
  SendForces();
}

GZ_REGISTER_MODEL_PLUGIN(SimpleRotor);
}
