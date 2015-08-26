/* multicopter.h
 * defines physical constants for a multirotor
 */
#include <vector>
#include <ros/ros.h>
#include <Eigen/Eigen>


#define I6 1
#define X6 2
#define I4 3
#define X4 4
#define H4 5


struct Rotor{
  double angle;
  double radius;
  double force_constant;
  double moment_constant;
  double direction;
  double max_rotor_speed;
};

class multicopter
{
public:
  std::vector<Rotor> rotors;
  Eigen::Matrix4Xd allocation_matrix;
  Eigen::Matrix4d inertia_matrix;
  int num_rotors;

  multicopter(){}

  ~multicopter(){ clear(); }

  void clear(){
    rotors.clear();
    num_rotors = 0;
    ROS_WARN("1.1.1");
    //allocation_matrix.resize(0,0);
    ROS_WARN("1.1.2");
    inertia_matrix.resize(4,4);
    ROS_WARN("1.1.2");
    inertia_matrix.setIdentity();
    ROS_WARN("1.1.3");
  }

  void loadfromParam(ros::NodeHandle& nh_private){
    ROS_WARN("loading params");
    // clear any previous data
    clear();

    ROS_WARN("1.1");
    // load inertial data - only use diagonal
    nh_private.param<double>("inertia/xx", inertia_matrix(0,0), 0.060224);
    nh_private.param<double>("inertia/yy", inertia_matrix(1,1), 0.122198);
    nh_private.param<double>("inertia/zz", inertia_matrix(2,2), 0.0977);
    nh_private.param<double>("mass",       inertia_matrix(3,3), 3.81);
    ROS_INFO_STREAM("INERTIA MATRIX \n" << inertia_matrix);

    // load rotor configuration - default to shredder
    int i = 0;
    double dummy_double;
    while(nh_private.getParam("rotor_configuration/" + std::to_string(i) + "/angle", dummy_double)){
      Rotor rotor;
      nh_private.param<double>("rotor_configuration/" + std::to_string(i) + "/angle", rotor.angle, 0.0);
      nh_private.param<double>("rotor_configuration/" + std::to_string(i) + "/arm_length" , rotor.radius, 0.3429);
      nh_private.param<double>("rotor_configuration/" + std::to_string(i) + "/rotor_force_constant" , rotor.force_constant, 1.426e-5);
      nh_private.param<double>("rotor_configuration/" + std::to_string(i) + "/rotor_moment_constant" , rotor.moment_constant, 0.25);
      nh_private.param<double>("rotor_configuration/" + std::to_string(i) + "/direction" , rotor.direction, 1.0);
      nh_private.param<double>("rotor_configuration/" + std::to_string(i) + "/max_rotor_speed", rotor.max_rotor_speed, 903.2);
      rotors.push_back(rotor);
      i++;
    }
    num_rotors = rotors.size();
    ROS_WARN_STREAM("number of rotors: " << num_rotors);
    calculateAllocationMatrix();
  }

//  void initialize(int frame_type_, double frame_radius, double force_constant,
//                  double moment_constant, double Jxx_, double Jyy_, double Jzz_,
//                  double mass_, double max_rotor_speed_)  {
//    inertia_matrix.setZero();
//    inertia_matrix(0,0) = Jxx_;
//    inertia_matrix(1,1) = Jyy_;
//    inertia_matrix(2,2) = Jzz_;
//    inertia_matrix(3,3) = mass_;
//    Jxx = Jxx_;
//    Jyy = Jyy_;
//    Jzz = Jzz_;
//    mass = mass_;
//    num_rotors = 0;
//    max_rotor_speed = max_rotor_speed_;
//    frame_type = frame_type_;
//    switch (frame_type)    {
//      case I6:
//        num_rotors = 6;
//        rotors.resize(num_rotors);
//        rotors[0].angle = 0.0;
//        rotors[1].angle = 1.0472;
//        rotors[2].angle = 2.0944;
//        rotors[3].angle = 3.1416;
//        rotors[4].angle = -2.0944;
//        rotors[5].angle = -1.0472;
//        break;
//      case X6:
//        num_rotors = 6;
//        rotors.resize(num_rotors);
//        rotors[0].angle = 0.5236;
//        rotors[1].angle = 1.5708;
//        rotors[2].angle = 2.6180;
//        rotors[3].angle = -2.6180;
//        rotors[4].angle = -1.5708;
//        rotors[5].angle = -0.5236;
//      case I4:
//        num_rotors = 4;
//        rotors.resize(num_rotors);
//        rotors[0].angle = 0.0;
//        rotors[1].angle = 1.5708;
//        rotors[2].angle = 3.1416;
//        rotors[3].angle = -1.5708;
//        break;
//      case X4:
//        num_rotors = 4;
//        rotors.resize(num_rotors);
//        rotors[0].angle = 0.7854;
//        rotors[1].angle = 2.3562;
//        rotors[2].angle = -2.3562;
//        rotors[3].angle = -0.7854;
//      case H4:
//        num_rotors = 4;
//        rotors.resize(num_rotors);
//        ROS_ERROR_STREAM("Hard-Coded Angles for H4 Frame");
//        rotors[0].angle = 0.7854;
//        rotors[1].angle = 2.3562;
//        rotors[2].angle = -2.3562;
//        rotors[3].angle = -0.7854;
//      default:
//        ROS_ERROR_STREAM("Unknown frame type");
//    }
//    for(int i = 0; i<num_rotors; i++)    {
//      rotors[i].radius = frame_radius;
//      rotors[i].force_constant = force_constant;
//      rotors[i].moment_constant = moment_constant;
//      rotors[i].direction = pow(-1.0,i);
//    }
//    calculateAllocationMatrix();
//  }

  void mixOutput(Eigen::VectorXd* rotor_velocities, Eigen::Vector4d* commands)
  {
    // pseudoinverse map from [l,m,n,fz] to [w1,w2,...,wn]
    (*rotor_velocities) = (allocation_matrix.transpose()*(allocation_matrix*allocation_matrix.transpose()).inverse())*(*commands);
//    ROS_WARN_STREAM("rotor_velocities = " << (*rotor_velocities)(0) << " " <<
//                                             (*rotor_velocities)(1) << " " <<
//                                             (*rotor_velocities)(2) << " " <<
//                                             (*rotor_velocities)(3) << " " <<
//                                             (*rotor_velocities)(4) << " " <<
//                                             (*rotor_velocities)(5));
//    double roll_c =     (*commands)(0);
//    double pitch_c =    (*commands)(1);
//    double yaw_rate_c = (*commands)(2);
//    double thrust_c =   (*commands)(3);
//    rotor_velocities->resize(num_rotors);
//    switch(frame_type){
//      case I6:
//        (*rotor_velocities)(0) = (thrust_c/6.0)/rotors[0].force_constant;
//        (*rotor_velocities)(1) = (thrust_c/6.0)/rotors[1].force_constant;
//        (*rotor_velocities)(2) = (thrust_c/6.0)/rotors[2].force_constant;
//        (*rotor_velocities)(3) = (thrust_c/6.0)/rotors[3].force_constant;
//        (*rotor_velocities)(4) = (thrust_c/6.0)/rotors[4].force_constant;
//        (*rotor_velocities)(5) = (thrust_c/6.0)/rotors[5].force_constant;
//        ROS_WARN_STREAM("thrust_c = " << thrust_c << " rotor_vel " << (*rotor_velocities)(5) <<  "force_const " << rotors[5].force_constant);
//        break;
//      default:
//        ROS_ERROR("cannot map inputs to unknown frame");
//    }
  }

private:
  void calculateAllocationMatrix()  {
    allocation_matrix.resize(4, num_rotors);
    for(int i = 0; i<num_rotors; i++){
      allocation_matrix(0,i) = sin(rotors[i].angle) * rotors[i].radius * rotors[i].force_constant;
      allocation_matrix(1,i) = -1.0*cos(rotors[i].angle) * rotors[i].radius * rotors[i].force_constant;
      allocation_matrix(2,i) = -rotors[i].direction * rotors[i].moment_constant;
      allocation_matrix(3,i) = rotors[i].force_constant;
    }
    ROS_INFO_STREAM("ALLOCATION MATRIX: \n" <<allocation_matrix);
  }

};
