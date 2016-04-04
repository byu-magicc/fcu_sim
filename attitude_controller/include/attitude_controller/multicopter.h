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
    inertia_matrix.resize(4,4);
    inertia_matrix.setIdentity();
  }

  void loadfromParam(ros::NodeHandle& nh_private){
    clear();

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
    calculateAllocationMatrix();
  }

  void mixOutput(Eigen::VectorXd* rotor_velocities, Eigen::Vector4d* commands)
  {
    // pseudoinverse map from [l,m,n,fz] to [w1,w2,...,wn]
    (*rotor_velocities) = (allocation_matrix.transpose()*(allocation_matrix*allocation_matrix.transpose()).inverse())*(*commands);
  }

private:
  void calculateAllocationMatrix()  {
    allocation_matrix.resize(4, num_rotors);
    for(int i = 0; i<num_rotors; i++){
      allocation_matrix(0,i) = sin(rotors[i].angle) * rotors[i].radius * rotors[i].force_constant;
      allocation_matrix(1,i) = cos(rotors[i].angle) * rotors[i].radius * rotors[i].force_constant;
      allocation_matrix(2,i) = -rotors[i].direction * rotors[i].moment_constant * rotors[i].force_constant;
      allocation_matrix(3,i) = rotors[i].force_constant;
    }
    ROS_INFO_STREAM("ALLOCATION MATRIX: \n" <<allocation_matrix);
  }

};
