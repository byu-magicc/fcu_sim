/* multicopter.h
 * defines physical constants for a multirotor
 */
#include <vector>
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
};

class multicopter
{
public:
  std::vector<Rotor> rotors;
  int num_rotors, frame_type;
  Eigen::Matrix4Xd allocation_matrix;
  Eigen::Matrix4d inertia_matrix;
  double Jxx, Jyy, Jzz, mass;
  double max_rotor_speed;

  multicopter(){}

  void setJxx(double Jxx_){
    Jxx = Jxx_;
    inertia_matrix(0,0) = Jxx_;
  }

  void setJyy(double Jyy_){
    Jyy = Jyy_;
    inertia_matrix(1,1) = Jyy_;
  }

  void setJzz(double Jzz_){
    Jzz = Jzz_;
    inertia_matrix(2,2) = Jzz_;
  }

  void setMass(double mass_){
    mass = mass_;
    inertia_matrix(3,3) = mass_;
  }

  void setNumRotors(int num_rotors_){
    num_rotors = num_rotors_;
  }

  void setMaxRotorSpeed(double max_rotor_speed_){
    max_rotor_speed = max_rotor_speed_;
  }

  void setRotor(int rotor_number, double force_constant, double moment_constant, double radius, double angle){
    if (rotor_number > num_rotors){
      ROS_ERROR_STREAM("incorrect rotor index: " << rotor_number << " is greater than number of rotors: " << num_rotors);
    }
    rotors[rotor_number].force_constant = force_constant;
    rotors[rotor_number].radius = radius;
    rotors[rotor_number].angle = angle;
    rotors[rotor_number].moment_constant = moment_constant;
    calculateAllocationMatrix();
  }

  void initialize(int frame_type_, double frame_radius, double force_constant,
                  double moment_constant, double Jxx_, double Jyy_, double Jzz_,
                  double mass_, double max_rotor_speed_)  {
    inertia_matrix.setZero();
    inertia_matrix(0,0) = Jxx_;
    inertia_matrix(1,1) = Jyy_;
    inertia_matrix(2,2) = Jzz_;
    inertia_matrix(3,3) = mass_;
    Jxx = Jxx_;
    Jyy = Jyy_;
    Jzz = Jzz_;
    mass = mass_;
    num_rotors = 0;
    max_rotor_speed = max_rotor_speed_;
    frame_type = frame_type_;
    switch (frame_type)    {
      case I6:
        num_rotors = 6;
        rotors.resize(num_rotors);
        rotors[0].angle = 0.0;
        rotors[1].angle = 1.0472;
        rotors[2].angle = 2.0944;
        rotors[3].angle = 3.1416;
        rotors[4].angle = -2.0944;
        rotors[5].angle = -1.0472;
        break;
      case X6:
        num_rotors = 6;
        rotors.resize(num_rotors);
        rotors[0].angle = 0.5236;
        rotors[1].angle = 1.5708;
        rotors[2].angle = 2.6180;
        rotors[3].angle = -2.6180;
        rotors[4].angle = -1.5708;
        rotors[5].angle = -0.5236;
      case I4:
        num_rotors = 4;
        rotors.resize(num_rotors);
        rotors[0].angle = 0.0;
        rotors[1].angle = 1.5708;
        rotors[2].angle = 3.1416;
        rotors[3].angle = -1.5708;
        break;
      case X4:
        num_rotors = 4;
        rotors.resize(num_rotors);
        rotors[0].angle = 0.7854;
        rotors[1].angle = 2.3562;
        rotors[2].angle = -2.3562;
        rotors[3].angle = -0.7854;
      case H4:
        num_rotors = 4;
        rotors.resize(num_rotors);
        ROS_ERROR_STREAM("Hard-Coded Angles for H4 Frame");
        rotors[0].angle = 0.7854;
        rotors[1].angle = 2.3562;
        rotors[2].angle = -2.3562;
        rotors[3].angle = -0.7854;
      default:
        ROS_ERROR_STREAM("Unknown frame type");
    }
    for(int i = 0; i<num_rotors; i++)    {
      rotors[i].radius = frame_radius;
      rotors[i].force_constant = force_constant;
      rotors[i].moment_constant = moment_constant;
      rotors[i].direction = pow(-1.0,i);
    }
    calculateAllocationMatrix();
  }

  ~multicopter()  {
    rotors.clear();
  }

  void setRotor(int i, double radius, double force_constant, double moment_constant, int direction)
  {
    rotors[i].radius = radius;
    rotors[i].force_constant = force_constant;
    rotors[i].moment_constant = moment_constant;
    rotors[i].direction = (direction > 0) - (direction < 0);
  }

  void mixOutput(Eigen::VectorXd* rotor_velocities, Eigen::Vector4d* commands)
  {
    // pseudoinverse map from [l,m,n,fz] to [w1,w2,...,wn]
    (*rotor_velocities) = (allocation_matrix.transpose()*(allocation_matrix*allocation_matrix.transpose()).inverse())*(*commands);
    ROS_WARN_STREAM("rotor_velocities = " << (*rotor_velocities)(0) << " " <<
                                             (*rotor_velocities)(1) << " " <<
                                             (*rotor_velocities)(2) << " " <<
                                             (*rotor_velocities)(3) << " " <<
                                             (*rotor_velocities)(4) << " " <<
                                             (*rotor_velocities)(5));


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
      // pitch row
      allocation_matrix(0,i) = sin(rotors[i].angle) * rotors[i].radius * rotors[i].force_constant;
      // roll row
      allocation_matrix(1,i) = -1.0*cos(rotors[i].angle) * rotors[i].radius * rotors[i].force_constant;
      // yaw row
      allocation_matrix(2,i) = -rotors[i].direction * rotors[i].moment_constant;
      // thrust row
      allocation_matrix(3,i) = rotors[i].force_constant;
    }
  }


private:

};
