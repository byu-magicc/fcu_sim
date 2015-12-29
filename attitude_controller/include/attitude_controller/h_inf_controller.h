#include <Eigen/Eigen>
#include "relative_nav/simple_pid.h"


namespace attitude_controller
{

class HInfController
{
public:
  HInfController();
  HInfController(double w1, double w2, double w3, double w4);
  void setOmega(double w1, double w2, double w3, double w4);
  void setMassMatrix(Eigen::Matrix4d mass_matrix);
  Eigen::Vector3d computeHInfinityControl(Eigen::Vector3d current_attitude, Eigen::Vector3d desired_attitude,
                                          Eigen::Vector3d current_rates, Eigen::Vector3d desired_rates,
                                          Eigen::Vector3d desired_accelerations,
                                          double dt);
  void resetIntegrator();

private:
  // gains
  double w1_;
  double w2_;
  double w3_;
  double w4_;
  Eigen::Matrix4d mass_matrix_;

  // data memory
  Eigen::Vector3d integrated_error_;

  // Helper Functions
  Eigen::Matrix3d find_M(Eigen::Vector3d attitude);
  Eigen::Matrix3d find_C(Eigen::Vector3d attitude, Eigen::Vector3d rates);
  Eigen::Matrix3d find_Kp(Eigen::Matrix3d M, Eigen::Matrix3d C);
  Eigen::Matrix3d find_Kd(Eigen::Matrix3d M, Eigen::Matrix3d C);
  Eigen::Matrix3d find_Ki(Eigen::Matrix3d M, Eigen::Matrix3d C);
};


}
