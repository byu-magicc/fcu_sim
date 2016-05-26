#include "attitude_controller/h_inf_controller.h"

namespace attitude_controller
{

HInfController::HInfController()
{
  integrated_error_ = Eigen::Vector3d::Zero();
  setOmega(0.0, 1.0, 0.0, 1.0);
}


HInfController::HInfController(double w1, double w2, double w3, double w4)
{
  integrated_error_ = Eigen::Vector3d::Zero();
  setOmega(w1, w2, w3, w4);
}


void HInfController::setOmega(double w1, double w2, double w3, double w4)
{
  w1_ = w1;
  w2_ = w2;
  w3_ = w3;
  w4_ = w4;
}

void HInfController::resetIntegrator()
{
  integrated_error_ = integrated_error_ = Eigen::Vector3d::Zero();
}


void HInfController::setMassMatrix(Eigen::Matrix4d mass_matrix)
{
  mass_matrix_ = mass_matrix;
}


Eigen::Vector3d HInfController::computeHInfinityControl(
    Eigen::Vector3d eta, Eigen::Vector3d eta_r, Eigen::Vector3d etadot,
    Eigen::Vector3d etadot_r, Eigen::Vector3d etaddot_r, double dt)
{

  // find mass and damping matrices
  Eigen::Matrix3d M = find_M(eta);
  Eigen::Matrix3d C = find_C(eta, etadot);

  // update gains
  Eigen::Matrix3d Kp = find_Kp(M, C);
  Eigen::Matrix3d Ki = find_Ki(M, C);
  Eigen::Matrix3d Kd = find_Kd(M, C);

  // calculate tildas
  Eigen::Vector3d eta_tilda = eta - eta_r;
  Eigen::Vector3d etadot_tilda = etadot - etadot_r;

  // calculate integral
  //integrated_error_ += etadot_tilda*dt;

  // calculate output
  Eigen::Vector3d T;
  T = M*etaddot_r + C*etadot_r - M*(Kp*eta_tilda + Ki*integrated_error_ + Kd*etadot_tilda);
//  ROS_INFO_STREAM("output: \n" << T);
  return T;
}


Eigen::Matrix3d HInfController::find_Kp(Eigen::Matrix3d M, Eigen::Matrix3d C)
{
  Eigen::Matrix3d I =Eigen::Matrix3d::Identity();
  return (w3_/w1_)*I + sqrt(pow(w2_,2)+2*w1_*w3_)/w1_*M.inverse()*(C+1/pow(w4_,2)*I);
}


Eigen::Matrix3d HInfController::find_Kd(Eigen::Matrix3d M, Eigen::Matrix3d C)
{
  Eigen::Matrix3d I =Eigen::Matrix3d::Identity();
  return sqrt(pow(w2_,2)+2*w1_*w3_)/w1_*I + M.inverse()*(C+1/pow(w4_,2)*I);
}


Eigen::Matrix3d HInfController::find_Ki(Eigen::Matrix3d M, Eigen::Matrix3d C)
{
  Eigen::Matrix3d I =Eigen::Matrix3d::Identity();
  return (w3_/w1_)*M.inverse()*(C+1/pow(w4_,2)*I);
}


Eigen::Matrix3d HInfController::find_C(Eigen::Vector3d attitude, Eigen::Vector3d rates)
{
  double phi = attitude(0);
  double theta = attitude(1);
  double psi = attitude(2);
  double dphi = rates(0);
  double dtheta = rates(1);
  double dpsi = rates(2);
  double Ixx = mass_matrix_(0,0);
  double Iyy = mass_matrix_(1,1);
  double Izz = mass_matrix_(2,2);
  double m = mass_matrix_(3,3);

  // pre-calculate frequently used trigs
  double st = sin(theta);
  double ct = cos(theta);
  double cp = cos(phi);
  double sp = sin(phi);

  Eigen::Matrix3d c;
  c(0,0) = 0;
  c(0,1) = (Iyy-Izz)*(dtheta*cp*sp + dpsi*pow(sp,2)*ct) + (Izz-Iyy)*dpsi*pow(cp,2)*ct
              - Ixx*dpsi*ct;
  c(0,2) = (Izz-Iyy)*dpsi*cp*sp*pow(ct,2);
  c(1,0) = (Izz-Iyy)*(dtheta*cp*sp+dpsi*pow(sp,2)*ct) + (Iyy-Izz)*dpsi*pow(cp,2)*ct
              + Ixx*dpsi*ct;
  c(1,1) = (Izz-Iyy)*dphi*cp*sp;
  c(1,2) = -Ixx*dpsi*st*ct + Iyy*dpsi*pow(sp,2)*ct*st + Izz*dpsi*pow(cp,2)*st*ct;
  c(2,0) = (Iyy-Izz)*dpsi*pow(ct,2)*sp*cp - Ixx*dtheta*ct;
  c(2,1) = (Izz-Iyy)*(dtheta*cp*sp*st + dphi*pow(sp,2)*ct) + (Iyy-Izz)*dphi*pow(cp,2)*ct
              + Ixx*dpsi*st*ct - Iyy*dpsi*pow(sp,2)*st*ct - Izz*dpsi*pow(cp,2)*st*ct;
  c(2,2) = (Iyy-Izz)*dphi*cp*sp*pow(ct,2) - Iyy*dtheta*pow(sp,2)*ct*st - Izz*dtheta*pow(cp,2)*ct*st
              + Ixx*dtheta*ct*st;
  return c;

}


Eigen::Matrix3d HInfController::find_M(Eigen::Vector3d attitude){
  double phi = attitude(0);
  double theta = attitude(1);
  double psi = attitude(2);
  double Ixx = mass_matrix_(0,0);
  double Iyy = mass_matrix_(1,1);
  double Izz = mass_matrix_(2,2);
  double m = mass_matrix_(3,3);

  // pre-calculate frequently used trigs
  double st = sin(theta);
  double ct = cos(theta);
  double cp = cos(phi);
  double sp = sin(phi);

  Eigen::Matrix3d out_m;
  // upper diagonal
  out_m(0,0) = Ixx;
  out_m(0,1) = 0;
  out_m(0,2) = -Ixx*st;
  out_m(1,1) = Iyy*pow(cp,2) + Izz*pow(sp,2);
  out_m(1,2) = (Iyy-Izz)*cp*sp*ct;
  out_m(2,2) = Ixx*pow(st,2) + Iyy*pow(sp,2)*pow(ct,2) + Izz*pow(cp,2)*pow(ct,2);

  // lower diagonal (symmetric matrix)
  out_m(1,0) = out_m(0,1);
  out_m(2,0) = out_m(0,2);
  out_m(2,1) = out_m(1,2);

  return out_m;
}

}
