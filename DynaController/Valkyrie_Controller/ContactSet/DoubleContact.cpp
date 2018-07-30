#include "DoubleContact.hpp"
#include <Valkyrie/Valkyrie_Model.hpp>
#include <Valkyrie/Valkyrie_Definition.h>
#include <Valkyrie_Controller/Valkyrie_StateProvider.hpp>
#include <Utils/utilities.hpp>

DoubleContact::DoubleContact(RobotSystem* robot):WBDC_ContactSpec(12)
{
  robot_sys_ = robot;
  sp_ = Valkyrie_StateProvider::getStateProvider();
  Jc_ = dynacore::Matrix(dim_contact_, valkyrie::num_qdot);
  // printf("[Double Contact] Constructed\n");
}

DoubleContact::~DoubleContact(){}

bool DoubleContact::_UpdateJc(){
  dynacore::Matrix Jtmp;
  robot_sys_->getFullJacobian(valkyrie_link::rightFoot, Jtmp);
  Jc_.block(0, 0, 6, valkyrie::num_qdot) = Jtmp;

  robot_sys_->getFullJacobian(valkyrie_link::leftFoot, Jtmp);
  Jc_.block(6, 0, 6, valkyrie::num_qdot) = Jtmp;

  // dynacore::pretty_print(Jc_, std::cout, "double] Jc");
  return true;
}
bool DoubleContact::_UpdateJcDotQdot(){
  JcDotQdot_ = dynacore::Vector::Zero(dim_contact_);
  return true;
}

bool DoubleContact::_UpdateUf(){
    double mu(0.3);
  double X(0.08);
  double Y(0.05);

  int size_u(17);
  Uf_ = dynacore::Matrix::Zero(size_u*2, dim_contact_);

  dynacore::Matrix U;
  _setU(X, Y, mu, U);
  Eigen::Quaternion<double> quat_tmp;

  robot_sys_->getOri(valkyrie_link::rightCOP_Frame, quat_tmp);
  Eigen::Matrix3d R_rfoot_mtx(quat_tmp);
  dynacore::Matrix R_rfoot(6,6); R_rfoot.setZero();
  R_rfoot.block(0,0, 3,3) = R_rfoot_mtx.transpose();
  R_rfoot.block(3,3, 3,3) = R_rfoot_mtx.transpose();

  robot_sys_->getOri(valkyrie_link::leftCOP_Frame, quat_tmp);
  Eigen::Matrix3d R_lfoot_mtx(quat_tmp);
  dynacore::Matrix R_lfoot(6,6); R_lfoot.setZero();
  R_lfoot.block(0,0, 3,3) = R_lfoot_mtx.transpose();
  R_lfoot.block(3,3, 3,3) = R_lfoot_mtx.transpose();

  Uf_.block(0,0, size_u, 6) = U * R_rfoot;
  Uf_.block(size_u, 6, size_u, 6) = U * R_lfoot;

  return true;
}

bool DoubleContact::_UpdateInequalityVector(){
  ieq_vec_ = dynacore::Vector::Zero(17*2);
  ieq_vec_[0] = 5.0;
  ieq_vec_[17] = 5.0;

  return true;
}

void DoubleContact::_setU(double x, double y, double mu, dynacore::Matrix & U){
    U = dynacore::Matrix::Zero(17, 6);

  U(0, 5) = 1.;

  U(1, 3) = 1.; U(1, 5) = mu;
  U(2, 3) = -1.; U(2, 5) = mu;

  U(3, 4) = 1.; U(3, 5) = mu;
  U(4, 4) = -1.; U(4, 5) = mu;

  U(5, 0) = 1.; U(5, 5) = y;
  U(6, 0) = -1.; U(6, 5) = y;

  U(7, 1) = 1.; U(7, 5) = x;
  U(8, 1) = -1.; U(8, 5) = x;

  // Tau
  U(9, 0) = -mu; U(9, 1) = -mu; U(9, 2) = 1;
  U(9, 3) = y;   U(9, 4) = x;   U(9, 5) = (x + y)*mu;

  U(10, 0) = -mu; U(10, 1) = mu; U(10, 2) = 1;
  U(10, 3) = y;   U(10, 4) = -x; U(10, 5) = (x + y)*mu;

  U(11, 0) = mu; U(11, 1) = -mu; U(11, 2) = 1;
  U(11, 3) = -y; U(11, 4) = x;   U(11, 5) = (x + y)*mu;

  U(12, 0) = mu; U(12, 1) = mu; U(12, 2) = 1;
  U(12, 3) = -y; U(12, 4) = -x; U(12, 5) = (x + y)*mu;
  /////////////////////////////////////////////////
  U(13, 0) = -mu; U(13, 1) = -mu; U(13, 2) = -1;
  U(13, 3) = -y;  U(13, 4) = -x;  U(13, 5) = (x + y)*mu;

  U(14, 0) = -mu; U(14, 1) = mu; U(14, 2) = -1;
  U(14, 3) = -y;  U(14, 4) = x;  U(14, 5) = (x + y)*mu;

  U(15, 0) = mu; U(15, 1) = -mu; U(15, 2) = -1;
  U(15, 3) = y;  U(15, 4) = -x;  U(15, 5) = (x + y)*mu;

  U(16, 0) = mu; U(16, 1) = mu; U(16, 2) = -1;
  U(16, 3) = y;  U(16, 4) = x;  U(16, 5) = (x + y)*mu;
  // ////////////////////////////////////////////////////
}
