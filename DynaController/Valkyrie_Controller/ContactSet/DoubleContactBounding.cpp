#include "DoubleContactBounding.hpp"
#include <Valkyrie/Valkyrie_Model.hpp>
#include <Valkyrie/Valkyrie_Definition.h>
#include <Valkyrie_Controller/Valkyrie_StateProvider.hpp>
#include <Utils/utilities.hpp>

DoubleContactBounding::DoubleContactBounding(RobotSystem* robot, int trans_pt):
  WBDC_ContactSpec(6),
  trans_pt_(trans_pt),
    mu_trans_(0.3),
    mu_stance_(0.3)
{
  robot_sys_ = robot;
  sp_ = Valkyrie_StateProvider::getStateProvider();
  Jc_ = dynacore::Matrix(6, valkyrie::num_qdot);

  ieq_vec_ = dynacore::Vector::Zero(5*2 + 1);
  // printf("[Double ContactBounding] Constructed\n");
}

DoubleContactBounding::~DoubleContactBounding(){}

bool DoubleContactBounding::_UpdateJc(){
  dynacore::Matrix Jtmp;
  robot_sys_->getFullJacobian(valkyrie_link::rightFoot, Jtmp);
  Jc_.block(0, 0, 3, valkyrie::num_qdot) = Jtmp.block(3, 0, 3, valkyrie::num_qdot);

  robot_sys_->getFullJacobian(valkyrie_link::leftFoot, Jtmp);
  Jc_.block(3, 0, 3, valkyrie::num_qdot) = Jtmp.block(3, 0, 3, valkyrie::num_qdot);

   //dynacore::pretty_print(Jc_, std::cout, "[double contact bounding] Jc");
  return true;
}
bool DoubleContactBounding::_UpdateJcDotQdot(){
  dynacore::Matrix JcDot(dim_contact_, valkyrie::num_qdot);
  dynacore::Matrix jcdot_tmp;
  // Right
  robot_sys_->getFullJacobianDot(valkyrie_link::rightFoot, jcdot_tmp);
  JcDot.block(0, 0, 3, valkyrie::num_qdot) = jcdot_tmp.block(3, 0, 3, valkyrie::num_qdot);
  // Left
  robot_sys_->getFullJacobianDot(valkyrie_link::leftFoot, jcdot_tmp);
  JcDot.block(3, 0, 3, valkyrie::num_qdot) = jcdot_tmp.block(3, 0, 3, valkyrie::num_qdot);

  //dynacore::pretty_print(JcDot, std::cout,  "[double contact bounding] JcDot");
  // TEST
  JcDot.setZero();
  JcDotQdot_ = JcDot * sp_->Qdot_;

  JcDotQdot_.setZero();
  return true;
}

bool DoubleContactBounding::_UpdateUf(){
  double mu_small(0.3);
    // U *x + ineq > 0
  int size_u(5);
  Uf_ = dynacore::Matrix::Zero(size_u*2+1, dim_contact_);
  dynacore::Matrix U;
  if(trans_pt_ == valkyrie_link::rightFoot){
      _setU(mu_trans_, U);
      Uf_.block(0, 0, size_u, 3) = U;
      _setU(mu_stance_, U);
      Uf_.block(size_u, 3, size_u, 3) = U;

      Uf_(size_u*2, 2) = -1.;
  }else if(trans_pt_ == valkyrie_link::leftFoot){
      _setU(mu_stance_, U);
      Uf_.block(0, 0, size_u, 3) = U;
      _setU(mu_trans_, U);
      Uf_.block(size_u, 3, size_u, 3) = U;

     Uf_(size_u*2, 5) = -1.;
  }else{
    printf("[Double Contact Bounding] Incorrect Foot Idx: %i\n", trans_pt_);
  }
  return true;
}
void DoubleContactBounding::setFzUpperLimit(double lim){
  ieq_vec_[10] = lim;
}

bool DoubleContactBounding::_UpdateInequalityVector(){

  return true;
}

void DoubleContactBounding::_setU(double mu, dynacore::Matrix & U){
  U = dynacore::Matrix::Zero(5, 3);

  U(0, 2) = 1.;

  U(1, 0) = 1.; U(1, 2) = mu;
  U(2, 0) = -1.; U(2, 2) = mu;

  U(3, 1) = 1.; U(3, 2) = mu;
  U(4, 1) = -1.; U(4, 2) = mu;
}
