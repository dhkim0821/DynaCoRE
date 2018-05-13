#include "BasicAccumulation.hpp"
#include <Configuration.h>
#include <Utils/utilities.hpp>
#include <Mercury/Mercury_Definition.h>

BasicAccumulation::BasicAccumulation():OriEstimator(), com_state_(6){
  global_ori_.w() = 1.;
  global_ori_.x() = 0.;
  com_state_.setZero();

}
BasicAccumulation::~BasicAccumulation(){}

void BasicAccumulation::CoMStateInitialization(
        const dynacore::Vect3 & com_pos, 
        const dynacore::Vect3 & com_vel){

    // com_state_[0] = com_pos[0];
    // com_state_[1] = com_pos[1];
    // com_state_[2] = com_vel[0];
    // com_state_[3] = com_vel[1];
}

void BasicAccumulation::getEstimatedCoMState(dynacore::Vector & com_state){
    com_state = com_state_;
}


void BasicAccumulation::EstimatorInitialization(const dynacore::Quaternion & ini_quat,
                                                const std::vector<double> & acc,
                                                const std::vector<double> & ang_vel){
  global_ori_.w() = 1.;
  global_ori_.x() = 0.;
  global_ori_.y() = 0.;
  global_ori_.z() = 0.;

  for(int i(0); i<3; ++i){
      global_ang_vel_[i] = ang_vel[i];
    ini_acc_[i] = acc[i];
    }
}

void BasicAccumulation::setSensorData(const std::vector<double> & acc,
                                      const std::vector<double> & acc_inc,
                                      const std::vector<double> & ang_vel){
  // Orientation
  dynacore::Quaternion delt_quat;
  dynacore::Vect3 delta_th;
  double theta(0.);
  for(int i(0); i<3; ++i){
    delta_th[i] = ang_vel[i] * mercury::servo_rate;
    theta += delta_th[i] * delta_th[i];
  }

  if(fabs(theta) > 1.e-20){
    delt_quat.w() = cos(theta/2.);
    delt_quat.x() = sin(theta/2.) * delta_th[0]/theta;
    delt_quat.y() = sin(theta/2.) * delta_th[1]/theta;
    delt_quat.z() = sin(theta/2.) * delta_th[2]/theta;
  } else {
    delt_quat.w() = 1.;
    delt_quat.x() = 0.;
    delt_quat.y() = 0.;
    delt_quat.z() = 0.;
  }

  global_ori_ = dynacore::QuatMultiply(global_ori_, delt_quat);
  static int count(0);
  ++count;
  if(count%500 == 501){
    dynacore::pretty_print(acc, "[estimator] acc");
    dynacore::pretty_print(ang_vel, "[estimator] ang vel");

    dynacore::pretty_print(delt_quat, std::cout, "delta quat");
    dynacore::pretty_print(global_ori_, std::cout, "global ori");
  }
  dynacore::Quaternion ang_quat;
  ang_quat.w() = 0.;
  ang_quat.x() = ang_vel[0];
  ang_quat.y() = ang_vel[1];
  ang_quat.z() = ang_vel[2];

  dynacore::Quaternion quat_dot = dynacore::QuatMultiply(global_ori_, ang_quat, false);
  quat_dot = dynacore::QuatMultiply(quat_dot, global_ori_.inverse(), false);

  global_ang_vel_[0] = quat_dot.x();
  global_ang_vel_[1] = quat_dot.y();
  global_ang_vel_[2] = quat_dot.z();


  dynacore::Quaternion global_acc;
  global_acc.w() = 0.;
  global_acc.x() = acc[0];
  global_acc.y() = acc[1];
  global_acc.z() = acc[2];

  dynacore::Quaternion quat_acc = dynacore::QuatMultiply(global_ori_, global_acc, false);
    quat_acc = dynacore::QuatMultiply(quat_acc, global_ori_.inverse(), false);

    // com_state_[4] = quat_acc.x() - ini_acc_[0]; 
    // com_state_[5] = quat_acc.y() - ini_acc_[1];

    // TEST
    com_state_[4] = acc[0] - ini_acc_[0];
    com_state_[5] = acc[1] - ini_acc_[1];

    com_state_[2] = com_state_[2] + com_state_[4]*mercury::servo_rate;
    com_state_[3] = com_state_[3] + com_state_[5]*mercury::servo_rate;

    com_state_[0] = com_state_[0] + com_state_[2]*mercury::servo_rate;
    com_state_[1] = com_state_[1] + com_state_[3]*mercury::servo_rate;


}
