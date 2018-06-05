#include "BodyFootPosEstimator.hpp"
#include <Mercury_Controller/MoCapManager.hpp>

BodyFootPosEstimator::BodyFootPosEstimator(RobotSystem* robot){
  mocap_manager_ = new MoCapManager(robot);
  mocap_manager_->start();

}
BodyFootPosEstimator::~BodyFootPosEstimator(){
  delete mocap_manager_;
}

void BodyFootPosEstimator::Initialization(){
	mocap_manager_->CoordinateUpdateCall();
}

void BodyFootPosEstimator::getMoCapBodyOri(dynacore::Quaternion & quat){
  quat = mocap_manager_->body_quat_;
}

void BodyFootPosEstimator::getMoCapBodyVel(dynacore::Vect3 & body_vel){	
	body_vel = mocap_manager_->body_led_vel_; 
}
