#include "BodyFootPosEstimator.hpp"
#include <Mercury_Controller/MoCapManager.hpp>
#include <Utils/DataManager.hpp>
#include <Mercury/Mercury_Definition.h>

BodyFootPosEstimator::BodyFootPosEstimator(RobotSystem* robot){
  mocap_manager_ = new MoCapManager(robot);
  mocap_manager_->start();

  body_led_vel_.setZero();
   for(int i(0); i<3 ; ++i){
    vel_filter_.push_back(new deriv_lp_filter(2.*100.*M_PI, mercury::servo_rate));
  }
  DataManager::GetDataManager()->RegisterData(&body_led_vel_, VECT3, "Body_LED_vel", 3);
}
BodyFootPosEstimator::~BodyFootPosEstimator(){
  delete mocap_manager_;
}

void BodyFootPosEstimator::Update(){
  for(int i(0); i<3; ++i){
    vel_filter_[i]->input(mocap_manager_->led_pos_data_[i]);
    body_led_vel_[i] = vel_filter_[i]->output();
  }

}
void BodyFootPosEstimator::Initialization(){
	mocap_manager_->CoordinateUpdateCall();
}

void BodyFootPosEstimator::getMoCapBodyOri(dynacore::Quaternion & quat){
  quat = mocap_manager_->body_quat_;
}

void BodyFootPosEstimator::getMoCapBodyVel(dynacore::Vect3 & body_vel){	
	body_vel = body_led_vel_;
}
