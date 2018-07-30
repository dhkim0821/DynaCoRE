#include "JPosTargetCtrl.hpp"
#include <Valkyrie_Controller/Valkyrie_StateProvider.hpp>
#include <Valkyrie_Controller/TaskSet/JPosTask.hpp>
#include <Valkyrie_Controller/ContactSet/FixedBodyContact.hpp>
#include <Valkyrie_Controller/Valkyrie_DynaControl_Definition.h>
#include <Valkyrie/Valkyrie_Model.hpp>
#include <WBDC/WBDC.hpp>
#include <ParamHandler/ParamHandler.hpp>
#include <Utils/utilities.hpp>

JPosTargetCtrl::JPosTargetCtrl(RobotSystem* robot):Controller(robot),
    jpos_target_(valkyrie::num_act_joint),
    end_time_(1000.0),
    ctrl_start_time_(0.),
    des_jpos_(valkyrie::num_act_joint),
    des_jvel_(valkyrie::num_act_joint)
{
    jpos_task_ = new JPosTask();
    fixed_body_contact_ = new FixedBodyContact(robot);

    std::vector<bool> act_list;
    act_list.resize(valkyrie::num_qdot, true);
    for(int i(0); i<valkyrie::num_virtual; ++i) act_list[i] = false;

    wbdc_ = new WBDC(act_list);
    wbdc_data_ = new WBDC_ExtraData();
    wbdc_data_->cost_weight = 
        dynacore::Vector::Constant(fixed_body_contact_->getDim() + 
                jpos_task_->getDim(), 100.0);

    wbdc_data_->cost_weight.tail(fixed_body_contact_->getDim()) = 
        dynacore::Vector::Constant(fixed_body_contact_->getDim(), 1.0);

    sp_ = Valkyrie_StateProvider::getStateProvider();

    printf( "[ CTRL - JPos Target] Constructed \n" );
}

JPosTargetCtrl::~JPosTargetCtrl(){
    delete jpos_task_;
    delete fixed_body_contact_;
    delete wbdc_;
    delete wbdc_data_;
}

void JPosTargetCtrl::OneStep(void* _cmd){
    _PreProcessing_Command();
    state_machine_time_ = sp_->curr_time_ - ctrl_start_time_;
    dynacore::Vector gamma;
    _fixed_body_contact_setup();
    _jpos_task_setup();
    _jpos_ctrl_wbdc(gamma);

    for(int i(0); i<valkyrie::num_act_joint; ++i){
        ((Valkyrie_Command*)_cmd)->jtorque_cmd[i] = gamma[i];
        ((Valkyrie_Command*)_cmd)->jpos_cmd[i] = des_jpos_[i];
        ((Valkyrie_Command*)_cmd)->jvel_cmd[i] = des_jvel_[i];
    }
    _PostProcessing_Command();
}

void JPosTargetCtrl::_jpos_ctrl_wbdc(dynacore::Vector & gamma){
    wbdc_->UpdateSetting(A_, Ainv_, coriolis_, grav_);
    wbdc_->MakeTorque(task_list_, contact_list_, gamma, wbdc_data_);
}


void JPosTargetCtrl::_jpos_task_setup(){
    dynacore::Vector jacc_des(valkyrie::num_act_joint); jacc_des.setZero();

    for(int i(0); i<valkyrie::num_act_joint; ++i){
        des_jpos_[i] = dynacore::smooth_changing(jpos_ini_[i], jpos_target_[i], end_time_, state_machine_time_);
        des_jvel_[i] = dynacore::smooth_changing_vel(jpos_ini_[i], jpos_target_[i], end_time_, state_machine_time_);
        jacc_des[i] = dynacore::smooth_changing_acc(jpos_ini_[i], jpos_target_[i], end_time_, state_machine_time_);
    }

    jpos_task_->UpdateTask(&(des_jpos_), des_jvel_, jacc_des);
    task_list_.push_back(jpos_task_);
}

void JPosTargetCtrl::_fixed_body_contact_setup(){
    fixed_body_contact_->UpdateContactSpec();
    contact_list_.push_back(fixed_body_contact_);
}

void JPosTargetCtrl::FirstVisit(){
    ctrl_start_time_ = sp_->curr_time_;
    jpos_ini_ = sp_->Q_.segment(valkyrie::num_virtual, valkyrie::num_act_joint);
}

void JPosTargetCtrl::LastVisit(){
}

bool JPosTargetCtrl::EndOfPhase(){
    if(state_machine_time_ > end_time_){
        return true;
    }
    return false;
}

void JPosTargetCtrl::CtrlInitialization(const std::string & setting_file_name){
    jpos_ini_ = sp_->Q_.segment(valkyrie::num_virtual, valkyrie::num_act_joint);
    ParamHandler handler(ValkyrieConfigPath + setting_file_name + ".yaml");

    std::vector<double> tmp_vec;
    // Feedback Gain
    handler.getVector("Kp", tmp_vec);
    for(int i(0); i<tmp_vec.size(); ++i){
        ((JPosTask*)jpos_task_)->Kp_vec_[i] = tmp_vec[i];
    }

    handler.getVector("Kd", tmp_vec);
    for(int i(0); i<tmp_vec.size(); ++i){
        ((JPosTask*)jpos_task_)->Kd_vec_[i] = tmp_vec[i];
    }
}

void JPosTargetCtrl::setTargetPosition(const std::vector<double>& jpos){
    for(int i(0); i<valkyrie::num_act_joint; ++i){
        jpos_target_[i] = jpos[i];
    }
}
