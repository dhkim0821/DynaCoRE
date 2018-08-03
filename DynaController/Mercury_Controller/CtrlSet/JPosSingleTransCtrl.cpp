#include "JPosSingleTransCtrl.hpp"
#include <Mercury_Controller/Mercury_StateProvider.hpp>
#include <Mercury_Controller/TaskSet/JPosTask.hpp>
#include <Mercury_Controller/ContactSet/FixedBodyContact.hpp>

#include <Mercury_Controller/TaskSet/ConfigTask.hpp>
#include <Mercury_Controller/ContactSet/DoubleContactBounding.hpp>

#include <Mercury_Controller/Mercury_StateProvider.hpp>
#include <WBDC_Rotor/WBDC_Rotor.hpp>
#include <ParamHandler/ParamHandler.hpp>
#include <Utils/DataManager.hpp>
#include <Mercury_Controller/Mercury_DynaControl_Definition.h>

JPosSingleTransCtrl::JPosSingleTransCtrl(RobotSystem* robot,
        int moving_foot, bool b_increase):
    Controller(robot),
    b_increase_(b_increase),
    end_time_(1000.0),
    b_jpos_set_(false),
    des_jpos_(mercury::num_act_joint),
    des_jvel_(mercury::num_act_joint),
   ctrl_start_time_(0.),
    set_jpos_(mercury::num_act_joint)
{
    set_jpos_.setZero();

    //jpos_task_ = new JPosTask();
    //contact_ = new FixedBodyContact(robot);
    jpos_task_ = new ConfigTask();
    contact_ = new DoubleContactBounding(robot, moving_foot);

    std::vector<bool> act_list;
    act_list.resize(mercury::num_qdot, true);
    for(int i(0); i<mercury::num_virtual; ++i) act_list[i] = false;

    wbdc_rotor_ = new WBDC_Rotor(act_list);
    wbdc_rotor_data_ = new WBDC_Rotor_ExtraData();
    wbdc_rotor_data_->A_rotor = 
        dynacore::Matrix::Zero(mercury::num_qdot, mercury::num_qdot);
    wbdc_rotor_data_->cost_weight = 
        dynacore::Vector::Constant(contact_->getDim() + 
                jpos_task_->getDim(), 100.0);
    wbdc_rotor_data_->cost_weight.tail(contact_->getDim()) = 
        dynacore::Vector::Constant(contact_->getDim(), 0.1);

    wbdc_rotor_data_->cost_weight[jpos_task_->getDim() + 2] = 0.001;
    wbdc_rotor_data_->cost_weight[jpos_task_->getDim() + 5] = 0.001;

    sp_ = Mercury_StateProvider::getStateProvider();

    //printf("[Joint Position Single Transition Controller] Constructeh\n");
}

JPosSingleTransCtrl::~JPosSingleTransCtrl(){
    delete jpos_task_;
    delete contact_;
    delete wbdc_rotor_;
    delete wbdc_rotor_data_;
}

void JPosSingleTransCtrl::OneStep(void* _cmd){
    _PreProcessing_Command();
    dynacore::Vector gamma = dynacore::Vector::Zero(mercury::num_act_joint);
    state_machine_time_ = sp_->curr_time_ - ctrl_start_time_;

    _contact_setup();
    _jpos_task_setup();
    _jpos_ctrl_wbdc_rotor(gamma);

    for(int i(0); i<mercury::num_act_joint; ++i){
        ((Mercury_Command*)_cmd)->jtorque_cmd[i] = gamma[i];
        ((Mercury_Command*)_cmd)->jpos_cmd[i] = des_jpos_[i];
        ((Mercury_Command*)_cmd)->jvel_cmd[i] = des_jvel_[i];
    }

    _PostProcessing_Command();
}

void JPosSingleTransCtrl::_jpos_ctrl_wbdc_rotor(dynacore::Vector & gamma){
    dynacore::Vector fb_cmd = dynacore::Vector::Zero(mercury::num_act_joint);
    for (int i(0); i<mercury::num_act_joint; ++i){
        wbdc_rotor_data_->A_rotor(i + mercury::num_virtual, i + mercury::num_virtual)
            = sp_->rotor_inertia_[i];
    }
    wbdc_rotor_->UpdateSetting(A_, Ainv_, coriolis_, grav_);
    wbdc_rotor_->MakeTorque(task_list_, contact_list_, fb_cmd, wbdc_rotor_data_);

    gamma = wbdc_rotor_data_->cmd_ff;

    sp_->qddot_cmd_ = wbdc_rotor_data_->result_qddot_;
}

void JPosSingleTransCtrl::_jpos_task_setup(){
    des_jvel_.setZero();

    if(b_jpos_set_){
        des_jpos_ = set_jpos_;
    }else{
        des_jpos_ = jpos_ini_;
    }
    //double Kp_roll(2.0);
    //double Kp_pitch(2.0);
    //des_jpos_[mercury_joint::rightAbduction - mercury::num_virtual] 
        //+= Kp_roll * sp_->Q_[mercury_joint::virtual_Rx];
    //des_jpos_[mercury_joint::leftAbduction - mercury::num_virtual] 
        //+= Kp_roll * sp_->Q_[mercury_joint::virtual_Rx];

    //des_jpos_[mercury_joint::rightHip - mercury::num_virtual] 
        //+= Kp_pitch * sp_->Q_[mercury_joint::virtual_Ry];
    //des_jpos_[mercury_joint::leftHip - mercury::num_virtual] 
        //+= Kp_pitch * sp_->Q_[mercury_joint::virtual_Ry];

    dynacore::Vector config_des = sp_->Q_;
    config_des.segment(mercury::num_virtual, mercury::num_act_joint) = 
        des_jpos_;
    
    dynacore::Vector qdot_des(mercury::num_qdot);
    qdot_des.setZero();
    dynacore::Vector qddot_des(mercury::num_qdot);
    qddot_des.setZero();

    jpos_task_->UpdateTask(&(config_des), qdot_des, qddot_des);
    task_list_.push_back(jpos_task_);
}

void JPosSingleTransCtrl::_contact_setup(){
    if(b_increase_){
        ((DoubleContactBounding*)contact_)->setFzUpperLimit(
            min_rf_z_ + state_machine_time_/end_time_ * (max_rf_z_ - min_rf_z_));
    } else {
        ((DoubleContactBounding*)contact_)->setFzUpperLimit(
            max_rf_z_ - state_machine_time_/end_time_ * (max_rf_z_ - min_rf_z_));
    }

    contact_->UpdateContactSpec();
    contact_list_.push_back(contact_);
}

void JPosSingleTransCtrl::FirstVisit(){
    jpos_ini_ = sp_->Q_.segment(mercury::num_virtual, mercury::num_act_joint);
    ctrl_start_time_ = sp_->curr_time_;
}

void JPosSingleTransCtrl::LastVisit(){
}

bool JPosSingleTransCtrl::EndOfPhase(){
    if(state_machine_time_ > end_time_){
        return true;
    }
    return false;
}
void JPosSingleTransCtrl::CtrlInitialization(const std::string & setting_file_name){
    jpos_ini_ = sp_->Q_.segment(mercury::num_virtual, mercury::num_act_joint);

    ParamHandler handler(MercuryConfigPath + setting_file_name + ".yaml");

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
