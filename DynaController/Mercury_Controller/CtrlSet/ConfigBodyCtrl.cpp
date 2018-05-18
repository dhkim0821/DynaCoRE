#include "ConfigBodyCtrl.hpp"
#include <Mercury_Controller/Mercury_StateProvider.hpp>
#include <Mercury_Controller/TaskSet/ConfigTask.hpp>
#include <Mercury_Controller/ContactSet/DoubleContact.hpp>
#include <Mercury_Controller/Mercury_StateProvider.hpp>
#include <WBDC_Rotor/WBDC_Rotor.hpp>
#include <ParamHandler/ParamHandler.hpp>
#include <Utils/DataManager.hpp>
#include <Mercury/Mercury_Definition.h>

ConfigBodyCtrl::ConfigBodyCtrl(RobotSystem* robot):Controller(robot),
    end_time_(1000.0),
    ctrl_start_time_(0.),
    b_set_height_target_(false),
    des_jpos_(mercury::num_act_joint),
    des_jvel_(mercury::num_act_joint)
{
    jpos_task_ = new ConfigTask();
    double_body_contact_ = new DoubleContact(robot);
    
    std::vector<bool> act_list;
    act_list.resize(mercury::num_qdot, true);
    for(int i(0); i<mercury::num_virtual; ++i) act_list[i] = false;
    
    wbdc_rotor_ = new WBDC_Rotor(act_list);
    wbdc_rotor_data_ = new WBDC_Rotor_ExtraData();
    wbdc_rotor_data_->A_rotor =  dynacore::Matrix::Zero(mercury::num_qdot, mercury::num_qdot);
   
    wbdc_rotor_data_->cost_weight = 
    dynacore::Vector::Constant(
        jpos_task_->getDim() + double_body_contact_->getDim(), 100.0);

    // wbdc_rotor_data_->cost_weight[0] = 10;    
    // wbdc_rotor_data_->cost_weight[1] = 10;    
    // wbdc_rotor_data_->cost_weight[2] = 200;    

    wbdc_rotor_data_->cost_weight.tail(double_body_contact_->getDim()) = 
        dynacore::Vector::Constant(double_body_contact_->getDim(), 1.);

    wbdc_rotor_data_->cost_weight[jpos_task_->getDim() + 2] = 0.001;
    wbdc_rotor_data_->cost_weight[jpos_task_->getDim() + 5] = 0.001;

    sp_ = Mercury_StateProvider::getStateProvider();

    printf("[Config Body Control] Constructed\n");
}

ConfigBodyCtrl::~ConfigBodyCtrl(){
    delete jpos_task_;
    delete double_body_contact_;
    delete wbdc_rotor_;
    delete wbdc_rotor_data_;
}

void ConfigBodyCtrl::OneStep(void* _cmd){
    _PreProcessing_Command();
    state_machine_time_ = sp_->curr_time_ - ctrl_start_time_;

    dynacore::Vector gamma;
    _double_body_contact_setup();
    _jpos_task_setup();
    _jpos_ctrl_wbdc_rotor(gamma);

    for(int i(0); i<mercury::num_act_joint; ++i){
        ((Mercury_Command*)_cmd)->jtorque_cmd[i] = gamma[i];
        ((Mercury_Command*)_cmd)->jpos_cmd[i] = des_jpos_[i];
        ((Mercury_Command*)_cmd)->jvel_cmd[i] = des_jvel_[i];
    }

    _PostProcessing_Command();
}

void ConfigBodyCtrl::_jpos_ctrl_wbdc_rotor(dynacore::Vector & gamma){
    dynacore::Vector fb_cmd = dynacore::Vector::Zero(mercury::num_act_joint);
    for (int i(0); i<mercury::num_act_joint; ++i){
        wbdc_rotor_data_->A_rotor(i + mercury::num_virtual, i + mercury::num_virtual)
            = sp_->rotor_inertia_[i];
    }
    wbdc_rotor_->UpdateSetting(A_, Ainv_, coriolis_, grav_);
    wbdc_rotor_->MakeTorque(task_list_, contact_list_, fb_cmd, wbdc_rotor_data_);

    gamma = wbdc_rotor_data_->cmd_ff;

    sp_->qddot_cmd_ = wbdc_rotor_data_->result_qddot_;
    dynacore::Vector reaction_force = 
             (wbdc_rotor_data_->opt_result_).tail(double_body_contact_->getDim());
    for(int i(0); i<double_body_contact_->getDim(); ++i)
        sp_->reaction_forces_[i] = reaction_force[i];
    sp_->reflected_reaction_force_ = wbdc_rotor_data_->reflected_reaction_force_;
}

void ConfigBodyCtrl::_jpos_task_setup(){
    // Calculate IK for a desired height and orientation.
    dynacore::Vector Q_cur = sp_->Q_;
    dynacore::Vector config_sol;

    double body_height_cmd;

    // Set Desired Orientation
    dynacore::Vect3 rpy_des;
    dynacore::Quaternion des_quat;
    rpy_des.setZero();
    dynacore::convert(rpy_des, des_quat);    

    dynacore::Vector pos_des(mercury::num_qdot); pos_des.setZero();
    dynacore::Vector vel_des(mercury::num_qdot); vel_des.setZero();
    dynacore::Vector acc_des(mercury::num_qdot); acc_des.setZero();

    if(b_set_height_target_) body_height_cmd = target_body_height_;
    else body_height_cmd = ini_body_height_;
    
    inv_kin_.getDoubleSupportLegConfig(Q_cur, des_quat, body_height_cmd, config_sol);
    sp_->body_pos_des_[2] = body_height_cmd;
    sp_->body_ori_des_ = des_quat;

    for (int i(0); i<mercury::num_act_joint; ++i){
        des_jpos_[i] = config_sol[mercury::num_virtual + i];
        pos_des[mercury::num_virtual + i] = des_jpos_[i];
        des_jvel_[i] = 0.;
    }

    //dynacore::pretty_print(Q_cur, std::cout, "Q_cur");
    //dynacore::pretty_print(config_sol, std::cout, "config_sol");
    // Maintain initial joint position desired
    jpos_task_->UpdateTask(&(pos_des), vel_des, acc_des);
    task_list_.push_back(jpos_task_);
}

void ConfigBodyCtrl::_double_body_contact_setup(){
    double_body_contact_->UpdateContactSpec();
    contact_list_.push_back(double_body_contact_);
}

void ConfigBodyCtrl::FirstVisit(){
    jpos_ini_ = sp_->Q_.segment(mercury::num_virtual, mercury::num_act_joint);
    ctrl_start_time_ = sp_->curr_time_;
    ini_body_height_ = sp_->Q_[mercury_joint::virtual_Z];
}

void ConfigBodyCtrl::LastVisit(){
}

bool ConfigBodyCtrl::EndOfPhase(){
    if(state_machine_time_ > end_time_){
        return true;
    }
    return false;
}
void ConfigBodyCtrl::CtrlInitialization(const std::string & setting_file_name){
    jpos_ini_ = sp_->Q_.segment(mercury::num_virtual, mercury::num_act_joint);

    ParamHandler handler(MercuryConfigPath + setting_file_name + ".yaml");

    std::vector<double> tmp_vec;
    // Feedback Gain
    handler.getVector("Kp", tmp_vec);
    for(int i(0); i<tmp_vec.size(); ++i){
        ((ConfigTask*)jpos_task_)->Kp_vec_[i] = tmp_vec[i];        

    }
    handler.getVector("Kd", tmp_vec);
    for(int i(0); i<tmp_vec.size(); ++i){
        ((ConfigTask*)jpos_task_)->Kd_vec_[i] = tmp_vec[i];
    }
}
