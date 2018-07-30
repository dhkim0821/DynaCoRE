#include "ContactTransConfigCtrl.hpp"
#include <Configuration.h>
#include <Atlas_Controller/Atlas_StateProvider.hpp>
#include <Atlas_Controller/TaskSet/ConfigTask.hpp>
#include <Atlas_Controller/ContactSet/DoubleTransitionContact.hpp>
#include <Atlas_Controller/Atlas_DynaControl_Definition.h>
#include <Atlas/Atlas_Model.hpp>
#include <WBDC/WBDC.hpp>
#include <ParamHandler/ParamHandler.hpp>
#include <Atlas_Controller/Atlas_InvKinematics.hpp>

ContactTransConfigCtrl::ContactTransConfigCtrl(RobotSystem* robot):
    Controller(robot),
    b_set_height_target_(false),
    end_time_(100.),
    body_pos_ini_(4),
	des_jpos_(atlas::num_act_joint),
	des_jvel_(atlas::num_act_joint),
    config_des_(atlas::num_qdot)
{
    config_des_.setZero();
    body_task_ = new ConfigTask();
    double_contact_ = new DoubleTransitionContact(robot);

    std::vector<bool> act_list;
    act_list.resize(atlas::num_qdot, true);
    for(int i(0); i<atlas::num_virtual; ++i) act_list[i] = false;

    wbdc_ = new WBDC(act_list);
    wbdc_data_ = new WBDC_ExtraData();
    wbdc_data_->cost_weight = 
        dynacore::Vector::Constant(
                body_task_->getDim() + 
                double_contact_->getDim(), 100.0);

    //wbdc_data_->cost_weight[0] = 0.0001; // X
    //wbdc_data_->cost_weight[1] = 0.0001; // Y
    //wbdc_data_->cost_weight[5] = 0.0001; // Yaw
    
    wbdc_data_->cost_weight.tail(double_contact_->getDim()) = 
        dynacore::Vector::Constant(double_contact_->getDim(), 1.0);
    wbdc_data_->cost_weight[body_task_->getDim() + 2]  = 0.001; // Fr_z
    wbdc_data_->cost_weight[body_task_->getDim() + 5]  = 0.001; // Fr_z

   sp_ = Atlas_StateProvider::getStateProvider();

   inv_kin_ = new Atlas_InvKinematics();

    printf("[Contact Transition Body Ctrl] Constructed\n");
}

ContactTransConfigCtrl::~ContactTransConfigCtrl(){
    delete body_task_;
    delete double_contact_;
    delete wbdc_;
    delete wbdc_data_;

    delete inv_kin_;
}

void ContactTransConfigCtrl::OneStep(void* _cmd){
    _PreProcessing_Command();
	state_machine_time_ = sp_->curr_time_ - ctrl_start_time_;

	dynacore::Vector gamma;
	_double_contact_setup();
	_body_task_setup();
	_body_ctrl_wbdc(gamma);
	for(int i(0); i<atlas::num_act_joint; ++i){
		((Atlas_Command*)_cmd)->jtorque_cmd[i] = gamma[i];
		((Atlas_Command*)_cmd)->jpos_cmd[i] = des_jpos_[i];
		((Atlas_Command*)_cmd)->jvel_cmd[i] = des_jvel_[i];
	}
    _PostProcessing_Command();
}

void ContactTransConfigCtrl::_body_ctrl_wbdc(dynacore::Vector & gamma){
    double ramp = (state_machine_time_)/(end_time_*0.5);
    if( state_machine_time_ > end_time_* 0.5 ) ramp = 1.;
    dynacore::Vector ramp_grav = ramp * grav_;

    wbdc_->UpdateSetting(A_, Ainv_, coriolis_, ramp_grav);
    wbdc_->MakeTorque(task_list_, contact_list_, gamma, wbdc_data_);
}

void ContactTransConfigCtrl::_body_task_setup(){
    double body_height_cmd;
    dynacore::Vector vel_des(body_task_->getDim());
    dynacore::Vector acc_des(body_task_->getDim());
    vel_des.setZero(); acc_des.setZero();

    // Body Pos
    if(b_set_height_target_){
        body_height_cmd = dynacore::smooth_changing(ini_body_pos_[2], des_body_height_, end_time_, state_machine_time_);
    }else{
        printf("[Warning] The body height is not specified\n");
    }
    // Orientation
    dynacore::Vect3 rpy_des;
    dynacore::Quaternion des_quat;
    rpy_des.setZero();
    dynacore::convert(rpy_des, des_quat);
    dynacore::Vector config_sol; 

    inv_kin_->getDoubleSupportLegConfig(sp_->Q_, des_quat, body_height_cmd, config_sol);

    for (int i(0); i<atlas::num_act_joint; ++i){
        des_jpos_[i] = config_sol[atlas::num_virtual + i];
        config_des_[atlas::num_virtual + i] = des_jpos_[i];
        des_jvel_[i] = 0.;
    }
    body_task_->UpdateTask(&(config_des_), vel_des, acc_des);

   // Push back to task list
    task_list_.push_back(body_task_);
}

void ContactTransConfigCtrl::_double_contact_setup(){
    ((DoubleTransitionContact*)double_contact_)->
        setFzUpperLimit(min_rf_z_ + 
                state_machine_time_/end_time_ * (max_rf_z_ - min_rf_z_));
    double_contact_->UpdateContactSpec();
    contact_list_.push_back(double_contact_);
}

void ContactTransConfigCtrl::FirstVisit(){
    ini_body_pos_ = sp_->Q_.head(3);
    ctrl_start_time_ = sp_->curr_time_;
}

void ContactTransConfigCtrl::LastVisit(){
    // printf("[ContactTransBody] End\n");
}

bool ContactTransConfigCtrl::EndOfPhase(){
    if(state_machine_time_ > end_time_){
        return true;
    }
    return false;
}
void ContactTransConfigCtrl::CtrlInitialization(const std::string & setting_file_name){
    ini_body_pos_ = sp_->Q_.head(3);

    ParamHandler handler(AtlasConfigPath + setting_file_name + ".yaml");
    handler.getValue("max_rf_z", max_rf_z_);
    handler.getValue("min_rf_z", min_rf_z_);
    // Feedback Gain
    std::vector<double> tmp_vec;
    handler.getVector("Kp", tmp_vec);
    for(int i(0); i<tmp_vec.size(); ++i){
        ((ConfigTask*)body_task_)->Kp_vec_[i] = tmp_vec[i];
    }
    handler.getVector("Kd", tmp_vec);
    for(int i(0); i<tmp_vec.size(); ++i){
        ((ConfigTask*)body_task_)->Kd_vec_[i] = tmp_vec[i];
    }
}
