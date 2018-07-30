#include "TransitionConfigCtrl.hpp"
#include <Configuration.h>
#include <Valkyrie_Controller/Valkyrie_StateProvider.hpp>
#include <Valkyrie_Controller/TaskSet/ConfigTask.hpp>
#include <Valkyrie_Controller/ContactSet/DoubleContactBounding.hpp>
#include <Valkyrie_Controller/Valkyrie_DynaControl_Definition.h>
#include <WBDC/WBDC.hpp>
#include <Valkyrie/Valkyrie_Model.hpp>
#include <ParamHandler/ParamHandler.hpp>

TransitionConfigCtrl::TransitionConfigCtrl(RobotSystem* robot, int moving_foot, bool b_increase):
    Controller(robot),
    b_set_height_target_(false),
    moving_foot_(moving_foot),
    b_increase_(b_increase),
    end_time_(100.),
    ctrl_start_time_(0.),
    des_jpos_(valkyrie::num_act_joint),
    des_jvel_(valkyrie::num_act_joint)
{
    config_task_ = new ConfigTask();
    double_contact_ = new DoubleContactBounding(robot, moving_foot);
    std::vector<bool> act_list;
    act_list.resize(valkyrie::num_qdot, true);
    for(int i(0); i<valkyrie::num_virtual; ++i) act_list[i] = false;

    wbdc_ = new WBDC(act_list);
    wbdc_data_ = new WBDC_ExtraData();
    wbdc_data_->cost_weight = 
        dynacore::Vector::Constant(
                config_task_->getDim() + 
                double_contact_->getDim(), 1000.0);

    wbdc_data_->cost_weight.tail(double_contact_->getDim()) = 
        dynacore::Vector::Constant(double_contact_->getDim(), 1.0);
    wbdc_data_->cost_weight[config_task_->getDim() + 5]  = 0.001; // Fr_z
    wbdc_data_->cost_weight[config_task_->getDim() + 11]  = 0.001; // Fr_z

    sp_ = Valkyrie_StateProvider::getStateProvider();
    inv_kin_ = new Valkyrie_InvKinematics();
    printf("[Transition Controller] Constructed\n");
}

TransitionConfigCtrl::~TransitionConfigCtrl(){
    delete config_task_;
    delete double_contact_;

    delete wbdc_;
    delete wbdc_data_;
}

void TransitionConfigCtrl::OneStep(void* _cmd){
    _PreProcessing_Command();
    state_machine_time_ = sp_->curr_time_ - ctrl_start_time_;
    dynacore::Vector gamma;

    _double_contact_setup();
    _body_task_setup();
    _body_ctrl_wbdc(gamma);
    for(int i(0); i<valkyrie::num_act_joint; ++i){
        ((Valkyrie_Command*)_cmd)->jtorque_cmd[i] = gamma[i];
        ((Valkyrie_Command*)_cmd)->jpos_cmd[i] = des_jpos_[i];
        ((Valkyrie_Command*)_cmd)->jvel_cmd[i] = des_jvel_[i];
    }
    _PostProcessing_Command();
}

void TransitionConfigCtrl::_body_ctrl_wbdc(dynacore::Vector & gamma){
    wbdc_->UpdateSetting(A_, Ainv_, coriolis_, grav_);
    wbdc_->MakeTorque(task_list_, contact_list_, gamma, wbdc_data_);
}

void TransitionConfigCtrl::_body_task_setup(){
    dynacore::Vector pos_des(config_task_->getDim()); // not exact orientation task
    dynacore::Vector vel_des(config_task_->getDim());
    dynacore::Vector acc_des(config_task_->getDim());
    pos_des.setZero(); vel_des.setZero(); acc_des.setZero();

    // Body Height
    double target_height = ini_body_pos_[2];
    if(b_set_height_target_) target_height = des_body_height_;
    // Orientation
    dynacore::Vect3 rpy_des;
    dynacore::Quaternion quat_des;
    rpy_des.setZero();

    dynacore::convert(rpy_des, quat_des);
    pos_des[3] = quat_des.w();
    pos_des[4] = quat_des.x();
    pos_des[5] = quat_des.y();
    pos_des[6] = quat_des.z();

    dynacore::Vector config_sol;
    inv_kin_->getDoubleSupportLegConfig(sp_->Q_, quat_des, target_height, config_sol);
    for (int i(0); i<valkyrie::num_act_joint; ++i){
        pos_des[valkyrie::num_virtual + i] = config_sol[valkyrie::num_virtual + i];  
        des_jpos_[i] = pos_des[valkyrie::num_virtual + i];
        des_jvel_[i] = 0.;
    }
    config_task_->UpdateTask(&(pos_des), vel_des, acc_des);

    // Push back to task list
    task_list_.push_back(config_task_);
}

void TransitionConfigCtrl::_double_contact_setup(){
    if(b_increase_){
        //((DoubleContactBounding*)double_contact_)->setFrictionCoeff(0.05, 0.3);
        ((DoubleContactBounding*)double_contact_)->setFzUpperLimit(
            min_rf_z_ + state_machine_time_/end_time_ * (max_rf_z_ - min_rf_z_));
    } else {
        //((DoubleContactBounding*)double_contact_)->setFrictionCoeff(0.3, 0.4);
        ((DoubleContactBounding*)double_contact_)->setFzUpperLimit(
            max_rf_z_ - state_machine_time_/end_time_ * (max_rf_z_ - min_rf_z_));
    }

    double_contact_->UpdateContactSpec();
    contact_list_.push_back(double_contact_);
}

void TransitionConfigCtrl::FirstVisit(){
    // printf("[Transition] Start\n");
    ctrl_start_time_ = sp_->curr_time_;
}

void TransitionConfigCtrl::LastVisit(){
    // printf("[Transition] End\n");
}

bool TransitionConfigCtrl::EndOfPhase(){
    if(state_machine_time_ > end_time_){
        return true;
    }
    return false;
}
void TransitionConfigCtrl::CtrlInitialization(const std::string & setting_file_name){
    std::vector<double> tmp_vec;
    ParamHandler handler(ValkyrieConfigPath + setting_file_name + ".yaml");
    handler.getValue("max_rf_z", max_rf_z_);
    handler.getValue("min_rf_z", min_rf_z_);

    // Feedback Gain
    handler.getVector("Kp", tmp_vec);
    for(int i(0); i<tmp_vec.size(); ++i){
        ((ConfigTask*)config_task_)->Kp_vec_[i] = tmp_vec[i];
    }

    handler.getVector("Kd", tmp_vec);
    for(int i(0); i<tmp_vec.size(); ++i){
        ((ConfigTask*)config_task_)->Kd_vec_[i] = tmp_vec[i];
    }
}
