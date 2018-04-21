#include "TransitionCtrl.hpp"
#include <Configuration.h>
#include <Mercury_Controller/Mercury_StateProvider.hpp>
#include <Mercury_Controller/TaskSet/CoMBodyOriTask.hpp>
#include <Mercury_Controller/ContactSet/DoubleContactBounding.hpp>
#include <WBDC_Relax/WBDC_Relax.hpp>
#include <Mercury/Mercury_Model.hpp>
#include <Mercury/Mercury_Definition.h>
#include <ParamHandler/ParamHandler.hpp>

TransitionCtrl::TransitionCtrl(RobotSystem* robot, int moving_foot, bool b_increase):
    Controller(robot),
    b_set_height_target_(false),
    moving_foot_(moving_foot),
    b_increase_(b_increase),
    end_time_(100.),
    ctrl_start_time_(0.)
{
    body_task_ = new CoMBodyOriTask(robot);
    double_contact_ = new DoubleContactBounding(robot, moving_foot);
    std::vector<bool> act_list;
    act_list.resize(mercury::num_qdot, true);
    for(int i(0); i<mercury::num_virtual; ++i) act_list[i] = false;

    wbdc_ = new WBDC_Relax(act_list);

    wbdc_data_ = new WBDC_Relax_ExtraData();
    wbdc_data_->tau_min = dynacore::Vector(mercury::num_act_joint);
    wbdc_data_->tau_max = dynacore::Vector(mercury::num_act_joint);

    for(int i(0); i<mercury::num_act_joint; ++i){
        wbdc_data_->tau_max[i] = 100.0;
        wbdc_data_->tau_min[i] = -100.0;
    }
    sp_ = Mercury_StateProvider::getStateProvider();
     printf("[Transition Controller] Constructed\n");
}

TransitionCtrl::~TransitionCtrl(){
    delete body_task_;
    delete double_contact_;
    delete wbdc_;
}

void TransitionCtrl::OneStep(dynacore::Vector & gamma){
    _PreProcessing_Command();
    state_machine_time_ = sp_->curr_time_ - ctrl_start_time_;
    gamma.setZero();
    _double_contact_setup();
    _body_task_setup();
    _body_ctrl(gamma);

    _PostProcessing_Command();
}

void TransitionCtrl::_body_ctrl(dynacore::Vector & gamma){
    wbdc_->UpdateSetting(A_, Ainv_, coriolis_, grav_);
    wbdc_->MakeTorque(task_list_, contact_list_, gamma, wbdc_data_);

    for(int i(0); i<6; ++i)
        sp_->reaction_forces_[i] = wbdc_data_->opt_result_[i];
}

void TransitionCtrl::_body_task_setup(){
    dynacore::Vector pos_des(3 + 4);
    dynacore::Vector vel_des(body_task_->getDim());
    dynacore::Vector acc_des(body_task_->getDim());
    pos_des.setZero(); vel_des.setZero(); acc_des.setZero();

    // CoM Pos
    pos_des.head(3) = ini_com_pos_;
    if(b_set_height_target_) pos_des[2] = des_com_height_;
    // Orientation
    dynacore::Vect3 rpy_des;
    dynacore::Quaternion quat_des;
    rpy_des.setZero();

    dynacore::convert(rpy_des, quat_des);
    pos_des[3] = quat_des.w();
    pos_des[4] = quat_des.x();
    pos_des[5] = quat_des.y();
    pos_des[6] = quat_des.z();

    body_task_->UpdateTask(&(pos_des), vel_des, acc_des);

    // set relaxed op direction
    // cost weight setup
    // bool b_height_relax(false);
    bool b_height_relax(true);
    if(b_height_relax){
        std::vector<bool> relaxed_op(body_task_->getDim(), true);
        body_task_->setRelaxedOpCtrl(relaxed_op);

        int prev_size(wbdc_data_->cost_weight.rows());
        wbdc_data_->cost_weight.conservativeResize(prev_size + body_task_->getDim());
        wbdc_data_->cost_weight[prev_size] = 0.0001;
        wbdc_data_->cost_weight[prev_size+1] = 0.0001;
        wbdc_data_->cost_weight[prev_size+2] = 10.;
        wbdc_data_->cost_weight[prev_size+3] = 10.;
        wbdc_data_->cost_weight[prev_size+4] = 10.;
        wbdc_data_->cost_weight[prev_size+5] = 1.;
    }

    // Push back to task list
    task_list_.push_back(body_task_);
}

void TransitionCtrl::_double_contact_setup(){
    if(b_increase_){
        ((DoubleContactBounding*)double_contact_)->setFzUpperLimit(min_rf_z_ + state_machine_time_/end_time_ * (max_rf_z_ - min_rf_z_));
    } else {
        ((DoubleContactBounding*)double_contact_)->setFzUpperLimit(max_rf_z_ - state_machine_time_/end_time_ * (max_rf_z_ - min_rf_z_));
    }
    // ((DoubleContactBounding*)double_contact_)->setFzUpperLimit(1000.);

    double_contact_->UpdateContactSpec();

    contact_list_.push_back(double_contact_);

    wbdc_data_->cost_weight = dynacore::Vector::Zero(double_contact_->getDim());
    for(int i(0); i<double_contact_->getDim(); ++i){
        wbdc_data_->cost_weight[i] = 1.;
    }
    wbdc_data_->cost_weight[2] = 0.0001;
    wbdc_data_->cost_weight[5] = 0.0001;
}

void TransitionCtrl::FirstVisit(){
    // printf("[Transition] Start\n");
    ctrl_start_time_ = sp_->curr_time_;
}

void TransitionCtrl::LastVisit(){
    // printf("[Transition] End\n");
}

bool TransitionCtrl::EndOfPhase(){
    if(state_machine_time_ > end_time_){
        return true;
    }
    return false;
}
void TransitionCtrl::CtrlInitialization(const std::string & setting_file_name){
    robot_sys_->getCoMPosition(ini_com_pos_);
    std::vector<double> tmp_vec;

    ParamHandler handler(MercuryConfigPath + setting_file_name + ".yaml");
    handler.getValue("max_rf_z", max_rf_z_);
    handler.getValue("min_rf_z", min_rf_z_);

    // Feedback Gain
    handler.getVector("Kp", tmp_vec);
    for(int i(0); i<tmp_vec.size(); ++i){
        ((CoMBodyOriTask*)body_task_)->Kp_vec_[i] = tmp_vec[i];
    }
    handler.getVector("Kd", tmp_vec);
    for(int i(0); i<tmp_vec.size(); ++i){
        ((CoMBodyOriTask*)body_task_)->Kd_vec_[i] = tmp_vec[i];
    }

}
