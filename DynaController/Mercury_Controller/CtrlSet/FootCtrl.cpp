#include "FootCtrl.hpp"
#include <Mercury_Controller/Mercury_StateProvider.hpp>
#include <Mercury_Controller/TaskSet/JPosTask.hpp>
#include <Mercury_Controller/TaskSet/LinkXYZTask.hpp>
#include <Mercury_Controller/ContactSet/FixedBodyContact.hpp>

#include <Mercury_Controller/Mercury_StateProvider.hpp>
#include <WBDC_Relax/WBDC_Relax.hpp>
#include <ParamHandler/ParamHandler.hpp>
#include <Mercury/Mercury_Model.hpp>
#include <Mercury/Mercury_Definition.h>

FootCtrl::FootCtrl(RobotSystem* robot, int swing_foot):Controller(robot),
    swing_foot_(swing_foot),
    end_time_(1000.0),
    ctrl_start_time_(0.),
    foot_pos_des_(3),
    foot_vel_des_(3),
    foot_acc_des_(3)
{
  foot_pos_des_.setZero();
  foot_vel_des_.setZero();
  foot_acc_des_.setZero();

  foot_pos_.setZero();
  foot_vel_.setZero();



    amp_.resize(3, 0.);
    freq_.resize(3, 0.);

    foot_pos_ini_.setZero();
    foot_task_ = new LinkXYZTask(robot, swing_foot_);
    jpos_task_ = new JPosTask();
    fixed_body_contact_ = new FixedBodyContact(robot);

    std::vector<bool> act_list;
    act_list.resize(mercury::num_qdot, true);
    for(int i(0); i<mercury::num_virtual; ++i) act_list[i] = false;

    wbdc_ = new WBDC_Relax(act_list);
    wbdc_data_ = new WBDC_Relax_ExtraData();

    wbdc_data_->tau_min = dynacore::Vector(mercury::num_act_joint);
    wbdc_data_->tau_max = dynacore::Vector(mercury::num_act_joint);
    for(int i(0); i<mercury::num_act_joint; ++i){
        wbdc_data_->tau_max[i] = 50.0;
        wbdc_data_->tau_min[i] = -50.0;
    }

    sp_ = Mercury_StateProvider::getStateProvider();
}

FootCtrl::~FootCtrl(){
    delete foot_task_;
    delete jpos_task_;
    delete fixed_body_contact_;
}

void FootCtrl::OneStep(dynacore::Vector & gamma){
    _PreProcessing_Command();
    state_machine_time_ = sp_->curr_time_ - ctrl_start_time_;

    gamma = dynacore::Vector::Zero(mercury::num_act_joint * 2);
    _fixed_body_contact_setup();
    _foot_pos_task_setup();
    _jpos_task_setup();
    _foot_pos_ctrl(gamma);

    _PostProcessing_Command();
}

void FootCtrl::_foot_pos_ctrl(dynacore::Vector & gamma){
    dynacore::Vector jtorque_cmd(mercury::num_act_joint);

    wbdc_->UpdateSetting(A_, Ainv_, coriolis_, grav_);
    wbdc_->MakeTorque(task_list_, contact_list_, jtorque_cmd, wbdc_data_);

    gamma.head(mercury::num_act_joint) = jtorque_cmd;

    dynacore::Matrix A_rotor = A_;
    for(int i(0); i<mercury::num_act_joint; ++i) {
        A_rotor(i+mercury::num_virtual,i + mercury::num_virtual) 
            += sp_->rotor_inertia_[i];
    }
    dynacore::Matrix A_rotor_inv;

    dynacore::pseudoInverse(A_rotor, 0.00001, A_rotor_inv, 0);
    wbdc_->UpdateSetting(A_rotor, A_rotor_inv, coriolis_, grav_);
    wbdc_->MakeTorque(task_list_, contact_list_, jtorque_cmd, wbdc_data_);
    gamma.tail(mercury::num_act_joint) = jtorque_cmd;
}

void FootCtrl::_foot_pos_task_setup(){
    foot_acc_des_.setZero();
    foot_vel_des_.setZero();
    foot_pos_des_.setZero();

    double omega;
    for(int i(0); i<3; ++i){
        omega = 2. * M_PI * freq_[i];
        foot_pos_des_[i] = 
            foot_pos_ini_[i] + amp_[i] * sin(omega * state_machine_time_ + phase_[i]);
        foot_vel_des_[i] = 
            amp_[i] * omega * cos(omega * state_machine_time_ + phase_[i]);
        foot_acc_des_[i] = 
            -amp_[i] * omega * omega * sin(omega * state_machine_time_ + phase_[i]);
    }
    foot_task_->UpdateTask(&foot_pos_des_, foot_vel_des_, foot_acc_des_);
    std::vector<bool> relaxed_op(foot_task_->getDim(), false);
    foot_task_->setRelaxedOpCtrl(relaxed_op);
    task_list_.push_back(foot_task_);

    // For Save
    robot_sys_->getPos(swing_foot_, foot_pos_);
    robot_sys_->getLinearVel(swing_foot_, foot_vel_);
}

void FootCtrl::_jpos_task_setup(){
    sp_->jpos_des_ = jpos_ini_;
    jpos_task_->UpdateTask(&(sp_->jpos_des_),
            dynacore::Vector::Zero(mercury::num_act_joint),
            dynacore::Vector::Zero(mercury::num_act_joint));

    std::vector<bool> relaxed_op(jpos_task_->getDim(), false);
    relaxed_op.resize(jpos_task_->getDim(), false);
    jpos_task_->setRelaxedOpCtrl(relaxed_op);
    task_list_.push_back(jpos_task_);
}

void FootCtrl::_fixed_body_contact_setup(){
    fixed_body_contact_->UpdateContactSpec();
    contact_list_.push_back(fixed_body_contact_);
    wbdc_data_->cost_weight = dynacore::Vector::Zero(fixed_body_contact_->getDim());

    for(int i(0); i<fixed_body_contact_->getDim(); ++i){
        wbdc_data_->cost_weight[i] = 1.;
    }
}

void FootCtrl::FirstVisit(){
    jpos_ini_ = sp_->Q_.segment(mercury::num_virtual, mercury::num_act_joint);
    robot_sys_->getPos(swing_foot_, foot_pos_ini_);
    ctrl_start_time_ = sp_->curr_time_;
}

void FootCtrl::LastVisit(){
}

bool FootCtrl::EndOfPhase(){
    if(state_machine_time_ > end_time_){
        return true;
    }
    return false;
}
void FootCtrl::CtrlInitialization(const std::string & setting_file_name){
    ParamHandler handler(MercuryConfigPath + setting_file_name + ".yaml");

    std::vector<double> tmp_vec;
    handler.getVector("foot_Kp", tmp_vec);
    for(int i(0); i<tmp_vec.size(); ++i)
        ((LinkXYZTask*)foot_task_)->Kp_vec_[i] = tmp_vec[i];

    handler.getVector("foot_Kd", tmp_vec);
    for(int i(0); i<tmp_vec.size(); ++i)
        ((LinkXYZTask*)foot_task_)->Kd_vec_[i] = tmp_vec[i];

    handler.getVector("jpos_Kp", tmp_vec);
    for(int i(0); i<tmp_vec.size(); ++i)
        ((JPosTask*)jpos_task_)->Kp_vec_[i] = tmp_vec[i];

    handler.getVector("jpos_Kd", tmp_vec);
    for(int i(0); i<tmp_vec.size(); ++i)
        ((JPosTask*)jpos_task_)->Kd_vec_[i] = tmp_vec[i];
    //printf("[Foot Control Parameter Set!\n");
}
