#include "BodyCtrl.hpp"
#include <DracoBip_Controller/DracoBip_StateProvider.hpp>
#include <DracoBip_Controller/TaskSet/BodyTask.hpp>
#include <DracoBip_Controller/ContactSet/DoubleContact.hpp>
#include <DracoBip_Controller/DracoBip_StateProvider.hpp>
#include <WBLC/KinWBC.hpp>
#include <WBLC/WBLC.hpp>
#include <ParamHandler/ParamHandler.hpp>
#include <Utils/DataManager.hpp>
#include <DracoBip_Controller/DracoBip_DynaCtrl_Definition.h>

BodyCtrl::BodyCtrl(RobotSystem* robot):Controller(robot),
    end_time_(1000.0),
    ctrl_start_time_(0.),
    b_set_height_target_(false),
    des_jpos_(dracobip::num_act_joint),
    des_jvel_(dracobip::num_act_joint),
    des_jacc_(dracobip::num_act_joint),
    Kp_(dracobip::num_act_joint),
    Kd_(dracobip::num_act_joint)
{
    std::vector<bool> act_list;
    act_list.resize(dracobip::num_qdot, true);
    for(int i(0); i<dracobip::num_virtual; ++i) act_list[i] = false;

    wblc_ = new WBLC(act_list);
    wblc_data_ = new WBLC_ExtraData();
    body_task_ = new BodyTask(robot);
    double_contact_ = new DoubleContact(robot);

    wblc_data_->W_qddot_ = dynacore::Vector::Constant(dracobip::num_qdot, 100.0);
    wblc_data_->W_rf_ = dynacore::Vector::Constant(double_contact_->getDim(), 0.1);
    wblc_data_->W_xddot_ = dynacore::Vector::Constant(double_contact_->getDim(), 1000.0);
    wblc_data_->W_rf_[4] = 0.01;
    wblc_data_->W_rf_[9] = 0.01;

    // torque limit default setting
    wblc_data_->tau_min_ = dynacore::Vector::Constant(dracobip::num_act_joint, -50.);
    wblc_data_->tau_max_ = dynacore::Vector::Constant(dracobip::num_act_joint, 50.);


    sp_ = DracoBip_StateProvider::getStateProvider();

    printf("[Body Control] Constructed\n");
}

BodyCtrl::~BodyCtrl(){
    delete body_task_;
    delete double_contact_;
    delete wblc_;
    delete wblc_data_;
}

void BodyCtrl::OneStep(void* _cmd){
    _PreProcessing_Command();
    state_machine_time_ = sp_->curr_time_ - ctrl_start_time_;

    dynacore::Vector gamma = dynacore::Vector::Zero(dracobip::num_act_joint);
    _double_contact_setup();
    _body_task_setup();
    _compute_torque_wblc(gamma);

    for(int i(0); i<dracobip::num_act_joint; ++i){
        ((DracoBip_Command*)_cmd)->jtorque_cmd[i] = gamma[i];
        ((DracoBip_Command*)_cmd)->jpos_cmd[i] = des_jpos_[i];
        ((DracoBip_Command*)_cmd)->jvel_cmd[i] = des_jvel_[i];
    }
    _PostProcessing_Command();
}

void BodyCtrl::_compute_torque_wblc(dynacore::Vector & gamma){
    // WBLC
    dynacore::Matrix A_rotor = A_;
    for (int i(0); i<dracobip::num_act_joint; ++i){
        A_rotor(i + dracobip::num_virtual, i + dracobip::num_virtual)
            += sp_->rotor_inertia_[i];
    }
    dynacore::Matrix A_rotor_inv = A_rotor.inverse();

    std::vector<Task*> dummy_task_list_;

    wblc_->UpdateSetting(A_rotor, A_rotor_inv, coriolis_, grav_);
    dynacore::Vector des_jacc_cmd = des_jacc_ 
        + Kp_.cwiseProduct(des_jpos_ - 
                sp_->Q_.segment(dracobip::num_virtual, dracobip::num_act_joint))
        + Kd_.cwiseProduct(des_jvel_ - sp_->Qdot_.tail(dracobip::num_act_joint));

    wblc_->MakeWBLC_Torque(
            des_jacc_cmd, contact_list_,
            gamma, wblc_data_);

    sp_->qddot_cmd_ = wblc_data_->qddot_;
    sp_->reaction_forces_ = wblc_data_->Fr_;
}

void BodyCtrl::_body_task_setup(){
    des_jpos_ = jpos_ini_;
    des_jvel_.setZero();
    des_jacc_.setZero();
    // Calculate IK for a desired height and orientation.
    dynacore::Vector Q_cur = sp_->Q_;
    dynacore::Vector config_sol;

    double body_height_cmd;

    // Set Desired Orientation
    dynacore::Vect3 rpy_des;
    dynacore::Quaternion des_quat;
    rpy_des.setZero();

    dynacore::convert(rpy_des, des_quat);    

    dynacore::Vector pos_des(7); pos_des.setZero();
    dynacore::Vector vel_des(6); vel_des.setZero();
    dynacore::Vector acc_des(6); acc_des.setZero();

    if(b_set_height_target_) body_height_cmd = target_body_height_;
    else body_height_cmd = ini_body_height_;

    //dynacore::pretty_print(Q_cur, std::cout, "Q_cur");
    //dynacore::pretty_print(config_sol, std::cout, "config_sol");
    // Maintain initial joint position desired
    // body_task_->UpdateTask(&(pos_des), vel_des, acc_des);
    // task_list_.push_back(body_task_);
}

void BodyCtrl::_double_contact_setup(){
    double_contact_->UpdateContactSpec();
    contact_list_.push_back(double_contact_);
}

void BodyCtrl::FirstVisit(){
    jpos_ini_ = sp_->Q_.segment(dracobip::num_virtual, dracobip::num_act_joint);
    ctrl_start_time_ = sp_->curr_time_;
    ini_body_height_ = sp_->Q_[dracobip_joint::virtual_Z];
}

void BodyCtrl::LastVisit(){  }

bool BodyCtrl::EndOfPhase(){
    if(state_machine_time_ > end_time_){
        return true;
    }
    return false;
}
void BodyCtrl::CtrlInitialization(const std::string & setting_file_name){
    jpos_ini_ = sp_->Q_.segment(dracobip::num_virtual, dracobip::num_act_joint);
    ParamHandler handler(DracoBipConfigPath + setting_file_name + ".yaml");

    std::vector<double> tmp_vec;
    // Feedback Gain
    handler.getVector("Kp", tmp_vec);
    for(int i(0); i<tmp_vec.size(); ++i){
        Kp_[i] = tmp_vec[i];
    }
    handler.getVector("Kd", tmp_vec);
    for(int i(0); i<tmp_vec.size(); ++i){
        Kd_[i] = tmp_vec[i];
    }
}
