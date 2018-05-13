#include "BodyJPosCtrl.hpp"
#include <Mercury_Controller/Mercury_StateProvider.hpp>
#include <Mercury_Controller/TaskSet/JPosTask.hpp>
#include <Mercury_Controller/TaskSet/ConfigTask.hpp>
#include <Mercury_Controller/ContactSet/FixedBodyContact.hpp>
#include <Mercury_Controller/ContactSet/DoubleContact.hpp>
#include <Mercury_Controller/Mercury_StateProvider.hpp>
#include <WBDC_Relax/WBDC_Relax.hpp>
#include <WBDC_Rotor/WBDC_Rotor.hpp>
#include <ParamHandler/ParamHandler.hpp>
#include <Utils/DataManager.hpp>
#include <Mercury/Mercury_Definition.h>

BodyJPosCtrl::BodyJPosCtrl(RobotSystem* robot):Controller(robot),
    end_time_(1000.0),
    b_jpos_set_(false),
    ctrl_start_time_(0.)
{
    set_jpos_.resize(mercury::num_act_joint, 0.);
    amp_.resize(mercury::num_act_joint, 0.);
    freq_.resize(mercury::num_act_joint, 0.);
    phase_.resize(mercury::num_act_joint, 0.);

    jpos_task_ = new ConfigTask();
    double_body_contact_ = new DoubleContact(robot);
//    double_body_contact_ = new FixedBodyContact(robot);
    std::vector<bool> act_list;
    act_list.resize(mercury::num_qdot, true);
    for(int i(0); i<mercury::num_virtual; ++i) act_list[i] = false;
    
    wbdc_rotor_ = new WBDC_Rotor(act_list);
    wbdc_rotor_data_ = new WBDC_Rotor_ExtraData();
    wbdc_rotor_data_->A_rotor =  dynacore::Matrix::Zero(mercury::num_qdot, mercury::num_qdot);
   
    wbdc_rotor_data_->cost_weight = 
    dynacore::Vector::Constant(
        jpos_task_->getDim() + double_body_contact_->getDim(), 100.0);

    // for(int i(0); i<mercury::num_virtual; ++i){
    //     wbdc_rotor_data_->cost_weight[i] = 0.00001;
    // }
    wbdc_rotor_data_->cost_weight[0] = 200;    
    wbdc_rotor_data_->cost_weight[1] = 200;    
    wbdc_rotor_data_->cost_weight[2] = 200;    

    wbdc_rotor_data_->cost_weight.tail(double_body_contact_->getDim()) = 
        dynacore::Vector::Constant(double_body_contact_->getDim(), 100);

    wbdc_rotor_data_->cost_weight[jpos_task_->getDim() + 2] = 0.001;
    wbdc_rotor_data_->cost_weight[jpos_task_->getDim() + 5] = 0.001;

    sp_ = Mercury_StateProvider::getStateProvider();

    printf("[Joint Position Control] Constructed\n");
}

BodyJPosCtrl::~BodyJPosCtrl(){
    delete jpos_task_;
    delete double_body_contact_;

    delete wbdc_rotor_;
    delete wbdc_rotor_data_;
}

void BodyJPosCtrl::OneStep(dynacore::Vector & gamma){
    _PreProcessing_Command();
    state_machine_time_ = sp_->curr_time_ - ctrl_start_time_;

    gamma = dynacore::Vector::Zero(mercury::num_act_joint * 2);
    _double_body_contact_setup();
    _jpos_task_setup();
    _jpos_ctrl_wbdc_rotor(gamma);

    //dynacore::pretty_print( gamma, std::cout, "gamma");

    _PostProcessing_Command();
}

void BodyJPosCtrl::_jpos_ctrl_wbdc_rotor(dynacore::Vector & gamma){
    dynacore::Vector fb_cmd = dynacore::Vector::Zero(mercury::num_act_joint);
    for (int i(0); i<mercury::num_act_joint; ++i){
        wbdc_rotor_data_->A_rotor(i + mercury::num_virtual, i + mercury::num_virtual)
            = sp_->rotor_inertia_[i];
    }
    wbdc_rotor_->UpdateSetting(A_, Ainv_, coriolis_, grav_);
    wbdc_rotor_->MakeTorque(task_list_, contact_list_, fb_cmd, wbdc_rotor_data_);

    gamma.head(mercury::num_act_joint) = fb_cmd;
    gamma.tail(mercury::num_act_joint) = wbdc_rotor_data_->cmd_ff;

    sp_->qddot_cmd_ = wbdc_rotor_data_->result_qddot_;
    dynacore::Vector reaction_force = 
             (wbdc_rotor_data_->opt_result_).tail(double_body_contact_->getDim());
    for(int i(0); i<double_body_contact_->getDim(); ++i)
        sp_->reaction_forces_[i] = reaction_force[i];
    sp_->reflected_reaction_force_ = wbdc_rotor_data_->reflected_reaction_force_;
}

void BodyJPosCtrl::_jpos_task_setup(){
    // Calculate IK for a desired height and orientation.
    dynacore::Vector Q_cur = sp_->Q_;
    dynacore::Vector config_sol;
    //dynacore::pretty_print(Q_cur, std::cout, "Q cur");   

    // Set Frequency
    double frequency = 1; //Hz
    double omega = 2. * M_PI * frequency;

    // Set Desired height
    double des_height = 0.85;//0.853;// 0.852689 is the current height
    //double des_height = 0.853;//0.853;// 0.852689 is the current height
    //double des_height = 0.75;//0.853;// 0.852689 is the current height
    //double des_height = 0.8 + 0.05*sin(omega * state_machine_time_);

    // Set Desired Orientation
    dynacore::Vect3 rpy_des;
    dynacore::Quaternion des_quat;
    rpy_des.setZero();
    //rpy_des[1] = 0.5;
    dynacore::convert(rpy_des, des_quat);    
    //rpy_des[1] = 0.25 + 0.25*sin(omega * state_machine_time_);

    dynacore::Vector jpos_des(mercury::num_qdot); jpos_des.setZero();
    dynacore::Vector jvel_des(mercury::num_qdot); jvel_des.setZero();
    dynacore::Vector jacc_des(mercury::num_qdot); jacc_des.setZero();

    // Maintain initial joint position desired
    for (int i(0); i<mercury::num_act_joint; ++i){
        //jpos_des[mercury::num_virtual + i] = jpos_ini_[i];//ini_jpos;
        jpos_des[mercury::num_virtual + i] = config_sol[mercury::num_virtual + i];
    }
    inv_kin_.getDoubleSupportLegConfig(Q_cur, des_quat, des_height, config_sol);

    dynacore::pretty_print(Q_cur, std::cout, "Q_cur");
    dynacore::pretty_print(config_sol, std::cout, "config_sol");
    // Maintain initial joint position desired
    for (int i(0); i<mercury::num_act_joint; ++i){
        //jpos_des[mercury::num_virtual + i] = jpos_ini_[i];//ini_jpos;
        jpos_des[mercury::num_virtual + i] = config_sol[mercury::num_virtual + i];  
        sp_->jpos_des_[i] = jpos_des[mercury::num_virtual + i];
    }

    jpos_task_->UpdateTask(&(jpos_des), jvel_des, jacc_des);
    task_list_.push_back(jpos_task_);
}

void BodyJPosCtrl::_double_body_contact_setup(){
    double_body_contact_->UpdateContactSpec();
    contact_list_.push_back(double_body_contact_);
}

void BodyJPosCtrl::FirstVisit(){
    jpos_ini_ = sp_->Q_.segment(mercury::num_virtual, mercury::num_act_joint);
    ctrl_start_time_ = sp_->curr_time_;
}

void BodyJPosCtrl::LastVisit(){
}

bool BodyJPosCtrl::EndOfPhase(){
    if(state_machine_time_ > end_time_){
        return true;
    }
    return false;
}
void BodyJPosCtrl::CtrlInitialization(const std::string & setting_file_name){
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
