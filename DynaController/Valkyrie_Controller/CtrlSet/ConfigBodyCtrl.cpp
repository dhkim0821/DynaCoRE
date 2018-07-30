#include "ConfigBodyCtrl.hpp"
#include <Valkyrie_Controller/Valkyrie_StateProvider.hpp>
#include <Valkyrie_Controller/TaskSet/ConfigTask.hpp>
#include <Valkyrie_Controller/ContactSet/DoubleContact.hpp>
#include <Valkyrie_Controller/Valkyrie_StateProvider.hpp>
#include <WBDC/WBDC.hpp>
#include <ParamHandler/ParamHandler.hpp>
#include <Utils/DataManager.hpp>
#include <Valkyrie_Controller/Valkyrie_DynaControl_Definition.h>

ConfigBodyCtrl::ConfigBodyCtrl(RobotSystem* robot):Controller(robot),
    end_time_(1000.0),
    ctrl_start_time_(0.),
    b_set_height_target_(false),
    des_jpos_(valkyrie::num_act_joint),
    des_jvel_(valkyrie::num_act_joint)
{
    jpos_task_ = new ConfigTask();
    double_body_contact_ = new DoubleContact(robot);
    
    std::vector<bool> act_list;
    act_list.resize(valkyrie::num_qdot, true);
    for(int i(0); i<valkyrie::num_virtual; ++i) act_list[i] = false;
    
    wbdc_ = new WBDC(act_list);
    wbdc_data_ = new WBDC_ExtraData();
   
    wbdc_data_->cost_weight = 
    dynacore::Vector::Constant(
        jpos_task_->getDim() + double_body_contact_->getDim(), 1000.0);

    // wbdc_data_->cost_weight[0] = 10;    
    // wbdc_data_->cost_weight[1] = 10;    
    // wbdc_data_->cost_weight[2] = 200;    

    wbdc_data_->cost_weight.tail(double_body_contact_->getDim()) = 
        dynacore::Vector::Constant(double_body_contact_->getDim(), 1.);

    wbdc_data_->cost_weight[jpos_task_->getDim() + 5] = 0.001;
    wbdc_data_->cost_weight[jpos_task_->getDim() + 11] = 0.001;

    sp_ = Valkyrie_StateProvider::getStateProvider();

    inv_kin_ = new Valkyrie_InvKinematics();
    printf("[Config Body Control] Constructed\n");
}

ConfigBodyCtrl::~ConfigBodyCtrl(){
    delete jpos_task_;
    delete double_body_contact_;
    delete wbdc_;
    delete wbdc_data_;
}

void ConfigBodyCtrl::OneStep(void* _cmd){
    _PreProcessing_Command();
    state_machine_time_ = sp_->curr_time_ - ctrl_start_time_;

    dynacore::Vector gamma;
    _double_body_contact_setup();
    _jpos_task_setup();
    _jpos_ctrl_wbdc(gamma);

    for(int i(0); i<valkyrie::num_act_joint; ++i){
        ((Valkyrie_Command*)_cmd)->jtorque_cmd[i] = gamma[i];
        ((Valkyrie_Command*)_cmd)->jpos_cmd[i] = des_jpos_[i];
        ((Valkyrie_Command*)_cmd)->jvel_cmd[i] = des_jvel_[i];
    }

    _PostProcessing_Command();
}

void ConfigBodyCtrl::_jpos_ctrl_wbdc(dynacore::Vector & gamma){
    wbdc_->UpdateSetting(A_, Ainv_, coriolis_, grav_);
    wbdc_->MakeTorque(task_list_, contact_list_, gamma, wbdc_data_);
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

    dynacore::Vector pos_des(valkyrie::num_qdot); pos_des.setZero();
    dynacore::Vector vel_des(valkyrie::num_qdot); vel_des.setZero();
    dynacore::Vector acc_des(valkyrie::num_qdot); acc_des.setZero();

    if(b_set_height_target_) body_height_cmd = target_body_height_;
    else body_height_cmd = ini_body_height_;
    inv_kin_->getDoubleSupportLegConfig(Q_cur, des_quat, body_height_cmd, config_sol);
    
    for (int i(0); i<valkyrie::num_act_joint; ++i){
        des_jpos_[i] = config_sol[valkyrie::num_virtual + i];
        pos_des[valkyrie::num_virtual + i] = des_jpos_[i];
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
    jpos_ini_ = sp_->Q_.segment(valkyrie::num_virtual, valkyrie::num_act_joint);
    ctrl_start_time_ = sp_->curr_time_;
    ini_body_height_ = sp_->Q_[valkyrie_joint::virtual_Z];
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
    jpos_ini_ = sp_->Q_.segment(valkyrie::num_virtual, valkyrie::num_act_joint);

    ParamHandler handler(ValkyrieConfigPath + setting_file_name + ".yaml");

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
