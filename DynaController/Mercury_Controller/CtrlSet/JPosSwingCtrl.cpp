#include "JPosSwingCtrl.hpp"
#include <Configuration.h>
#include <Mercury_Controller/Mercury_StateProvider.hpp>
#include <Mercury_Controller/TaskSet/JPosTask.hpp>
#include <Mercury_Controller/ContactSet/FixedBodyContact.hpp>
#include <WBDC_Rotor/WBDC_Rotor.hpp>
#include <Mercury/Mercury_Model.hpp>
#include <Mercury/Mercury_Definition.h>
#include <Mercury_Controller/Mercury_DynaControl_Definition.h>
#include <ParamHandler/ParamHandler.hpp>
#include <Planner/PIPM_FootPlacementPlanner/Reversal_LIPM_Planner.hpp>
#include <Utils/utilities.hpp>
#include <Utils/DataManager.hpp>


#if MEASURE_TIME_WBDC
#include <chrono>
#endif 

JPosSwingCtrl::JPosSwingCtrl(
        const RobotSystem* robot, int swing_foot, Planner* planner):
    Controller(robot),
    planner_(planner),
    des_jpos_(mercury::num_act_joint),
    des_jvel_(mercury::num_act_joint),
    jpos_swing_delta_(3),
    stance_jpos_(mercury::num_act_joint),
    planning_moment_portion_(0.5),
    push_down_height_(0.0),
    b_replan_(false)
{
    jpos_task_ = new JPosTask();
    contact_constraint_ = new FixedBodyContact(robot);

    if(swing_foot == mercury_link::leftFoot) {
        swing_leg_jidx_ = mercury_joint::leftAbduction;
    }
    else if(swing_foot == mercury_link::rightFoot) {
        swing_leg_jidx_ = mercury_joint::rightAbduction;
    }
    else printf("[Warnning] swing foot is not foot: %i\n", swing_foot);

    std::vector<bool> act_list;
    act_list.resize(mercury::num_qdot, true);
    for(int i(0); i<mercury::num_virtual; ++i) act_list[i] = false;

    wbdc_rotor_ = new WBDC_Rotor(act_list);
    wbdc_rotor_data_ = new WBDC_Rotor_ExtraData();
    wbdc_rotor_data_->A_rotor = 
        dynacore::Matrix::Zero(mercury::num_qdot, mercury::num_qdot);

    wbdc_rotor_data_->cost_weight = 
        dynacore::Vector::Constant(
                jpos_task_->getDim() + 
                contact_constraint_->getDim(), 100.0);

    wbdc_rotor_data_->cost_weight.tail(contact_constraint_->getDim()) = 
        dynacore::Vector::Constant(contact_constraint_->getDim(), 1.0);

    sp_ = Mercury_StateProvider::getStateProvider();

    for(size_t i = 0; i < 3; i++){
        min_jerk_jpos_initial_.push_back(new MinJerk_OneDimension());
    }
    printf("[BodyFootJPosPlanning Controller] Constructed\n");
}
void JPosSwingCtrl::_SetMinJerkTraj(
        double moving_duration,
        const dynacore::Vect3 & st_pos,
        const dynacore::Vect3 & st_vel,
        const dynacore::Vect3 & st_acc,
        const dynacore::Vect3 & target_pos,
        const dynacore::Vect3 & target_vel,
        const dynacore::Vect3 & target_acc){

    std::vector< dynacore::Vect3 > min_jerk_initial_params;
    std::vector< dynacore::Vect3 > min_jerk_final_params;   
    dynacore::Vect3 init_params; 
    dynacore::Vect3 final_params;

    // Set Minimum Jerk Boundary Conditions
    for(size_t i = 0; i < 3; i++){
        init_params.setZero(); final_params.setZero();
        // Set Dimension i's initial pos, vel and acceleration
        init_params[0] = st_pos[i]; init_params[1] = st_vel[i];  init_params[2] = st_acc[i];
        // Set Dimension i's final pos, vel, acceleration
        final_params[0] = target_pos[i]; final_params[1] = target_vel[i];  final_params[2] = target_acc[i];
        // Add to the parameter vector 
        min_jerk_initial_params.push_back(init_params);
        min_jerk_final_params.push_back(final_params);
    }


    // Set Minimum Jerk Parameters for each dimension
    for(size_t i = 0; i < 3; i++){
        min_jerk_jpos_initial_[i]->setParams(
                min_jerk_initial_params[i], 
                min_jerk_final_params[i],
                0., moving_duration);  
        // min_jerk_cartesian[i]->printParameters();  
    }
}


JPosSwingCtrl::~JPosSwingCtrl(){
    delete jpos_task_;
    delete contact_constraint_;
}

void JPosSwingCtrl::OneStep(void* _cmd){
    _PreProcessing_Command();
    state_machine_time_ = sp_->curr_time_ - ctrl_start_time_;

    dynacore::Vector gamma;
    _contact_setup();
    _task_setup();
    _body_foot_ctrl(gamma);

    for(int i(0); i<mercury::num_act_joint; ++i){
        ((Mercury_Command*)_cmd)->jtorque_cmd[i] = gamma[i];
        ((Mercury_Command*)_cmd)->jpos_cmd[i] = des_jpos_[i];
        ((Mercury_Command*)_cmd)->jvel_cmd[i] = des_jvel_[i];
    }

    _PostProcessing_Command();
}
void JPosSwingCtrl::_body_foot_ctrl(dynacore::Vector & gamma){
    dynacore::Vector fb_cmd = dynacore::Vector::Zero(mercury::num_act_joint);
    for (int i(0); i<mercury::num_act_joint; ++i){
        wbdc_rotor_data_->A_rotor(i + mercury::num_virtual, i + mercury::num_virtual)
            = sp_->rotor_inertia_[i];
    }
    wbdc_rotor_->UpdateSetting(A_, Ainv_, coriolis_, grav_);
    wbdc_rotor_->MakeTorque(task_list_, contact_list_, fb_cmd, wbdc_rotor_data_);

    gamma = wbdc_rotor_data_->cmd_ff;

    int offset(0);
    if(swing_foot_ == mercury_link::rightFoot) offset = 3;
    dynacore::Vector reaction_force = 
        (wbdc_rotor_data_->opt_result_).tail(contact_constraint_->getDim());
    for(int i(0); i<3; ++i)
        sp_->reaction_forces_[i + offset] = reaction_force[i];

    sp_->qddot_cmd_ = wbdc_rotor_data_->result_qddot_;
}


void JPosSwingCtrl::_task_setup(){
    dynacore::Vector acc_des(jpos_task_->getDim());
    des_jvel_.setZero(); acc_des.setZero();
    des_jpos_ = stance_jpos_;

    // Push back to task list
    jpos_task_->UpdateTask(&(des_jpos_), des_jvel_, acc_des);
    task_list_.push_back(jpos_task_);
}

void JPosSwingCtrl::_CheckPlanning(){
    if( state_machine_time_ > planning_moment_portion_ * end_time_ ){
        dynacore::Vect3 target_loc;
        _Replanning(target_loc);
    }
}

void JPosSwingCtrl::_Replanning(dynacore::Vect3 & target_loc){
    dynacore::Vect3 com_pos, com_vel;
    dynacore::Vect3 del_com_pos;
    // Direct value used
    robot_sys_->getCoMPosition(com_pos);
    robot_sys_->getCoMVelocity(com_vel);

    // Average velocity computation
    for(int i(0); i<2; ++i){ 
        sp_->average_vel_[i] = (sp_->Q_[i] - ini_config_[i])/state_machine_time_;
    }

    // TEST
    for(int i(0); i<2; ++i){
        // com_pos[i] = sp_->Q_[i] + body_pt_offset_[i];
        // TEST jpos update must be true
        // com_pos[i] = sp_->jjpos_body_pos_[i] + body_pt_offset_[i];
        //com_pos[i] += body_pt_offset_[i];
        // com_vel[i] = sp_->average_vel_[i]; 
        // com_vel[i] = sp_->est_CoM_vel_[i]; 
        com_pos[i] = sp_->est_mocap_body_pos_[i] + body_pt_offset_[i];
        com_vel[i] = sp_->est_mocap_body_vel_[i]; 
        //com_vel[i] = sp_->ekf_body_vel_[i]; 
    }

    printf("planning com state: %f, %f, %f, %f\n",
            com_pos[0], com_pos[1],
            com_vel[0], com_vel[1]);

    OutputReversalPL pl_output;
    ParamReversalPL pl_param;

    pl_param.swing_time = end_time_ - state_machine_time_
        + transition_time_ * transition_phase_ratio_
        + stance_time_ * double_stance_ratio_;

    pl_param.des_loc = sp_->des_location_;
    pl_param.stance_foot_loc = sp_->global_pos_local_;

    if(swing_foot_ == mercury_link::leftFoot)
        pl_param.b_positive_sidestep = true;
    else
        pl_param.b_positive_sidestep = false;

    planner_->getNextFootLocation(com_pos + sp_->global_pos_local_,
            com_vel,
            target_loc,
            &pl_param, &pl_output);
    // Time Modification
    replan_moment_ = state_machine_time_;
    end_time_ += pl_output.time_modification;
    target_loc -= sp_->global_pos_local_;

    target_loc[2] = default_target_loc_[2];
    dynacore::pretty_print(target_loc, std::cout, "next foot loc");
}


void JPosSwingCtrl::_contact_setup(){
    contact_constraint_->UpdateContactSpec();
    contact_list_.push_back(contact_constraint_);
}

void JPosSwingCtrl::FirstVisit(){
    ini_config_ = sp_->Q_;
    ctrl_start_time_ = sp_->curr_time_;
    state_machine_time_ = 0.;
    replan_moment_ = 0.;
}

void JPosSwingCtrl::LastVisit(){
}

bool JPosSwingCtrl::EndOfPhase(){
    if(state_machine_time_ > end_time_){
        //printf("[Body Foot Ctrl] End, state_machine time/ end time: (%f, %f)\n", 
        //state_machine_time_, end_time_);
        return true;
    }
    // Swing foot contact = END
    if(b_contact_switch_check_){
        bool contact_happen(false);
        if(swing_foot_ == mercury_link::leftFoot && sp_->b_lfoot_contact_){
            contact_happen = true;
        }
        if(swing_foot_ == mercury_link::rightFoot && sp_->b_rfoot_contact_){
            contact_happen = true;
        }
        if(state_machine_time_ > end_time_ * 0.5 && contact_happen){
            printf("[Body Foot JPos Ctrl] contact happen, state_machine_time/ end time: (%f, %f)\n",
                    state_machine_time_, end_time_);

            return true;
        }
    }
    return false;
}


void JPosSwingCtrl::CtrlInitialization(const std::string & setting_file_name){
    std::vector<double> tmp_vec;
    // Setting Parameters
    ParamHandler handler(MercuryConfigPath + setting_file_name + ".yaml");
    handler.getValue("push_down_height", push_down_height_);

    handler.getVector("jpos_swing_delta", tmp_vec);
    for(int i(0); i<3; ++i){ jpos_swing_delta_[i] = tmp_vec[i]; }

    // Feedback Gain
    handler.getVector("Kp", tmp_vec);
    for(int i(0); i<tmp_vec.size(); ++i){
        ((JPosTask*)jpos_task_)->Kp_vec_[i] = tmp_vec[i];
    }
    handler.getVector("Kd", tmp_vec);
    for(int i(0); i<tmp_vec.size(); ++i){
        ((JPosTask*)jpos_task_)->Kd_vec_[i] = tmp_vec[i];
    }

    // Body Point offset
    handler.getVector("body_pt_offset", tmp_vec);
    for(int i(0); i<2; ++i){
        body_pt_offset_[i] = tmp_vec[i];
    }

    handler.getValue("roll_offset_gain", roll_offset_gain_);
    handler.getValue("pitch_offset_gain", pitch_offset_gain_);
    
    static bool b_bodypute_eigenvalue(true);
    if(b_bodypute_eigenvalue){
        ((Reversal_LIPM_Planner*)planner_)->
            CheckEigenValues(double_stance_ratio_*stance_time_ + 
                    transition_phase_ratio_*transition_time_ + 
                    end_time_);
        b_bodypute_eigenvalue = false;
    }
}


