#include "JPosTrajPlanningCtrl.hpp"
#include <Configuration.h>
#include <Atlas_Controller/Atlas_StateProvider.hpp>
#include <Atlas_Controller/TaskSet/ConfigTask.hpp>
#include <Atlas_Controller/ContactSet/SingleContact.hpp>
#include <WBDC/WBDC.hpp>
#include <Atlas/Atlas_Model.hpp>
#include <Atlas_Controller/Atlas_DynaControl_Definition.h>
#include <ParamHandler/ParamHandler.hpp>
#include <Planner/PIPM_FootPlacementPlanner/Reversal_LIPM_Planner.hpp>
#include <Utils/utilities.hpp>
#include <Utils/DataManager.hpp>


#if MEASURE_TIME_WBDC
#include <chrono>
#endif 

JPosTrajPlanningCtrl::JPosTrajPlanningCtrl(
        const RobotSystem* robot, int swing_foot, Planner* planner):
    SwingPlanningCtrl(robot, swing_foot, planner),
    des_jpos_(atlas::num_act_joint),
    des_jvel_(atlas::num_act_joint),
    push_down_height_(0.0),
    initial_traj_mix_ratio_(0.6)
{
    config_body_foot_task_ = new ConfigTask();
    if(swing_foot == atlas_link::leftFoot) {
        single_contact_ = new SingleContact(robot, atlas_link::rightFoot); 
        swing_leg_jidx_ = atlas_joint::l_leg_hpz;
        swing_leg_roll_jidx_ = atlas_joint::l_leg_hpx;
        swing_leg_pitch_jidx_ = atlas_joint::l_leg_hpy;

        inv_kin_ = new Atlas_InvKinematics(swing_foot, atlas_link::rightFoot);
    }
    else if(swing_foot == atlas_link::rightFoot) {
        single_contact_ = new SingleContact(robot, atlas_link::leftFoot);
        swing_leg_jidx_ = atlas_joint::r_leg_hpz;
        swing_leg_roll_jidx_ = atlas_joint::r_leg_hpx;
        swing_leg_pitch_jidx_ = atlas_joint::r_leg_hpy;
        inv_kin_ = new Atlas_InvKinematics(swing_foot, atlas_link::leftFoot);
    }
    else printf("[Warnning] swing foot is not foot: %i\n", swing_foot);

    task_kp_ = dynacore::Vector::Zero(config_body_foot_task_->getDim());
    task_kd_ = dynacore::Vector::Zero(config_body_foot_task_->getDim());

    std::vector<bool> act_list;
    act_list.resize(atlas::num_qdot, true);
    for(int i(0); i<atlas::num_virtual; ++i) act_list[i] = false;


    wbdc_ = new WBDC(act_list);
    wbdc_data_ = new WBDC_ExtraData();
    wbdc_data_->cost_weight = 
        dynacore::Vector::Constant(
                config_body_foot_task_->getDim() + 
                single_contact_->getDim(), 1000.0);

    wbdc_data_->cost_weight.tail(single_contact_->getDim()) = 
        dynacore::Vector::Constant(single_contact_->getDim(), 1.0);
    wbdc_data_->cost_weight[config_body_foot_task_->getDim() + 5]  = 0.001; // Fr_z


    for(size_t i = 0; i < atlas::num_leg_joint; i++){
        min_jerk_jpos_initial_.push_back(new MinJerk_OneDimension());
    }
    printf("[BodyFootJPosPlanning Controller] Constructed\n");
}

void JPosTrajPlanningCtrl::_SetMinJerkTraj(
        double moving_duration,
        const dynacore::Vector & st_pos,
        const dynacore::Vector & st_vel,
        const dynacore::Vector & st_acc,
        const dynacore::Vector & target_pos,
        const dynacore::Vector & target_vel,
        const dynacore::Vector & target_acc){

    std::vector< dynacore::Vect3 > min_jerk_initial_params;
    std::vector< dynacore::Vect3 > min_jerk_final_params;   
    dynacore::Vect3 init_params; 
    dynacore::Vect3 final_params;

    // Set Minimum Jerk Boundary Conditions
    for(size_t i = 0; i < atlas::num_leg_joint; i++){
        init_params.setZero(); final_params.setZero();
        // Set Dimension i's initial pos, vel and acceleration
        init_params[0] = st_pos[i]; 
        init_params[1] = st_vel[i];  
        init_params[2] = st_acc[i];
        // Set Dimension i's final pos, vel, acceleration
        final_params[0] = target_pos[i]; 
        final_params[1] = target_vel[i];  
        final_params[2] = target_acc[i];
        // Add to the parameter vector 
        min_jerk_initial_params.push_back(init_params);
        min_jerk_final_params.push_back(final_params);
    }


    // Set Minimum Jerk Parameters for each dimension
    for(size_t i = 0; i < atlas::num_leg_joint; i++){
        min_jerk_jpos_initial_[i]->setParams(
                min_jerk_initial_params[i], 
                min_jerk_final_params[i],
                0., moving_duration);  
        // min_jerk_cartesian[i]->printParameters();  
    }
}


JPosTrajPlanningCtrl::~JPosTrajPlanningCtrl(){
    delete config_body_foot_task_;
    delete single_contact_;
}

void JPosTrajPlanningCtrl::OneStep(void* _cmd){
    _PreProcessing_Command();
    state_machine_time_ = sp_->curr_time_ - ctrl_start_time_;

    dynacore::Vector gamma;
    _single_contact_setup();
    _task_setup();
    _body_foot_ctrl(gamma);

    for(int i(0); i<atlas::num_act_joint; ++i){
        ((Atlas_Command*)_cmd)->jtorque_cmd[i] = gamma[i];
        ((Atlas_Command*)_cmd)->jpos_cmd[i] = des_jpos_[i];
        ((Atlas_Command*)_cmd)->jvel_cmd[i] = des_jvel_[i];
    }

    _PostProcessing_Command();
}

void JPosTrajPlanningCtrl::_body_foot_ctrl(dynacore::Vector & gamma){
    wbdc_->UpdateSetting(A_, Ainv_, coriolis_, grav_);
    wbdc_->MakeTorque(task_list_, contact_list_, gamma, wbdc_data_);
    //dynacore::pretty_print(gamma,std::cout, "gamma");
}


void JPosTrajPlanningCtrl::_task_setup(){
    dynacore::Vector pos_des(config_body_foot_task_->getDim());
    dynacore::Vector vel_des(config_body_foot_task_->getDim());
    dynacore::Vector acc_des(config_body_foot_task_->getDim());
    pos_des.setZero(); vel_des.setZero(); acc_des.setZero();

    // Body height
    double target_height = ini_body_pos_[2];
    if(b_set_height_target_) target_height = des_body_height_;

    // Orientation
    dynacore::Vect3 rpy_des;
    dynacore::Quaternion des_quat;
    rpy_des.setZero();
    dynacore::convert(rpy_des, des_quat);

    _CheckPlanning();
    double traj_time = state_machine_time_ - replan_moment_;

    dynacore::Vector config_sol; 
    dynacore::Vector qdot_cmd = dynacore::Vector::Zero(atlas::num_qdot);
    dynacore::Vector qddot_cmd = dynacore::Vector::Zero(atlas::num_qdot);

    inv_kin_->getSingleSupportStanceLegConfiguration(
            sp_->Q_, des_quat, target_height, config_sol);

    double ramp_time(0.02);
    if(state_machine_time_ > half_swing_time_){
        double traj_time = state_machine_time_-half_swing_time_;
        for(int i(0); i<atlas::num_leg_joint; ++i){
            config_sol[swing_leg_jidx_ + i] = dynacore::smooth_changing(
                    mid_swing_leg_config_[i], target_swing_leg_config_[i], half_swing_time_, traj_time);
            qdot_cmd[swing_leg_jidx_ + i] = dynacore::smooth_changing_vel(
                    mid_swing_leg_config_[i], target_swing_leg_config_[i], half_swing_time_, traj_time);
            qddot_cmd[swing_leg_jidx_ + i] = dynacore::smooth_changing_acc(
                    mid_swing_leg_config_[i], target_swing_leg_config_[i], half_swing_time_, traj_time);
        }
        config_sol[swing_leg_roll_jidx_] += roll_offset_gain_ * sp_->Q_[3];
        config_sol[swing_leg_pitch_jidx_] += pitch_offset_gain_ * sp_->Q_[4];
    }else{
        if(state_machine_time_ < half_swing_time_ * initial_traj_mix_ratio_){
            double jpos, jvel, jacc;
            for(int i(0); i<atlas::num_leg_joint; ++i){
                min_jerk_jpos_initial_[i]->getPos(state_machine_time_, jpos);
                min_jerk_jpos_initial_[i]->getVel(state_machine_time_, jvel);
                min_jerk_jpos_initial_[i]->getAcc(state_machine_time_, jacc);        

                config_sol[swing_leg_jidx_ + i] = jpos;
                qdot_cmd[swing_leg_jidx_ + i] = jvel;
                qddot_cmd[swing_leg_jidx_ + i] = jacc; 
            }
        } else {
            for(int i(0); i<atlas::num_leg_joint; ++i){
                config_sol[swing_leg_jidx_ + i] = dynacore::smooth_changing(
                        ini_swing_leg_config_[i], mid_swing_leg_config_[i], 
                        half_swing_time_, state_machine_time_);
                qdot_cmd[swing_leg_jidx_ + i] = dynacore::smooth_changing_vel(
                        ini_swing_leg_config_[i], mid_swing_leg_config_[i], 
                        half_swing_time_, state_machine_time_);
                qddot_cmd[swing_leg_jidx_ + i] = dynacore::smooth_changing_acc(
                        ini_swing_leg_config_[i], mid_swing_leg_config_[i], 
                        half_swing_time_, state_machine_time_);
            }
        }
    }

    for (int i(0); i<atlas::num_act_joint; ++i){
        pos_des[atlas::num_virtual + i] = config_sol[atlas::num_virtual + i];  
        vel_des[atlas::num_virtual + i] = qdot_cmd[atlas::num_virtual + i];
        acc_des[atlas::num_virtual + i] = qddot_cmd[atlas::num_virtual + i];

        des_jpos_[i] = pos_des[atlas::num_virtual + i];
        des_jvel_[i] = vel_des[atlas::num_virtual + i];
    }

    //dynacore::pretty_print(pos_des, std::cout, "pos des");
    //dynacore::pretty_print(vel_des, std::cout, "vel des");
    //dynacore::pretty_print(acc_des, std::cout, "acc des");
    //dynacore::pretty_print(config_sol, std::cout, "config sol");
    // Push back to task list
    config_body_foot_task_->UpdateTask(&(pos_des), vel_des, acc_des);
    task_list_.push_back(config_body_foot_task_);
}

void JPosTrajPlanningCtrl::_CheckPlanning(){
    if( state_machine_time_ > 
            (end_time_/(planning_frequency_ + 1.) * (num_planning_ + 1.) + 0.002) ){
        dynacore::Vect3 target_loc;
        _Replanning(target_loc);

        dynacore::Vector guess_q = sp_->Q_;
        dynacore::Vector config_sol = sp_->Q_;

        // Update Target config
        inv_kin_->getLegConfigAtVerticalPosture(target_loc, guess_q, config_sol);
        target_swing_leg_config_ = 
            config_sol.segment(swing_leg_jidx_, atlas::num_leg_joint);

        ++num_planning_;
    }
}

void JPosTrajPlanningCtrl::_Replanning(dynacore::Vect3 & target_loc){
    dynacore::Vect3 com_pos, com_vel;
    dynacore::Vect3 del_com_pos;
    // Direct value used
    robot_sys_->getCoMPosition(com_pos);
    robot_sys_->getCoMVelocity(com_vel);
    //dynacore::pretty_print(com_pos, std::cout, "com pos");
    //dynacore::pretty_print(com_vel, std::cout, "com vel");
    
    com_pos[0] = sp_->Q_[0] + body_pt_offset_[0]; 
    com_pos[1] = sp_->Q_[1] + body_pt_offset_[1]; 
    com_vel[0] = sp_->Qdot_[0]; 
    com_vel[1] = sp_->Qdot_[1]; 

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

    if(swing_foot_ == atlas_link::leftFoot)
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


void JPosTrajPlanningCtrl::_single_contact_setup(){
    single_contact_->UpdateContactSpec();
    contact_list_.push_back(single_contact_);
}

void JPosTrajPlanningCtrl::FirstVisit(){
    ini_config_ = sp_->Q_;
    ctrl_start_time_ = sp_->curr_time_;
    state_machine_time_ = 0.;
    replan_moment_ = 0.;

    // Initial set by the current configuration
    ini_swing_leg_config_ = sp_->Q_.segment(swing_leg_jidx_, atlas::num_leg_joint);
    robot_sys_->getPos(swing_foot_, ini_foot_pos_);
    dynacore::Vector zero(atlas::num_leg_joint);
    zero.setZero();

    dynacore::Vect3 target_loc = default_target_loc_;
    target_loc[0] += sp_->Q_[0];
    target_loc[1] += sp_->Q_[1];
    target_loc[2] = ini_foot_pos_[2] - push_down_height_; 
    default_target_loc_[2] = target_loc[2];

    // Compute Middle point Configuration
    dynacore::Vect3 middle_pos;
    dynacore::Vector config_sol = sp_->Q_;

    middle_pos = (ini_foot_pos_ + target_loc)/2.;
    middle_pos[2] = swing_height_ + target_loc[2];

    inv_kin_->getLegConfigAtVerticalPosture(middle_pos, sp_->Q_, config_sol);
    mid_swing_leg_config_ = config_sol.segment(swing_leg_jidx_, atlas::num_leg_joint);

    // Compute Target config
    inv_kin_->getLegConfigAtVerticalPosture(target_loc, sp_->Q_, config_sol);
    target_swing_leg_config_ = 
        config_sol.segment(swing_leg_jidx_, atlas::num_leg_joint);

    // Min jerk smoothing at initial
    dynacore::Vector mid_mix_pt_pos(atlas::num_leg_joint);
    dynacore::Vector mid_mix_pt_vel(atlas::num_leg_joint);
    dynacore::Vector mid_mix_pt_acc(atlas::num_leg_joint);

    double mid_mix_time = half_swing_time_ * initial_traj_mix_ratio_;
    for(int i(0); i<atlas::num_leg_joint; ++i){
        mid_mix_pt_pos[i] = dynacore::smooth_changing(
                ini_swing_leg_config_[i], mid_swing_leg_config_[i], half_swing_time_, mid_mix_time);
        mid_mix_pt_vel[i] = dynacore::smooth_changing_vel(
                ini_swing_leg_config_[i], mid_swing_leg_config_[i], half_swing_time_, mid_mix_time);
        mid_mix_pt_acc[i] = dynacore::smooth_changing_acc(
                ini_swing_leg_config_[i], mid_swing_leg_config_[i], half_swing_time_, mid_mix_time);
    }
    _SetMinJerkTraj(mid_mix_time,
            ini_swing_leg_config_, zero, zero, 
            mid_mix_pt_pos, mid_mix_pt_vel, mid_mix_pt_acc );

    //dynacore::pretty_print(ini_swing_leg_config_, std::cout, "mid config");
    //dynacore::pretty_print(mid_swing_leg_config_, std::cout, "mid config");
    //dynacore::pretty_print(target_swing_leg_config_, std::cout, "targe config");
    num_planning_ = 0;
}

void JPosTrajPlanningCtrl::LastVisit(){
}

bool JPosTrajPlanningCtrl::EndOfPhase(){
    if(state_machine_time_ > end_time_){
        //printf("[Body Foot Ctrl] End, state_machine time/ end time: (%f, %f)\n", 
        //state_machine_time_, end_time_);
        return true;
    }
    // Swing foot contact = END
    if(b_contact_switch_check_){
        bool contact_happen(false);
        if(swing_foot_ == atlas_link::leftFoot && sp_->b_lfoot_contact_){
            contact_happen = true;
        }
        if(swing_foot_ == atlas_link::rightFoot && sp_->b_rfoot_contact_){
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

void JPosTrajPlanningCtrl::_setTaskGain(
        const dynacore::Vector & Kp, const dynacore::Vector & Kd){
    ((ConfigTask*)config_body_foot_task_)->Kp_vec_ = Kp;
    ((ConfigTask*)config_body_foot_task_)->Kd_vec_ = Kd;
}

void JPosTrajPlanningCtrl::CtrlInitialization(const std::string & setting_file_name){
    ini_body_pos_ = sp_->Q_.head(3);
    std::vector<double> tmp_vec;

    // Setting Parameters
    ParamHandler handler(AtlasConfigPath + setting_file_name + ".yaml");
    handler.getValue("swing_height", swing_height_);
    handler.getValue("push_down_height", push_down_height_);
    handler.getVector("default_target_foot_location", tmp_vec);
    for(int i(0); i<3; ++i){
        default_target_loc_[i] = tmp_vec[i];
    }

    // Feedback Gain
    handler.getVector("Kp", tmp_vec);
    for(int i(0); i<tmp_vec.size(); ++i){
        task_kp_[i] = tmp_vec[i];
    }
    handler.getVector("Kd", tmp_vec);
    for(int i(0); i<tmp_vec.size(); ++i){
        task_kd_[i] = tmp_vec[i];
    }
    handler.getValue("roll_offset_gain", roll_offset_gain_);
    handler.getValue("pitch_offset_gain", pitch_offset_gain_);
    // Body Point offset
    handler.getVector("body_pt_offset", tmp_vec);
    for(int i(0); i<2; ++i){
        body_pt_offset_[i] = tmp_vec[i];
    }


    ((ConfigTask*)config_body_foot_task_)->Kp_vec_ = task_kp_;
    ((ConfigTask*)config_body_foot_task_)->Kd_vec_ = task_kd_;

    static bool b_bodypute_eigenvalue(true);
    if(b_bodypute_eigenvalue){
        ((Reversal_LIPM_Planner*)planner_)->
            CheckEigenValues(double_stance_ratio_*stance_time_ + 
                    transition_phase_ratio_*transition_time_ + 
                    end_time_);
        b_bodypute_eigenvalue = false;
    }
}
