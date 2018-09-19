#include "Mercury_StateEstimator.hpp"
#include "Mercury_StateProvider.hpp"
#include "Mercury_interface.hpp"
#include <Utils/utilities.hpp>
#include <Mercury/Mercury_Model.hpp>
#include "Mercury_DynaControl_Definition.h"

// Mocap based Estimator
#include <Mercury_Controller/StateEstimator/BodyFootPosEstimator.hpp>

// Orientation Estimators
#include <Mercury_Controller/StateEstimator/BasicAccumulation.hpp>

// Velocity Estimators
#include <Mercury_Controller/StateEstimator/SimpleAverageEstimator.hpp>
#include "MoCapManager.hpp"

// TODO: body_ang_vel_ is defined in global frame currently although the truth is 
// rotation axis representing the floating base orientation changes
// as configuration change --> body (local) frame
Mercury_StateEstimator::Mercury_StateEstimator(RobotSystem* robot):
    base_cond_(0),
    b_using_jpos_(false),
    curr_config_(mercury::num_q),
    curr_qdot_(mercury::num_qdot)
{
    sp_ = Mercury_StateProvider::getStateProvider();
    robot_sys_ = robot;

    body_foot_est_ = new BodyFootPosEstimator(robot);
    ori_est_ = new BasicAccumulation();
    vel_est_ = new SimpleAverageEstimator();
    mocap_vel_est_ = new SimpleAverageEstimator();
}

Mercury_StateEstimator::~Mercury_StateEstimator(){
    delete body_foot_est_;
    delete vel_est_;
    delete mocap_vel_est_;
}

void Mercury_StateEstimator::Initialization(Mercury_SensorData* data){
    _JointUpdate(data);
    std::vector<double> imu_acc(3);
    std::vector<double> imu_ang_vel(3);

    for(int i(0); i<3; ++i){
        imu_acc[i]  = data->imu_acc[i];
        imu_ang_vel[i] = data->imu_ang_vel[i];
    }
    ori_est_->EstimatorInitialization(body_ori_, imu_acc, imu_ang_vel);   
    ori_est_->getEstimatedState(body_ori_, body_ang_vel_);
    body_foot_est_->Initialization(body_ori_);

    _ConfigurationAndModelUpdate();
    
    robot_sys_->getCoMPosition(sp_->CoM_pos_);
    robot_sys_->getCoMVelocity(sp_->CoM_vel_);
    ((BasicAccumulation*)ori_est_)->CoMStateInitialization(sp_->CoM_pos_, sp_->CoM_vel_);
    vel_est_->Initialization(sp_->CoM_vel_[0], sp_->CoM_vel_[1]);
    mocap_vel_est_->Initialization(sp_->CoM_vel_[0], sp_->CoM_vel_[1]);

    _FootContactUpdate(data);
    sp_->SaveCurrentData(data, robot_sys_);
}

void Mercury_StateEstimator::Update(Mercury_SensorData* data){
    _JointUpdate(data);
    
    std::vector<double> imu_acc(3);
    std::vector<double> imu_ang_vel(3);
    std::vector<double> imu_inc(3);

    for(int i(0); i<3; ++i){
        imu_inc[i] = data->imu_inc[i];
        imu_acc[i]  = data->imu_acc[i];
        imu_ang_vel[i] = data->imu_ang_vel[i];
    }
    ori_est_->setSensorData(imu_acc, imu_inc, imu_ang_vel);
    ori_est_->getEstimatedState(body_ori_, body_ang_vel_);

    static bool visit_once(false);
    if ((sp_->phase_copy_ == 2) && (!visit_once)){
        vel_est_->Initialization(sp_->CoM_vel_[0], sp_->CoM_vel_[1]);
        mocap_vel_est_->Initialization(sp_->CoM_vel_[0], sp_->CoM_vel_[1]);
        body_foot_est_->Initialization(body_ori_);
        visit_once = true;
    }

    _ConfigurationAndModelUpdate();

    robot_sys_->getCoMPosition(sp_->CoM_pos_);
    robot_sys_->getCoMVelocity(sp_->CoM_vel_);

    // CoM velocity data smoothing filter
    vel_est_->Update(sp_->CoM_vel_[0], sp_->CoM_vel_[1]);
    vel_est_->Output(sp_->est_CoM_vel_[0], sp_->est_CoM_vel_[1]);

    // Mocap based body velocity estimator
    dynacore::Vect3 mocap_body_vel;
    body_foot_est_->Update();
    body_foot_est_->getMoCapBodyVel(mocap_body_vel);
    body_foot_est_->getMoCapBodyPos(body_ori_, sp_->est_mocap_body_pos_);
    mocap_vel_est_->Update(mocap_body_vel[0], mocap_body_vel[1]);
    mocap_vel_est_->Output(
            sp_->est_mocap_body_vel_[0], 
            sp_->est_mocap_body_vel_[1]);

    _FootContactUpdate(data);
    sp_->SaveCurrentData(data, robot_sys_);
}

void Mercury_StateEstimator::_JointUpdate(Mercury_SensorData* data){
    curr_config_.setZero();
    curr_qdot_.setZero();
    curr_config_[mercury::num_qdot] = 1.;

    sp_->jjpos_config_.setZero();
    sp_->jjvel_qdot_.setZero();
    sp_->jjpos_config_[mercury::num_qdot] = 1.;

    for (int i(0); i<mercury::num_act_joint; ++i){
        if(b_using_jpos_){
            curr_config_[mercury::num_virtual + i] = data->joint_jpos[i];    
        } else{
            curr_config_[mercury::num_virtual + i] = data->motor_jpos[i];
        }
        curr_qdot_[mercury::num_virtual + i] = data->motor_jvel[i];
        sp_->rotor_inertia_[i] = data->reflected_rotor_inertia[i];

        // Joint encoder update
        sp_->jjpos_config_[mercury::num_virtual + i] = data->joint_jpos[i];
        sp_->jjvel_qdot_[mercury::num_virtual + i] = data->joint_jvel[i];
        sp_->mjpos_[i] = data->motor_jpos[i];
    }
}

void Mercury_StateEstimator::_ConfigurationAndModelUpdate(){
    // Local Frame Setting
    if(base_cond_ == base_condition::floating){
        curr_config_[3] = body_ori_.x();
        curr_config_[4] = body_ori_.y();
        curr_config_[5] = body_ori_.z();
        curr_config_[mercury::num_qdot] = body_ori_.w();

        for(int i(0); i<3; ++i)
            curr_qdot_[i+3] = body_ang_vel_[i];

        robot_sys_->UpdateSystem(curr_config_, curr_qdot_);

        dynacore::Vect3 foot_pos, foot_vel;
        robot_sys_->getPos(sp_->stance_foot_, foot_pos);
        robot_sys_->getLinearVel(sp_->stance_foot_, foot_vel);
        curr_config_[0] = -foot_pos[0];
        curr_config_[1] = -foot_pos[1];
        curr_config_[2] = -foot_pos[2];
        curr_qdot_[0] = -foot_vel[0];
        curr_qdot_[1] = -foot_vel[1];
        curr_qdot_[2] = -foot_vel[2];

        robot_sys_->UpdateSystem(curr_config_, curr_qdot_);

        /// Jpos based model update  ////////////////////////////////
        if(b_jpos_model_update_){
            sp_->jjpos_config_[3] = body_ori_.x();
            sp_->jjpos_config_[4] = body_ori_.y();
            sp_->jjpos_config_[5] = body_ori_.z();
            sp_->jjpos_config_[mercury::num_qdot] = body_ori_.w();
            for(int i(0); i<3; ++i)
                sp_->jjvel_qdot_[i+3] = body_ang_vel_[i];

            sp_->jjpos_robot_sys_->UpdateSystem(sp_->jjpos_config_, sp_->jjvel_qdot_);

            sp_->jjpos_robot_sys_->getPos(sp_->stance_foot_, foot_pos);
            sp_->jjpos_robot_sys_->getLinearVel(sp_->stance_foot_, foot_vel);
            sp_->jjpos_config_[0] = -foot_pos[0];
            sp_->jjpos_config_[1] = -foot_pos[1];
            sp_->jjpos_config_[2] = -foot_pos[2];
            sp_->jjvel_qdot_[0] = -foot_vel[0];
            sp_->jjvel_qdot_[1] = -foot_vel[1];
            sp_->jjvel_qdot_[2] = -foot_vel[2];

            sp_->jjpos_robot_sys_->UpdateSystem(sp_->jjpos_config_, sp_->jjvel_qdot_);
        }
        /// END of Jpos based model update ///////////////////////////
    } else if (base_cond_ == base_condition::fixed){
        robot_sys_->UpdateSystem(curr_config_, curr_qdot_);

    } else if (base_cond_ == base_condition::lying){
        // pitch rotation (PI/2)
        curr_config_[4] = sin(M_PI/2.0/2.0);
        curr_config_[mercury::num_qdot] = cos(M_PI/2.0/2.0);

        robot_sys_->UpdateSystem(curr_config_, curr_qdot_);
    } else {
        printf("[Error] Incorrect base condition setup\n");
        exit(0);
    }
    sp_->Q_ = curr_config_;
    sp_->Qdot_ = curr_qdot_;
}

void Mercury_StateEstimator::_FootContactUpdate(Mercury_SensorData* data){
    // Right Contact 
    if(data->rfoot_contact) sp_->b_rfoot_contact_ = 1;
    else sp_->b_rfoot_contact_ = 0;
    // Left Contact 
    if(data->lfoot_contact) sp_->b_lfoot_contact_ = 1;
    else sp_->b_lfoot_contact_ = 0;
}
