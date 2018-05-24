#include "Mercury_StateEstimator.hpp"
#include "Mercury_StateProvider.hpp"
#include "Mercury_interface.hpp"
#include <Utils/utilities.hpp>
#include <Mercury/Mercury_Model.hpp>
#include <Mercury/Mercury_Definition.h>
#include <Filter/filters.hpp>

// Mocap based Estimator
#include <Mercury_Controller/StateEstimator/BodyFootPosEstimator.hpp>

// Orientation Estimators
#include <Mercury_Controller/StateEstimator/OriEstAccObs.hpp>
#include <Mercury_Controller/StateEstimator/BasicAccumulation.hpp>
#include <Mercury_Controller/StateEstimator/NoBias.hpp>
#include <Mercury_Controller/StateEstimator/NoAccState.hpp>

// EKF based estimators
#include <Mercury_Controller/StateEstimator/EKF_RotellaEstimator.hpp> // EKF
#include "MoCapManager.hpp"

Mercury_StateEstimator::Mercury_StateEstimator(RobotSystem* robot):
    base_cond_(0),
    curr_config_(mercury::num_q),
    curr_qdot_(mercury::num_qdot)
{
    sp_ = Mercury_StateProvider::getStateProvider();
    robot_sys_ = robot;

    body_foot_est_ = new BodyFootPosEstimator(robot);
    ori_est_ = new BasicAccumulation();
    ekf_est_ = new EKF_RotellaEstimator(); // EKF
    // ori_est_ = new OriEstAccObs();
    // ori_est_ = new NoBias();
    //ori_est_ = new NoAccState();

}

Mercury_StateEstimator::~Mercury_StateEstimator(){
    delete ori_est_;
}

void Mercury_StateEstimator::Initialization(Mercury_SensorData* data){
    curr_config_.setZero();
    curr_qdot_.setZero();
    curr_config_[mercury::num_qdot] = 1.;

    // Joint Set
    for (int i(0); i<mercury::num_act_joint; ++i){
        curr_config_[mercury::num_virtual + i] = data->joint_jpos[i];
        //curr_config_[mercury::num_virtual + i] = data->motor_jpos[i];
        curr_qdot_[mercury::num_virtual + i] = data->motor_jvel[i];

        sp_->rotor_inertia_[i] = data->reflected_rotor_inertia[i];
    }
    // Update Orientation w/ Mocap data
    // body_foot_est_->getMoCapBodyOri(sp_->body_ori_);
    std::vector<double> imu_acc(3);
    std::vector<double> imu_ang_vel(3);

    for(int i(0); i<3; ++i){
        imu_acc[i]  = data->imu_acc[i];
        imu_ang_vel[i] = data->imu_ang_vel[i];
    }

    ori_est_->EstimatorInitialization(sp_->body_ori_, imu_acc, imu_ang_vel);   
    ori_est_->getEstimatedState(sp_->body_ori_, sp_->body_ang_vel_);
    ekf_est_->EstimatorInitialization(sp_->body_ori_, imu_acc, imu_ang_vel); // EKF

    // Local Frame Setting
    if(base_cond_ == base_condition::floating){
        curr_config_[3] = sp_->body_ori_.x();
        curr_config_[4] = sp_->body_ori_.y();
        curr_config_[5] = sp_->body_ori_.z();
        curr_config_[mercury::num_qdot] = sp_->body_ori_.w();

        for(int i(0); i<3; ++i)
            curr_qdot_[i+3] = sp_->body_ang_vel_[i];

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

        //curr_config_[0] += sp_->global_pos_local_[0];
        //curr_config_[1] += sp_->global_pos_local_[1];
        //curr_config_[2] += sp_->global_foot_height_;
        robot_sys_->UpdateSystem(curr_config_, curr_qdot_);
    } else if (base_cond_ == base_condition::fixed){
        robot_sys_->UpdateSystem(curr_config_, curr_qdot_);
        
    } else if (base_cond_ == base_condition::lying){
        // pitch rotation (PI/2)
        curr_config_[4] = sin(M_PI/2.0/2.0);
        curr_config_[mercury::num_qdot] = cos(M_PI/2.0/2.0);

        robot_sys_->UpdateSystem(curr_config_, curr_qdot_);
    }
    sp_->Q_ = curr_config_;
    sp_->Qdot_ = curr_qdot_;

    robot_sys_->getCoMPosition(sp_->CoM_pos_);
    robot_sys_->getCoMVelocity(sp_->CoM_vel_);

    ((BasicAccumulation*)ori_est_)->CoMStateInitialization(sp_->CoM_pos_, sp_->CoM_vel_);

    // Warning: state provider setup
    sp_->SaveCurrentData();
    // LED data save  ////////////////////
    dynacore::Vect3 pos;
    int led_idx = mercury_link::LED_BODY_0;
    for(int i(0); i<NUM_MARKERS; ++i){
        robot_sys_->getPos(mercury_link::LED_BODY_0 + i, pos);
        for (int j(0); j<3; ++j)  sp_->led_kin_data_[3*i + j] = pos[j];
    }
    //////////////////////////////////////

    // Right Contact 
    if(data->rfoot_contact) sp_->b_rfoot_contact_ = 1;
    else sp_->b_rfoot_contact_ = 0;
    // Left Contact 
    if(data->lfoot_contact) sp_->b_lfoot_contact_ = 1;
    else sp_->b_lfoot_contact_ = 0;

    // Foot pos
    robot_sys_->getPos(mercury_link::rightFoot, sp_->Rfoot_pos_);
    robot_sys_->getLinearVel(mercury_link::rightFoot, sp_->Rfoot_vel_);
    robot_sys_->getPos(mercury_link::leftFoot, sp_->Lfoot_pos_);
    robot_sys_->getLinearVel(mercury_link::leftFoot, sp_->Lfoot_vel_);
}

void Mercury_StateEstimator::Update(Mercury_SensorData* data){
    curr_config_.setZero();
    curr_qdot_.setZero();
    curr_config_[mercury::num_qdot] = 1.;


    for (int i(0); i<mercury::num_act_joint; ++i){
        curr_config_[mercury::num_virtual + i] = data->joint_jpos[i];
        //curr_config_[mercury::num_virtual + i] = data->motor_jpos[i];
        curr_qdot_[mercury::num_virtual + i] = data->motor_jvel[i];
        
        sp_->rotor_inertia_[i] = data->reflected_rotor_inertia[i];
    }
    std::vector<double> imu_acc(3);
    std::vector<double> imu_ang_vel(3);
    std::vector<double> imu_inc(3);

    for(int i(0); i<3; ++i){
        imu_inc[i] = data->imu_inc[i];
        imu_acc[i]  = data->imu_acc[i];
        imu_ang_vel[i] = data->imu_ang_vel[i];
    }
    ori_est_->setSensorData(imu_acc, imu_inc, imu_ang_vel);
    ori_est_->getEstimatedState(sp_->body_ori_, sp_->body_ang_vel_);
    ((BasicAccumulation*)ori_est_)->getEstimatedCoMState(sp_->com_state_imu_);
    //printf("\n");


    // EKF set sensor data
    ekf_est_->setSensorData(imu_acc, imu_inc, imu_ang_vel, 
                            data->lfoot_contact, 
                            data->rfoot_contact,
                            curr_config_.segment(mercury::num_virtual, mercury::num_act_joint));

    if(base_cond_ == base_condition::floating){
        curr_config_[3] = sp_->body_ori_.x();
        curr_config_[4] = sp_->body_ori_.y();
        curr_config_[5] = sp_->body_ori_.z();
        curr_config_[mercury::num_qdot] = sp_->body_ori_.w();

        for(int i(0); i<3; ++i)
            curr_qdot_[i+3] = sp_->body_ang_vel_[i];

        robot_sys_->UpdateSystem(curr_config_, curr_qdot_);

        // Foot position based offset
        dynacore::Vect3 foot_pos, foot_vel;
        robot_sys_->getPos(sp_->stance_foot_, foot_pos);
        robot_sys_->getLinearVel(sp_->stance_foot_, foot_vel);

        curr_config_[0] = -foot_pos[0];
        curr_config_[1] = -foot_pos[1];
        curr_config_[2] = -foot_pos[2];
        curr_qdot_[0] = -foot_vel[0];
        curr_qdot_[1] = -foot_vel[1];
        curr_qdot_[2] = -foot_vel[2];

        //curr_config_[0] += sp_->global_pos_local_[0];
        //curr_config_[1] += sp_->global_pos_local_[1];
        //curr_config_[2] += sp_->global_foot_height_;

        //curr_config_[2] += sp_->global_pos_local_[2];
        robot_sys_->UpdateSystem(curr_config_, curr_qdot_);

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


    // Warning: Save Sensor Data in StateProvider
    sp_->SaveCurrentData();
    // LED data save  ////////////////////
    dynacore::Vect3 pos;
    int led_idx = mercury_link::LED_BODY_0;
    for(int i(0); i<NUM_MARKERS; ++i){
        robot_sys_->getPos(mercury_link::LED_BODY_0 + i, pos);
        for (int j(0); j<3; ++j)  sp_->led_kin_data_[3*i + j] = pos[j];
    }
    //////////////////////////////////////

    robot_sys_->getCoMPosition(sp_->CoM_pos_);
    robot_sys_->getCoMVelocity(sp_->CoM_vel_);
    // Right Contact 
    if(data->rfoot_contact) sp_->b_rfoot_contact_ = 1;
    else sp_->b_rfoot_contact_ = 0;
    // Left Contact 
    if(data->lfoot_contact) sp_->b_lfoot_contact_ = 1;
    else sp_->b_lfoot_contact_ = 0;

    for(int i(0); i<3; ++i){
        sp_->imu_acc_inc_[i] = data->imu_inc[i];
        sp_->imu_acc_[i] = data->imu_acc[i];
        sp_->imu_ang_vel_[i] = data->imu_ang_vel[i];
    }
    // Foot pos
    robot_sys_->getPos(mercury_link::rightFoot, sp_->Rfoot_pos_);
    robot_sys_->getLinearVel(mercury_link::rightFoot, sp_->Rfoot_vel_);
    robot_sys_->getPos(mercury_link::leftFoot, sp_->Lfoot_pos_);
    robot_sys_->getLinearVel(mercury_link::leftFoot, sp_->Lfoot_vel_);
}
