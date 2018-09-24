#include "DracoBip_StateEstimator.hpp"
#include "DracoBip_StateProvider.hpp"
#include <Utils/utilities.hpp>
#include <DracoBip/DracoBip_Model.hpp>
#include <DracoBip_Controller/DracoBip_DynaCtrl_Definition.h>

// Orientation Estimators
#include <DracoBip_Controller/StateEstimator/BasicAccumulation.hpp>

DracoBip_StateEstimator::DracoBip_StateEstimator(RobotSystem* robot):
    curr_config_(dracobip::num_q),
    curr_qdot_(dracobip::num_qdot)
{
    sp_ = DracoBip_StateProvider::getStateProvider();
    robot_sys_ = robot;
    ori_est_ = new BasicAccumulation();
}

DracoBip_StateEstimator::~DracoBip_StateEstimator(){
    delete ori_est_;
}

void DracoBip_StateEstimator::Initialization(DracoBip_SensorData* data){
    curr_config_.setZero();
    curr_qdot_.setZero();
    curr_config_[dracobip::num_qdot] = 1.;

    // Joint Set
    for (int i(0); i<dracobip::num_act_joint; ++i){
        curr_config_[dracobip::num_virtual + i] = data->jpos[i];
        curr_qdot_[dracobip::num_virtual + i] = data->jvel[i];
    }
    std::vector<double> imu_acc(3);
    std::vector<double> imu_ang_vel(3);

    for(int i(0); i<3; ++i){
        imu_acc[i]  = data->imu_acc[i];
        imu_ang_vel[i] = data->imu_ang_vel[i];
    }

    dynacore::Quaternion body_ori;
    body_ori.w() = 1.; body_ori.x() = 0.; body_ori.y() = 0.; body_ori.z() = 0;
    dynacore::Vect3 body_ang_vel;

    ori_est_->EstimatorInitialization(body_ori, imu_acc, imu_ang_vel);   
    ori_est_->getEstimatedState(body_ori, body_ang_vel);

    curr_config_[3] = body_ori.x();
    curr_config_[4] = body_ori.y();
    curr_config_[5] = body_ori.z();
    curr_config_[dracobip::num_qdot] = body_ori.w();

    for(int i(0); i<3; ++i)  curr_qdot_[i+3] = body_ang_vel[i];

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

    sp_->Q_ = curr_config_;
    sp_->Qdot_ = curr_qdot_;

    // Right Contact 
    if(data->rfoot_contact) sp_->b_rfoot_contact_ = 1;
    else sp_->b_rfoot_contact_ = 0;
    // Left Contact 
    if(data->lfoot_contact) sp_->b_lfoot_contact_ = 1;
    else sp_->b_lfoot_contact_ = 0;

    //_RBDL_TEST();
    sp_->SaveCurrentData(robot_sys_);
}
void DracoBip_StateEstimator::Update(DracoBip_SensorData* data){
    curr_config_.setZero();
    curr_qdot_.setZero();
    curr_config_[dracobip::num_qdot] = 1.;

    for (int i(0); i<dracobip::num_act_joint; ++i){
        curr_config_[dracobip::num_virtual + i] = data->jpos[i];
        curr_qdot_[dracobip::num_virtual + i] = data->jvel[i];
    }
    std::vector<double> imu_acc(3);
    std::vector<double> imu_ang_vel(3);
 
    for(int i(0); i<3; ++i){
        imu_acc[i]  = data->imu_acc[i];
        imu_ang_vel[i] = data->imu_ang_vel[i];
    }
   
    dynacore::Quaternion body_ori;
    body_ori.w() = 1.; body_ori.x() = 0.; body_ori.y() = 0.; body_ori.z() = 0;
    dynacore::Vect3 body_ang_vel;

    ori_est_->setSensorData( imu_acc, imu_ang_vel);
    ori_est_->getEstimatedState(body_ori, body_ang_vel);

    curr_config_[3] = body_ori.x();
    curr_config_[4] = body_ori.y();
    curr_config_[5] = body_ori.z();
    curr_config_[dracobip::num_qdot] = body_ori.w();

    for(int i(0); i<3; ++i)  curr_qdot_[i+3] = body_ang_vel[i];
    
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

    robot_sys_->UpdateSystem(curr_config_, curr_qdot_);

    sp_->Q_ = curr_config_;
    sp_->Qdot_ = curr_qdot_;

    //dynacore::pretty_print(sp_->Q_, std::cout, "state estimator config");

    // Right Contact 
    if(data->rfoot_contact) sp_->b_rfoot_contact_ = 1;
    else sp_->b_rfoot_contact_ = 0;
    // Left Contact 
    if(data->lfoot_contact) sp_->b_lfoot_contact_ = 1;
    else sp_->b_lfoot_contact_ = 0;
    
    sp_->SaveCurrentData(robot_sys_);
}

void DracoBip_StateEstimator::_RBDL_TEST(){
    // TEST
    dynacore::Vector q(dracobip::num_q); q.setZero();
    dynacore::Vector qdot(dracobip::num_qdot); qdot.setZero();

    q[dracobip_joint::virtual_Rw] = 1.0;
    q[dracobip_joint::lHipYaw] = 1.0;
    //q[dracobip_joint::lHipYaw] = 1.0;
    q[dracobip_joint::virtual_Rw] = 1.0;
    q[dracobip_joint::lAnkle] = M_PI/2.;
    q[dracobip_joint::rAnkle] = M_PI/2.;

    qdot[dracobip_joint::virtual_Ry] = 1.0;
    qdot[dracobip_joint::lAnkle] = 1.0;
    dynacore::Matrix J;
    dynacore::Vector JdotQdot;
    dynacore::Vect3 pos, vel, ang_vel;
     // Orientation
    dynacore::Vect3 rpy; rpy.setZero();
    dynacore::Quaternion quat_floating, link_ori_quat;
    rpy[1] = M_PI/4.;
    dynacore::convert(rpy, quat_floating);

    q[dracobip_joint::virtual_Rx] = quat_floating.x();
    q[dracobip_joint::virtual_Ry] = quat_floating.y();
    q[dracobip_joint::virtual_Rz] = quat_floating.z();
    q[dracobip_joint::virtual_Rw] = quat_floating.w();

    robot_sys_->UpdateSystem(q, qdot);
    int link_idx = dracobip_link::lAnkle;

    robot_sys_->getFullJacobian(link_idx, J);
    robot_sys_->getFullJDotQdot(link_idx, JdotQdot);
    robot_sys_->getOri(link_idx, link_ori_quat);
    robot_sys_->getPos(link_idx, pos);
    robot_sys_->getLinearVel(link_idx, vel);
    robot_sys_->getAngularVel(link_idx, ang_vel);
    
    Eigen::Matrix3d ori_rot(link_ori_quat); 
    dynacore::Matrix ori_rot_mt(6,6); ori_rot_mt.setZero();
    ori_rot_mt.topLeftCorner(3,3) = ori_rot;
    ori_rot_mt.bottomRightCorner(3,3) = ori_rot;

    dynacore::Matrix J_rot = ori_rot_mt.transpose() * J;

    dynacore::pretty_print(q, std::cout, "q");
    dynacore::pretty_print(qdot, std::cout, "qdot");
    dynacore::pretty_print(J, std::cout, "Jacobian");
    dynacore::pretty_print(J_rot, std::cout, "Jacobian Local");
    dynacore::pretty_print(link_ori_quat, std::cout, "ori");
    dynacore::pretty_print(JdotQdot, std::cout, "JdotQdot");
    dynacore::pretty_print(pos, std::cout, "pos");
    dynacore::pretty_print(vel, std::cout, "vel");
    dynacore::pretty_print(ang_vel, std::cout, "ang_vel");
    exit(0);
}

