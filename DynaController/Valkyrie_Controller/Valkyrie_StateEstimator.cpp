#include "Valkyrie_StateEstimator.hpp"
#include "Valkyrie_StateProvider.hpp"
#include <Utils/utilities.hpp>
#include <Valkyrie/Valkyrie_Model.hpp>
#include <Valkyrie_Controller/Valkyrie_DynaControl_Definition.h>
#include <Utils/pseudo_inverse.hpp>
// Orientation Estimators
#include <Valkyrie_Controller/StateEstimator/BasicAccumulation.hpp>

Valkyrie_StateEstimator::Valkyrie_StateEstimator(RobotSystem* robot):
    curr_config_(valkyrie::num_q),
    curr_qdot_(valkyrie::num_qdot)
{
    sp_ = Valkyrie_StateProvider::getStateProvider();
    robot_sys_ = robot;
    ori_est_ = new BasicAccumulation();
}

Valkyrie_StateEstimator::~Valkyrie_StateEstimator(){
    delete ori_est_;
}
void Valkyrie_StateEstimator::_RBDL_TEST(){
    // TEST
    dynacore::Vector q(valkyrie::num_q); q.setZero();
    dynacore::Vector qdot(valkyrie::num_qdot); qdot.setZero();

    q[valkyrie_joint::virtual_Rw] = 1.0;
    
    q[valkyrie_joint::rightHipRoll] = -0.2;
    q[valkyrie_joint::rightHipPitch] = -0.2;
    q[valkyrie_joint::rightKneePitch] = 0.4;
 
    q[valkyrie_joint::leftHipRoll] = -0.2;
    q[valkyrie_joint::leftHipPitch] = -0.2;
    q[valkyrie_joint::leftKneePitch] = 0.4;
    
    // Orientation
    dynacore::Vect3 rpy; rpy.setZero();
    dynacore::Quaternion quat_floating;
    //rpy[1] = M_PI/4.;
    rpy[1] = M_PI/10.;
    dynacore::convert(rpy, quat_floating);

    q[valkyrie_joint::virtual_Rx] = quat_floating.x();
    q[valkyrie_joint::virtual_Ry] = quat_floating.y();
    q[valkyrie_joint::virtual_Rz] = quat_floating.z();
    q[valkyrie_joint::virtual_Rw] = quat_floating.w();
    dynacore::pretty_print(q, std::cout, "q");
    dynacore::pretty_print(qdot, std::cout, "qdot");

    robot_sys_->UpdateSystem(q, qdot);
    //_BasicTest();
    _ProjectionTest();
    exit(0);
}
void Valkyrie_StateEstimator::_pseudoInv(const dynacore::Matrix & J, 
        dynacore::Matrix & J_pinv){

    double threshold(0.00000000001);
    dynacore::Matrix A, Ainv;
    robot_sys_->getMassInertia(A);
    robot_sys_->getInverseMassInertia(Ainv);

    //dynacore::pseudoInverse(J, threshold, J_pinv);

    //A(8,8) *= 100.0;
    //A(11,11) *= 100.0;
    dynacore::Matrix A_einv = A.inverse();
    Ainv = A_einv;
    //dynacore::pretty_print(A, std::cout, "A");
    //dynacore::pretty_print(Ainv, std::cout, "Ainv");

    //dynacore::pretty_print(A_einv,std::cout, "A inv test");
    //dynacore::Matrix test = A*Ainv;
    //dynacore::pretty_print(test,std::cout, "test");

    Ainv.setIdentity();
    //Ainv.block(0,0, 6,6) *= 0.01;
    dynacore::Matrix lambda_inv = J * Ainv * J.transpose();
    dynacore::Matrix lambda;
    dynacore::pseudoInverse(lambda_inv, threshold, lambda);
    J_pinv = Ainv * J.transpose() * lambda;
}
void Valkyrie_StateEstimator::_ProjectionTest(){

    dynacore::Matrix Jbody, Jlfoot, Jrfoot, Jtmp;
    robot_sys_->getFullJacobian(valkyrie_link::pelvis, Jbody);
    robot_sys_->getFullJacobian(valkyrie_link::leftCOP_Frame, Jlfoot);
    robot_sys_->getFullJacobian(valkyrie_link::rightCOP_Frame, Jrfoot);

    // 
    Jbody -= Jlfoot;
    Jbody.setZero();
    Jbody.block(0,3, 3, 3) = Jrfoot.block(0,3, 3,3);
    Jbody.block(3,0, 3,3).setIdentity();
    Jrfoot -= Jbody;

    dynacore::pretty_print(Jbody, std::cout, "Jbody");
    dynacore::Matrix I_mtx(valkyrie::num_qdot, valkyrie::num_qdot);
    I_mtx.setIdentity();

    dynacore::Matrix Jbody_inv, Jlfoot_inv;
    _pseudoInv(Jbody, Jbody_inv);
    _pseudoInv(Jlfoot, Jlfoot_inv);

    dynacore::pretty_print(Jlfoot_inv, std::cout, "Jlfoot inv");
    dynacore::Matrix Nbody = I_mtx - Jbody_inv * Jbody;
    dynacore::Matrix Nlfoot = I_mtx - Jlfoot_inv * Jlfoot;

    dynacore::Matrix nullspace_check_left = Jlfoot_inv * Jlfoot;
    dynacore::pretty_print(nullspace_check_left, std::cout, "null check lfoot");
    dynacore::Matrix JcBody = Jbody * Nlfoot;
    dynacore::Matrix JcBody_inv;
    _pseudoInv(JcBody, JcBody_inv);
    dynacore::Matrix NcBody = I_mtx - JcBody_inv * JcBody;


    dynacore::Matrix JfootBody = Jrfoot * NcBody;

    dynacore::Matrix A, Ainv;
    robot_sys_->getInverseMassInertia(Ainv);
    robot_sys_->getMassInertia(A);
    dynacore::Matrix M_check = A;
    //Eigen::JacobiSVD<dynacore::Matrix> svd(M_check, Eigen::ComputeThinU | Eigen::ComputeThinV);
    //dynacore::pretty_print(svd.singularValues(), std::cout, "svd singular value");


    dynacore::Matrix lambda_inv = JcBody * Ainv * JcBody.transpose();
    dynacore::Matrix lambda;
    dynacore::pseudoInverse(lambda_inv, 0.00001, lambda);
    dynacore::pretty_print(lambda, std::cout, "lambda");
    //dynacore::pretty_print(Jbody, std::cout, "J body");
    //dynacore::pretty_print(Nbody, std::cout, "N body");

    //Nlfoot *= 100000.;
    dynacore::pretty_print(Jlfoot, std::cout, "J lfoot");
    dynacore::pretty_print(Jrfoot, std::cout, "J rfoot");
    dynacore::pretty_print(Nlfoot, std::cout, "Nlfoot");
    dynacore::pretty_print(JcBody, std::cout, "Jc body");
    dynacore::pretty_print(NcBody, std::cout, "Nc body");

    dynacore::pretty_print(JfootBody, std::cout, "Jfootbody");

}

void Valkyrie_StateEstimator::Initialization(Valkyrie_SensorData* data){
    _RBDL_TEST();
    curr_config_.setZero();
    curr_qdot_.setZero();
    curr_config_[valkyrie::num_qdot] = 1.;

    // Joint Set
    for (int i(0); i<valkyrie::num_act_joint; ++i){
        curr_config_[valkyrie::num_virtual + i] = data->jpos[i];
        curr_qdot_[valkyrie::num_virtual + i] = data->jvel[i];
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
    curr_config_[valkyrie::num_qdot] = body_ori.w();

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
}

void Valkyrie_StateEstimator::Update(Valkyrie_SensorData* data){
    curr_config_.setZero();
    curr_qdot_.setZero();
    curr_config_[valkyrie::num_qdot] = 1.;

    for (int i(0); i<valkyrie::num_act_joint; ++i){
        curr_config_[valkyrie::num_virtual + i] = data->jpos[i];
        curr_qdot_[valkyrie::num_virtual + i] = data->jvel[i];
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
    curr_config_[valkyrie::num_qdot] = body_ori.w();

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
}
