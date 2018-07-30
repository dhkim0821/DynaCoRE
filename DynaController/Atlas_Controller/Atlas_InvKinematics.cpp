#include "Atlas_InvKinematics.hpp"
#include <Atlas/Atlas_Definition.h>
#include <Atlas/Atlas_Model.hpp>
#include <rbdl/urdfreader.h>
#include <Configuration.h>
#include <Utils/utilities.hpp>
#include <Utils/pseudo_inverse.hpp>
#include <Eigen/QR>

using namespace RigidBodyDynamics;

Atlas_InvKinematics::Atlas_InvKinematics():Atlas_InvKinematics(0, 0) {
}

Atlas_InvKinematics::Atlas_InvKinematics(int swing_foot_id, int stance_foot_id):
    max_iter_(100), 
    swing_foot_id_(swing_foot_id), 
    stance_foot_id_(stance_foot_id)
{
    body_id_ = atlas_link::torso;
    pelvis_id_ = atlas_link::pelvis;
    switch(swing_foot_id){
        case atlas_link::leftFoot:
            swing_leg_first_jidx_ = atlas_joint::l_leg_hpz;
            break;
        case atlas_link::rightFoot:
            swing_leg_first_jidx_ = atlas_joint::r_leg_hpz;
            break;
        case 0: // No swing leg setup case
            break;
        default:
            printf("[Inverse Kinematics] Not valid link id\n");
            exit(0);
    }
    model_ = new Atlas_Model();
}

Atlas_InvKinematics::~Atlas_InvKinematics(){  delete model_;  }

void Atlas_InvKinematics::getLegConfigAtVerticalPosture(
        const dynacore::Vect3 & target_foot_pos,
        const dynacore::Vector & guess_Q, 
        dynacore::Vector & config_sol){

    if(swing_foot_id_ == 0){ 
        printf("[Inv kin] No swing foot definition\n"); 
        exit(0);
    }
    // Vertial Orientation
    config_sol = guess_Q;
    dynacore::Vector zero_qdot(atlas::num_qdot);
    zero_qdot.setZero();

    config_sol[atlas_joint::virtual_Rx] = 0.;
    config_sol[atlas_joint::virtual_Ry] = 0.;
    config_sol[atlas_joint::virtual_Rz] = 0.;
    config_sol[atlas_joint::virtual_Rw] = 1.;
    model_->UpdateSystem(guess_Q, zero_qdot);
    
    // Set parameters up
    int num_iter(0);
    double err(1000.0);
    double pre_err = err + 1000.;
    dynacore::Vector err_vec(6);
    dynacore::Quaternion curr_quat;
    dynacore::Quaternion des_quat;
    des_quat.w() = 1.; des_quat.x() = 0.; des_quat.y() = 0.; des_quat.z() = 0.;

    dynacore::Quaternion err_quat;
    dynacore::Vect3 err_ori_vec;
    dynacore::Vector delta_q;
    dynacore::Vect3 current_pos;
    dynacore::Matrix J_leg, Jinv;
    // search
    while((num_iter < max_iter_) && (err > 0.00001)){  // && (pre_err > err)){
        pre_err = err;
        model_->getOri(swing_foot_id_, curr_quat);
        err_quat = dynacore::QuatMultiply(des_quat, curr_quat.inverse());
        dynacore::convert(err_quat, err_ori_vec);
        model_->getPos(swing_foot_id_, current_pos);
        // Error Setup
        err_vec.head(3) = err_ori_vec;
        err_vec.tail(3) = (target_foot_pos - current_pos);
        
        model_->getFullJacobian(swing_foot_id_, J_leg);
        J_leg.block(0,0, 6,6) = dynacore::Matrix::Zero(6,6);
        dynacore::pseudoInverse(J_leg, 0.0001, Jinv);
 
        //dynacore::pretty_print(J_leg, std::cout, "J leg");
        //dynacore::pretty_print(config_sol, std::cout, "config_sol");
        //dynacore::pretty_print(delta_q, std::cout, "delta q");
        //dynacore::pretty_print(err_vec, std::cout, "error vector");
        
        delta_q = Jinv *  err_vec;
        config_sol.segment(swing_leg_first_jidx_, atlas::num_leg_joint) 
            +=  delta_q.segment(swing_leg_first_jidx_, atlas::num_leg_joint);
        model_->UpdateSystem(config_sol, zero_qdot);

        ++num_iter;
        err = err_vec.norm();

       if(num_iter == max_iter_){
            printf("%d iter, err: %f\n", num_iter, err);
        }
    }
}

void Atlas_InvKinematics::getDoubleSupportLegConfig(
        const dynacore::Vector & current_Q,
        const dynacore::Quaternion & des_quat,
        const double & des_height, dynacore::Vector & config_sol){

    config_sol = current_Q;
    dynacore::Vector zero_qdot = dynacore::Vector::Zero(atlas::num_qdot);
    // Update internal model
    model_->UpdateSystem(config_sol, zero_qdot);

    int lfoot_id = atlas_link::leftFoot;
    int rfoot_id = atlas_link::rightFoot;

    dynacore::Matrix full_J;
    dynacore::Matrix J_left_foot, J_right_foot;
    
    model_->getFullJacobian(lfoot_id, J_left_foot);
    model_->getFullJacobian(rfoot_id, J_right_foot);

    // Stack the Constraint Jacobians
    dynacore::Matrix J1 = dynacore::Matrix::Zero(12, atlas::num_qdot);
    J1.block(0,0, 6, atlas::num_qdot) = J_left_foot;
    J1.block(6,0, 6, atlas::num_qdot) = J_right_foot;

    // Create the Nullspace
    dynacore::Matrix J1_pinv;
    double threshold = 0.00001;
    dynacore::pseudoInverse(J1, threshold, J1_pinv);
    dynacore::Matrix N1 = 
        dynacore::Matrix::Identity(atlas::num_qdot, atlas::num_qdot) - 
        J1_pinv*J1;

    // Get the Jacobian of the body and pelvis
    dynacore::Matrix J_body, J_pelvis;
    model_->getFullJacobian(body_id_, J_body);
    model_->getFullJacobian(pelvis_id_, J_pelvis);

    // Get the Jacobian rows for Body Roll, Pitch and Height, Pelvis Roll Pitch, Yaw
    int dim_ctrl(7);
    dynacore::Matrix J2 = dynacore::Matrix::Zero(dim_ctrl, atlas::num_qdot);
    J2(0, 2) = 1.;
    J2.block(1,0, 3, atlas::num_qdot) = J_body.block(0, 0, 3, atlas::num_qdot);
    J2.block(4,0, 3, atlas::num_qdot) = J_pelvis.block(0, 0, 3, atlas::num_qdot);
    
    //// Orientation Error Computation
    // Body ori err
    dynacore::Vect3 body_ori_err, pelvis_ori_err;
    dynacore::Quaternion curr_quat, err_quat;

    model_->getOri(body_id_, curr_quat);
    err_quat = dynacore::QuatMultiply(des_quat, curr_quat.inverse());
    dynacore::convert(err_quat, body_ori_err);
    // Pelvis ori err
    model_->getOri(pelvis_id_, curr_quat);
    err_quat = dynacore::QuatMultiply(des_quat, curr_quat.inverse());
    dynacore::convert(err_quat, pelvis_ori_err);

    // Compute Height Error
    double height_error = des_height - current_Q[2];
    
    // Construct the operational space error
    dynacore::Vector delta_x = dynacore::Vector::Zero(dim_ctrl);
    delta_x[0] = height_error;
    delta_x[1] = body_ori_err[0];   
    delta_x[2] = body_ori_err[1];
    delta_x[3] = body_ori_err[2];
    delta_x[4] = pelvis_ori_err[0];
    delta_x[5] = pelvis_ori_err[1];
    delta_x[6] = pelvis_ori_err[2];

    dynacore::Matrix J2N1 = J2*N1;
    dynacore::Matrix J2N1_pinv;
    dynacore::pseudoInverse(J2N1, threshold, J2N1_pinv);
    dynacore::Vector delta_q(atlas::num_qdot); delta_q.setZero();
    delta_q = J2N1_pinv*delta_x;
    
    config_sol.segment(atlas::num_virtual, atlas::num_act_joint) 
        += delta_q.segment(atlas::num_virtual, atlas::num_act_joint);
    
    //dynacore::pretty_print(config_sol, std::cout, "config_sol");
    //dynacore::pretty_print(J1, std::cout, "J1");
    //dynacore::pretty_print(J2N1, std::cout, "J2N1");
    //dynacore::pretty_print(J2N1_pinv, std::cout, "J2N1_pinv");
    //dynacore::pretty_print(delta_x, std::cout, "delta x");
    //dynacore::pretty_print(delta_q, std::cout, "delta q");
}

void Atlas_InvKinematics::getSingleSupportStanceLegConfiguration(
        const dynacore::Vector & current_Q,
        const dynacore::Quaternion & des_quat,
        const double & des_height, 
        dynacore::Vector & config_sol){

    if(swing_foot_id_ == 0){ 
        printf("[Inv Kin] No swing foot definition\n"); 
        exit(0);
    }

    config_sol = current_Q;
    dynacore::Vector zero_qdot = dynacore::Vector::Zero(atlas::num_qdot);
    model_->UpdateSystem(config_sol, zero_qdot);

    // contact jacobian and null space
    dynacore::Matrix Jc, J_full;
    model_->getFullJacobian(stance_foot_id_, Jc);
    
    dynacore::Matrix Jc_pinv;
    double threshold = 0.0000001;
    dynacore::pseudoInverse(Jc, threshold, Jc_pinv);
    
    dynacore::Matrix Nc = 
        dynacore::Matrix::Identity(atlas::num_qdot, atlas::num_qdot) -
        Jc_pinv * Jc;

    //// Operational Space error (height, Rx, Ry, pelvis Rx, pelvis Ry) *****
    int ctrl_dim(5);
    dynacore::Vector body_err(ctrl_dim);
    dynacore::Matrix Jop(ctrl_dim, atlas::num_qdot); Jop.setZero();

    // Height
    body_err[0] = des_height - current_Q[atlas_joint::virtual_Z];
    
    // Orientation
    dynacore::Quaternion curr_quat;
    dynacore::Vect3 body_ori_err, pelvis_ori_err;
    model_->getOri(body_id_, curr_quat);

    dynacore::Quaternion err_quat = dynacore::QuatMultiply(des_quat, curr_quat.inverse());
    dynacore::convert(err_quat, body_ori_err);

    model_->getOri(atlas_link::pelvis, curr_quat);
    err_quat = dynacore::QuatMultiply(des_quat, curr_quat.inverse());
    dynacore::convert(err_quat, pelvis_ori_err);
    for(int i(0); i<2; ++i){
        body_err[1+i] = body_ori_err[i];
        body_err[3+i] = pelvis_ori_err[i];
    }
  
    // Jacobian 
    dynacore::Matrix J_body, J_pelvis;
    model_->getFullJacobian(body_id_, J_body);
    model_->getFullJacobian(atlas_link::pelvis, J_pelvis);
    
    Jop(0, 2) = 1.; // Height
    Jop.block(1, 0, 2, atlas::num_qdot) = J_body.block(0,0, 2, atlas::num_qdot);
    Jop.block(3, 0, 2, atlas::num_qdot) = J_pelvis.block(0,0, 2, atlas::num_qdot);
    
    dynacore::Matrix JNc = Jop * Nc;
    dynacore::Matrix JNc_pinv;
    dynacore::pseudoInverse(JNc, threshold, JNc_pinv);
    dynacore::Vector qdelta = JNc_pinv * body_err;

    config_sol.segment(atlas::num_virtual, atlas::num_act_joint) += 
        qdelta.segment(atlas::num_virtual, atlas::num_act_joint);

    //dynacore::pretty_print(qdelta, std::cout, "delta");
    //dynacore::pretty_print(current_Q, std::cout, "Current Q");
    //dynacore::pretty_print(Jop, std::cout, "Jop");
    //dynacore::pretty_print(Jfoot, std::cout, "Jfoot");
    if(isnan(current_Q[1])){ exit(0); }
}

