#include "Mercury_InvKinematics.hpp"
#include <Mercury/Mercury_Definition.h>
#include <rbdl/urdfreader.h>
#include <Configuration.h>
#include <Utils/utilities.hpp>
#include <Utils/pseudo_inverse.hpp>

using namespace RigidBodyDynamics;

Mercury_InvKinematics::Mercury_InvKinematics():max_iter_(1000)
{
  model_ = new Model();
  rbdl_check_api_version (RBDL_API_VERSION);

  if (!Addons::URDFReadFromFile (THIS_COM"/RobotSystems/Mercury/mercury.urdf", model_, false)) {
    std::cerr << "Error loading model ./mercury.urdf" << std::endl;
    abort();
  }
}

Mercury_InvKinematics::~Mercury_InvKinematics(){
    delete model_;
}

void Mercury_InvKinematics::getFootPosAtVerticalPosture(
        int link_id, const dynacore::Vect3 & leg_config,
        const dynacore::Vector & guess_Q, dynacore::Vect3 & foot_pos){

    dynacore::Vector config = guess_Q;
    config[mercury_joint::virtual_Rx] = 0.;
    config[mercury_joint::virtual_Ry] = 0.;
    config[mercury_joint::virtual_Rz] = 0.;
    config[mercury_joint::virtual_Rw] = 1.;

    // Find Foot body id
    unsigned int bodyid;
    int leg_jidx;
    switch(link_id){
        case mercury_link::leftFoot:
            bodyid = model_->GetBodyId("lfoot");
            leg_jidx = mercury_joint::leftAbduction;
            break;
        case mercury_link::rightFoot:
            bodyid = model_->GetBodyId("rfoot");
            leg_jidx = mercury_joint::rightAbduction;
            break;
        default:
            printf("[Inverse Kinematics] Not valid link id\n");
            exit(0);
    }

    for(int i(0); i<3; ++i)  config[leg_jidx + i] = leg_config[i];

    // Set parameters up
    dynacore::Vect3 Local_CoM= model_->mFixedBodies[
        bodyid - model_->fixed_body_discriminator].mCenterOfMass;

    foot_pos = CalcBodyToBaseCoordinates(
            *model_, config, bodyid, Local_CoM, true);
}


void Mercury_InvKinematics::getLegConfigAtVerticalPosture(
        int link_id, const dynacore::Vect3 & target_foot_pos,
        const dynacore::Vector & guess_Q, dynacore::Vector & config_sol){
    // Vertial Orientation
    config_sol = guess_Q;
    config_sol[mercury_joint::virtual_Rx] = 0.;
    config_sol[mercury_joint::virtual_Ry] = 0.;
    config_sol[mercury_joint::virtual_Rz] = 0.;
    config_sol[mercury_joint::virtual_Rw] = 1.;

    // Find Foot body id
    unsigned int bodyid;
    int leg_jidx;
    switch(link_id){
        case mercury_link::leftFoot:
            bodyid = model_->GetBodyId("lfoot");
            leg_jidx = mercury_joint::leftAbduction;
            break;
        case mercury_link::rightFoot:
            bodyid = model_->GetBodyId("rfoot");
            leg_jidx = mercury_joint::rightAbduction;
            break;
        default:
            printf("[Inverse Kinematics] Not valid link id\n");
            exit(0);
    }

    // Set parameters up
    dynacore::Vect3 Local_CoM= model_->mFixedBodies[
        bodyid - model_->fixed_body_discriminator].mCenterOfMass;
    int num_iter(0);
    double err(1000.0);
    double pre_err = err + 1000.;
    dynacore::Vect3 err_vec;
    dynacore::Vector delta_q;
    dynacore::Vect3 current_pos;
    dynacore::Matrix J, J_leg, Jinv;


    J = dynacore::Matrix::Zero(3, mercury::num_qdot);
    // search
    while((num_iter < max_iter_) && (err > 0.000001)){  // && (pre_err > err)){
        pre_err = err;
        current_pos = CalcBodyToBaseCoordinates(
                *model_, config_sol, bodyid, Local_CoM, true);
        err_vec = (target_foot_pos - current_pos);

        CalcPointJacobian(*model_, config_sol, bodyid, Local_CoM, J, false);
        J_leg = J.block(0, leg_jidx, 3, 3);
        //dynacore::pretty_print(J, std::cout, "J");
        //dynacore::pretty_print(J_leg, std::cout, "J leg");
        //dynacore::pretty_print(target_foot_pos, std::cout, "target foot pos");
        //dynacore::pretty_print(current_pos, std::cout, "current pos");
        //dynacore::pretty_print(Local_CoM, std::cout, "local com");
        Jinv = J_leg.inverse();
        delta_q = Jinv *  err_vec;
        config_sol.segment(leg_jidx, 3) +=  delta_q;

        ++num_iter;
        err = err_vec.norm();
        
            //printf("%d iter, err: %f\n", num_iter, err);
        if(num_iter == max_iter_){
            printf("%d iter, err: %f\n", num_iter, err);
        }
    }
}

void Mercury_InvKinematics::getDoubleSupportLegConfig(const dynacore::Vector & current_Q, 
                                                      const dynacore::Quaternion & des_quat,
                                                      const double & des_height, dynacore::Vector & config_sol){
    config_sol = current_Q;

    // Find Foot body id
    unsigned int left_foot_bodyid = model_->GetBodyId("lfoot");;
    unsigned int right_foot_bodyid = model_->GetBodyId("rfoot");;
    unsigned int body_bodyid = model_->GetBodyId("body");

    int left_leg_jidx = mercury_joint::leftAbduction;
    int right_leg_jidx = mercury_joint::rightAbduction;

    // Initialize Parameters
    dynacore::Vect3 left_foot_local_com = model_->mFixedBodies[left_foot_bodyid - model_->fixed_body_discriminator].mCenterOfMass;
    dynacore::Vect3 right_foot_local_com = model_->mFixedBodies[right_foot_bodyid - model_->fixed_body_discriminator].mCenterOfMass;    
    //dynacore::Vect3 body_local_com = model_->mFixedBodies[body_bodyid - model_->fixed_body_discriminator].mCenterOfMass;    
    dynacore::Vect3 body_local_com;
    body_local_com.setZero();

    // Get the Jacobians of the foot
    dynacore::Matrix J_left_foot = dynacore::Matrix::Zero(3, mercury::num_qdot);
    dynacore::Matrix J_right_foot = dynacore::Matrix::Zero(3, mercury::num_qdot);    

    CalcPointJacobian(*model_, current_Q, left_foot_bodyid, left_foot_local_com, J_left_foot, true);
    CalcPointJacobian(*model_, current_Q, right_foot_bodyid, right_foot_local_com, J_right_foot, false);

    dynacore::pretty_print(current_Q, std::cout, "current_Q");
    dynacore::pretty_print(J_left_foot, std::cout, "J left foot");
    dynacore::pretty_print(J_right_foot, std::cout, "J right foot");    

    // Stack the Constraint Jacobians
    dynacore::Matrix J1 = dynacore::Matrix::Zero(6, mercury::num_qdot);
    J1.block(0,0, 3, mercury::num_qdot) = J_left_foot;
    J1.block(3,0, 3, mercury::num_qdot) = J_right_foot;
    //dynacore::pretty_print(J1, std::cout, "J1");

    // Create the Nullspace
    // dynacore::Matrix J1_pinv;
    // double threshold = 0.001;
    // dynacore::pseudoInverse(J1, threshold, J1_pinv);
    // dynacore::Matrix N1 = dynacore::Matrix::Identity(mercury::num_qdot, mercury::num_qdot) - J1_pinv*J1;
    // dynacore::pretty_print(N1, std::cout, "N1");


    // Get the Jacobian of the body
    dynacore::Matrix J_body = dynacore::Matrix::Zero(6, mercury::num_qdot);
    //dynacore::pretty_print(body_local_com, std::cout, "body_local_com");

    //CalcPointJacobian6D(*model_, current_Q, body_bodyid, body_local_com, J_body, false);   
    J_body.block(0,0,6,6) = dynacore::Matrix::Identity(6,6);

    dynacore::pretty_print(J_body, std::cout, "body Jacobian");

    // Get the Jacobian rows for Body Roll, Pitch and Height
    dynacore::Matrix J2 = dynacore::Matrix::Zero(3, mercury::num_qdot);
    J2.block(0, 0, 2, mercury::num_qdot) = J_body.block(0, 0, 2, mercury::num_qdot);  // (Rx, Ry) Roll and Pitch  
    J2.block(2, 0, 1, mercury::num_qdot) = J_body.block(5, 0, 1, mercury::num_qdot);  //  Z - body height
    dynacore::pretty_print(J2, std::cout, "J2");

    //Compute Orientation Error
    // Orientation
    dynacore::Quaternion curr_quat;
    curr_quat.w() = current_Q[mercury::num_qdot];
    curr_quat.x() = current_Q[3];
    curr_quat.y() = current_Q[4];
    curr_quat.z() = current_Q[5];

    dynacore::Quaternion err_quat = dynacore::QuatMultiply(des_quat, curr_quat.inverse());
    dynacore::Vect3 ori_err;
    dynacore::convert(err_quat, ori_err);

    // dynacore::pretty_print(curr_quat, std::cout, "curr_quat");
    // dynacore::pretty_print(des_quat, std::cout, "des_quat");    
    // dynacore::pretty_print(err_quat, std::cout, "err_quat");    
    // dynacore::pretty_print(ori_err, std::cout, "ori_err");

    // Compute Height Error
    double current_height = current_Q[2];
    double height_error = des_height - current_height;
    // Construct the operational space error
    dynacore::Vector delta_x = dynacore::Vector::Zero(3);
    delta_x[0] = ori_err[0];    
    delta_x[1] = ori_err[1];
    delta_x[2] = height_error;
    //dynacore::pretty_print(delta_x, std::cout, "delta_x");

    // delta_q = pinv(J2N1)*delta_x
}


