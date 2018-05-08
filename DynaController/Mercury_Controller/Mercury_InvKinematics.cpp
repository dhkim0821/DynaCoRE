#include "Mercury_InvKinematics.hpp"
#include <Mercury/Mercury_Definition.h>
#include <rbdl/urdfreader.h>
#include <Configuration.h>
#include <Utils/utilities.hpp>

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


