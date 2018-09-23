#include "FootTask.hpp"
#include <Configuration.h>
#include <DracoBip_Controller/DracoBip_StateProvider.hpp>
#include <DracoBip/DracoBip_Model.hpp>
#include <DracoBip/DracoBip_Definition.h>

#include <Utils/utilities.hpp>

FootTask::FootTask(const RobotSystem* robot, int swing_foot):
    KinTask(3),
    robot_sys_(robot),
    swing_foot_(swing_foot)
{
    sp_ = DracoBip_StateProvider::getStateProvider();
    Jt_ = dynacore::Matrix::Zero(dim_task_, dracobip::num_qdot);
    if(swing_foot_ == dracobip_link::lAnkle) stance_foot_ = dracobip_link::rAnkle;
    if(swing_foot_ == dracobip_link::rAnkle) stance_foot_ = dracobip_link::lAnkle;
    // printf("[BodyFoot Task] Constructed\n");
}

FootTask::~FootTask(){}

bool FootTask::_UpdateCommand(void* pos_des,
        const dynacore::Vector & vel_des,
        const dynacore::Vector & acc_des){
    dynacore::Vect3* pos_cmd = (dynacore::Vect3*)pos_des;
    
    dynacore::Vect3 foot_pos;
    robot_sys_->getPos(swing_foot_, foot_pos);
 
    for(int i(0); i<dim_task_; ++i){
        pos_err_[i] = 10. * ((*pos_cmd)[i] - foot_pos[i] );
        vel_des_[i] = vel_des[i];
        acc_des_[i] = acc_des[i];
    }

     //dynacore::pretty_print(pos_err_, std::cout, "pos err");
     //dynacore::pretty_print(acc_des, std::cout, "acc des");
     //dynacore::pretty_print(vel_des, std::cout, "vel des");

    return true;
}

bool FootTask::_UpdateTaskJacobian(){
    dynacore::Matrix Jswing, Jstance;
    robot_sys_->getFullJacobian(swing_foot_, Jswing);
    robot_sys_->getFullJacobian(stance_foot_, Jstance);
    //Jt_ = Jswing.block(3,0,3, dracobip::num_qdot) - Jstance.block(3, 0, 3, dracobip::num_qdot);
    Jt_.block(0, 0, 3, dracobip::num_qdot) = Jswing.block(3,0,3, dracobip::num_qdot);
    (Jt_.block(0, 0, 3, dracobip::num_virtual)).setZero();
    
    // dynacore::pretty_print(Jswing, std::cout, "Jswing");
    // dynacore::pretty_print(Jfoot, std::cout, "Jfoot");
    // dynacore::pretty_print(Jt_, std::cout, "Jt BodyFoot");
    return true;
}

bool FootTask::_UpdateTaskJDotQdot(){
    dynacore::Vector Jdotqdot;
    robot_sys_->getFullJDotQdot(swing_foot_, Jdotqdot);
    JtDotQdot_ = Jdotqdot.tail(3);

    return true;
}

