#include "FootTask.hpp"
#include <Configuration.h>
#include <Mercury_Controller/Mercury_StateProvider.hpp>
#include <Mercury/Mercury_Model.hpp>
#include <Mercury/Mercury_Definition.h>

#include <Utils/utilities.hpp>

FootTask::FootTask(RobotSystem* robot, int swing_foot):
    KinTask(3),
    robot_sys_(robot),
    swing_foot_(swing_foot)
{
    sp_ = Mercury_StateProvider::getStateProvider();
    Jt_ = dynacore::Matrix::Zero(dim_task_, mercury::num_qdot);
    // printf("[BodyFoot Task] Constructed\n");
}

FootTask::~FootTask(){}

bool FootTask::_UpdateCommand(void* pos_des,
        const dynacore::Vector & vel_des,
        const dynacore::Vector & acc_des){

    dynacore::Vector* pos_cmd = (dynacore::Vector*)pos_des;
    
    dynacore::Vect3 foot_pos;
    robot_sys_->getPos(swing_foot_, foot_pos);
 
    for(int i(0); i<dim_task_; ++i){
        pos_err_[i] = (*pos_cmd)[i] - foot_pos[i];
        vel_des_[i] = vel_des[i];
        acc_des_[i] = acc_des[i];
    }

    // dynacore::pretty_print(op_cmd_, std::cout, "op cmd");
    // dynacore::pretty_print(*pos_cmd, std::cout, "pos cmd");
    // dynacore::pretty_print(foot_pos, std::cout, "foot pos");
    // dynacore::pretty_print(acc_des, std::cout, "acc des");
    // dynacore::pretty_print(vel_des, std::cout, "vel des");

    return true;
}

bool FootTask::_UpdateTaskJacobian(){
    dynacore::Matrix Jswing;
    robot_sys_->getFullJacobian(swing_foot_, Jswing);
    Jt_.block(0, 0, 3, mercury::num_qdot) = Jswing.block(3,0,3, mercury::num_qdot);

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

