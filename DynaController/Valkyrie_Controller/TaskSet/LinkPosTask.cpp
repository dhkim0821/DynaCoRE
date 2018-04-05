#include "LinkPosTask.hpp"
#include "../Valkyrie_StateProvider.hpp"
#include <Utils/utilities.hpp>
#include <Valkyrie/Valkyrie_Model.hpp>


LinkPosTask::LinkPosTask(RobotSystem* robot, int bodyID):Task(3),
                     Kp_vec_(3),
                     Kd_vec_(3),
                     robot_sys_(robot),
                     bodyID_(bodyID)
{
  Kp_vec_.setZero();
  Kd_vec_.setZero();
  for(int i(0); i<dim_task_; ++i){
    Kp_vec_[i] = 250.;
    Kd_vec_[i] = 25.;
  }

  sp_ = Valkyrie_StateProvider::getStateProvider();
  Jt_ = dynacore::Matrix(dim_task_, valkyrie::num_qdot);
  JtDotQdot_ = dynacore::Vector(dim_task_);
}

LinkPosTask::~LinkPosTask(){}

bool LinkPosTask::_UpdateCommand(void* pos_des,
                              const dynacore::Vector & vel_des,
                              const dynacore::Vector & acc_des){
  dynacore::Vector* pos_cmd = (dynacore::Vector*)pos_des;

  dynacore::Vect3 link_pos, link_vel;
  robot_sys_->getPos(bodyID_, link_pos);
  robot_sys_->getLinearVel(bodyID_, link_vel);

  for(int i(0); i<dim_task_; ++i){
      op_cmd_[i] = acc_des[i] 
          + Kp_vec_[i] * ((*pos_cmd)[i] - link_pos[i])
          + Kd_vec_[i] * (vel_des[i] - link_vel[i]);
  }

  //dynacore::pretty_print(acc_des, std::cout, "acc_des");
  //dynacore::pretty_print(op_cmd_, std::cout, "op cmd");
  //dynacore::pretty_print(*pos_cmd, std::cout, "pos cmd");
  //dynacore::pretty_print(link_pos, std::cout, "link pos");
  //dynacore::pretty_print(link_vel, std::cout, "link_vel");
  //dynacore::pretty_print(sp_->q_, std::cout, "q");
  //dynacore::pretty_print(sp_->qdot_, std::cout, "qdot");
  return true;
}

bool LinkPosTask::_UpdateTaskJacobian(){
    dynacore::Matrix Jtmp;
    robot_sys_->getFullJacobian(bodyID_, Jtmp);
    Jt_ = Jtmp.block(3,0, 3, valkyrie::num_qdot);

    return true;
}

bool LinkPosTask::_UpdateTaskJDotQdot(){
    dynacore::Matrix Jtmp;
    robot_sys_->getFullJacobianDot(bodyID_, Jtmp);
    JtDotQdot_ = Jtmp.block(3,0, 3, valkyrie::num_qdot) * sp_->qdot_;
    return true;
}
