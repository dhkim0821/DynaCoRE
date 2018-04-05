#include "LinkOriTask.hpp"
#include "../Valkyrie_StateProvider.hpp"
#include <Utils/utilities.hpp>
#include <Valkyrie/Valkyrie_Model.hpp>


LinkOriTask::LinkOriTask(RobotSystem* robot, int bodyID):Task(3),
                     Kp_vec_(3),
                     Kd_vec_(3),
                     robot_sys_(robot),
                     bodyID_(bodyID)
{
  Kp_vec_.setZero();
  Kd_vec_.setZero();
  for(int i(0); i<dim_task_; ++i){
    Kp_vec_[i] = 20.;
    Kd_vec_[i] = 3.;
  }

  sp_ = Valkyrie_StateProvider::getStateProvider();
  Jt_ = dynacore::Matrix(dim_task_, valkyrie::num_qdot);
  JtDotQdot_ = dynacore::Vector(dim_task_);
}

LinkOriTask::~LinkOriTask(){}

bool LinkOriTask::_UpdateCommand(void* pos_des,
                              const dynacore::Vector & vel_des,
                              const dynacore::Vector & acc_des){
  dynacore::Quaternion* ori_cmd = (dynacore::Quaternion*)pos_des;

  dynacore::Quaternion quat_ori;
  robot_sys_->getOri(bodyID_, quat_ori);
  dynacore::Vect3 ang_vel;
  robot_sys_->getAngularVel(bodyID_, ang_vel);

  dynacore::Quaternion quat_ori_err =  
      dynacore::QuatMultiply(*ori_cmd, quat_ori.inverse());
  dynacore::Vect3 ori_err;
  dynacore::convert(quat_ori_err, ori_err);


  for(int i(0); i<dim_task_; ++i){
      op_cmd_[i] = acc_des[i] 
          + Kp_vec_[i] * ori_err[i] 
          + Kd_vec_[i] * (vel_des[i] - ang_vel[i]);
  }
  //dynacore::pretty_print(acc_des, std::cout, "acc_des");
  //dynacore::pretty_print(op_cmd_, std::cout, "op cmd");
  //dynacore::pretty_print(*ori_cmd, std::cout, "pos cmd");
  //dynacore::pretty_print(quat_ori, std::cout, "quat ori");
  //dynacore::pretty_print(ori_err, std::cout, "ori_err");
    //dynacore::pretty_print(ang_vel, std::cout, "ang_vel");
    //dynacore::pretty_print(sp_->q_, std::cout, "q");
    //dynacore::pretty_print(sp_->qdot_, std::cout, "qdot");
  return true;
}

bool LinkOriTask::_UpdateTaskJacobian(){
    dynacore::Matrix Jtmp;
    robot_sys_->getFullJacobian(bodyID_, Jtmp);
    Jt_ = Jtmp.block(0,0, 3, valkyrie::num_qdot);

    return true;
}

bool LinkOriTask::_UpdateTaskJDotQdot(){
  dynacore::Matrix Jtmp;
  robot_sys_->getFullJacobianDot(bodyID_, Jtmp);
  JtDotQdot_ = Jtmp.block(0,0, 3, valkyrie::num_qdot) * sp_->qdot_;
  return true;
}
