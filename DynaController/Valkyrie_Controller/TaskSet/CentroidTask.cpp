#include "CentroidTask.hpp"
#include "../Valkyrie_StateProvider.hpp"
#include <Utils/utilities.hpp>
#include <Valkyrie/Valkyrie_Model.hpp>


CentroidTask::CentroidTask(RobotSystem* robot):Task(6),
                     Kp_vec_(6),
                     Kd_vec_(6),
                     robot_sys_(robot)
{
  Kp_vec_.setZero();
  Kd_vec_.setZero();
  for(int i(0); i<dim_task_; ++i){
    Kp_vec_[i] = 200.;
    Kd_vec_[i] = 5.;
  }
  for(int i(0); i<3; ++i)  Kp_vec_[i] = 0.; // No centroid angular pos ctrl

  sp_ = Valkyrie_StateProvider::getStateProvider();
  Jt_ = dynacore::Matrix(dim_task_, valkyrie::num_qdot);
  JtDotQdot_ = dynacore::Vector(dim_task_);
}

CentroidTask::~CentroidTask(){}

bool CentroidTask::_UpdateCommand(void* pos_des,
                              const dynacore::Vector & vel_des,
                              const dynacore::Vector & acc_des){
  dynacore::Vector* pos_cmd = (dynacore::Vector*)pos_des;

  dynacore::Vect3 com_pos;
  dynacore::Vector ctr_vel;
  robot_sys_->getCoMPosition(com_pos);
  robot_sys_->getCentroidVelocity(ctr_vel);

  // Centroidal Angular Control
  for(int i(0); i<3; ++i){
    op_cmd_[i] = acc_des[i] 
        + Kd_vec_[i] * (vel_des[i] - ctr_vel[i]);
  }
  for(int i(3); i<6; ++i){
      op_cmd_[i] = acc_des[i] 
        + Kp_vec_[i] * ((*pos_cmd)[i] - com_pos[i-3])
        + Kd_vec_[i] * (vel_des[i] - ctr_vel[i]);
  }
 // dynacore::pretty_print(acc_des, std::cout, "acc_des");
  // dynacore::pretty_print(op_cmd_, std::cout, "op cmd");
   //dynacore::pretty_print(*pos_cmd, std::cout, "pos cmd");
   //dynacore::pretty_print(com_pos, std::cout, "com pos");

  return true;
}

bool CentroidTask::_UpdateTaskJacobian(){
  Jt_.setZero();
  robot_sys_->getCentroidJacobian(Jt_);

  dynacore::Matrix Icm;
  robot_sys_->getCentroidInertia(Icm);
  //dynacore::pretty_print(Icm, std::cout, "Icm");
  return true;
}

bool CentroidTask::_UpdateTaskJDotQdot(){
  JtDotQdot_.setZero();
  dynacore::Matrix Ainv, Jcm;
  dynacore::Vector cori;
  robot_sys_->getInverseMassInertia(Ainv);
  robot_sys_->getCoriolis(cori);
  robot_sys_->getCentroidJacobian(Jcm);
  JtDotQdot_ = Jcm * Ainv * cori;
  return true;
}
