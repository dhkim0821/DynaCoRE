#include "Mercury_Dyn_environment.hpp"
#include "common/utils.h"
#include <Utils/wrap_eigen.hpp>
#include <Utils/utilities.hpp>
#include <ControlSystem/Mercury/Mercury_Controller/interface.hpp>
#include <ControlSystem/Mercury/Mercury_Controller/StateProvider.hpp>
#include <ParamHandler/ParamHandler.hpp>

// #define SENSOR_NOISE
#define SENSOR_DELAY 0 // Sensor_delay* SERVO_RATE (sec) = time delay 


Mercury_Dyn_environment::Mercury_Dyn_environment():
  num_substep_rendering_(15),
  count_(0)
{
  _ParamterSetup();

  m_Mercury = new Mercury(Vec3(0.0, 0.0, 0.0), srSystem::FIXED, srJoint::TORQUE);
  m_Space = new srSpace();
  m_ground = new Ground();

  interface_ = new interface();

  m_Space->AddSystem(m_ground->BuildGround());
  m_Space->AddSystem((srSystem*)m_Mercury);
  m_Space->DYN_MODE_PRESTEP();

  m_Space->SET_USER_CONTROL_FUNCTION_2(ContolFunction);
  m_Space->SetTimestep(SERVO_RATE);
  m_Space->SetGravity(0.0,0.0,-9.81);
  m_Space->SetNumberofSubstepForRendering(num_substep_rendering_);
}

void Mercury_Dyn_environment::ContolFunction( void* _data ) {
  Mercury_Dyn_environment* pDyn_env = (Mercury_Dyn_environment*)_data;
  Mercury* robot = pDyn_env->m_Mercury;
  ++pDyn_env->count_ ;
  double alternate_time = SIM_SERVO_RATE * pDyn_env->count_;
  std::vector<double> jpos(6);
  std::vector<double> mjvel(6);
  std::vector<double> jjvel(6);
  std::vector<double> jtorque(6);
  std::vector<double> imu_acc(3);
  std::vector<double> imu_ang_vel(3);
  std::vector<double> imu_inc(3);
  bool rfoot_contact(false);
  bool lfoot_contact(false);
  std::vector<double> torque_command(robot->num_act_joint_, 0.);

  pDyn_env->getIMU_Data(imu_acc, imu_ang_vel);

  int lj_start_idx(4);
  // Right
  for(int i(0); i< 3; ++i){
    jpos[i] = robot->r_joint_[i]->m_State.m_rValue[0];
    mjvel[i] = robot->r_joint_[i]->m_State.m_rValue[1];
    jjvel[i] = robot->r_joint_[i]->m_State.m_rValue[1];
    jtorque[i] = robot->r_joint_[i]->m_State.m_rValue[3];
  }
  // Left
  for(int i(0); i< 3; ++i){
    jpos[i+3] = robot->r_joint_[i+lj_start_idx]->m_State.m_rValue[0];
    mjvel[i+3] = robot->r_joint_[i+lj_start_idx]->m_State.m_rValue[1];
    jjvel[i] = robot->r_joint_[i]->m_State.m_rValue[1];
    jtorque[i+3] = robot->r_joint_[i+lj_start_idx]->m_State.m_rValue[3];
  }

  pDyn_env->interface_->GetCommand(alternate_time, jpos, mjvel, jjvel, jtorque, imu_acc, imu_ang_vel, imu_acc, rfoot_contact, lfoot_contact, jtorque, torque_command);

  for(int i(0); i<3; ++i){
    robot->vp_joint_[i]->m_State.m_rCommand = 0.0;
    robot->vr_joint_[i]->m_State.m_rCommand = 0.0;
  }

  // Right
  for(int i(0); i<3; ++i){
    robot->r_joint_[i]->m_State.m_rCommand = torque_command[i];
  }
  // Left
  for(int i(0); i<3; ++i){
    robot->r_joint_[i+lj_start_idx]->m_State.m_rCommand = torque_command[i+3];
  }

  // Ankle Passive
  pDyn_env->PassiveAnkleSpring();

  // Push Body
  pDyn_env->PushRobotBody();

  // Hold Body
  if(pDyn_env->count_*SIM_SERVO_RATE < pDyn_env->release_time_){
    pDyn_env->FixXY();
  }
}

void Mercury_Dyn_environment::PushRobotBody(){
  static int push_idx(0);
  if(push_time_.size() > push_idx){

    if(count_ * SIM_SERVO_RATE > push_time_[push_idx]){
      int body_lk_idx(m_Mercury->link_idx_map_.find("body")->second);
      // Force Setting
      double force_size(push_force_[push_idx]);
      double force_dir(DEG2RAD(push_direction_[push_idx]));

      double force_x = -force_size * cos(force_dir);
      double force_y = -force_size * sin(force_dir);

      dse3 ext_force(0., 0., 0.,force_x, force_y, 0.);
      m_Mercury->link_[body_lk_idx]->AddUserExternalForce(ext_force);

      if(count_*SERVO_RATE > push_time_[push_idx] + 0.1){
        ++push_idx;
      }
    }
  }
}
void Mercury_Dyn_environment::PassiveAnkleSpring(){
  double kp(0.02);
  double kd(0.005);
  double des_pos(-0.5);
  m_Mercury->r_joint_[3]->m_State.m_rCommand = kp * (des_pos - m_Mercury->r_joint_[3]->m_State.m_rValue[0]) - kd* m_Mercury->r_joint_[3]->m_State.m_rValue[1];
  m_Mercury->r_joint_[7]->m_State.m_rCommand = kp * (des_pos - m_Mercury->r_joint_[7]->m_State.m_rValue[0]) - kd* m_Mercury->r_joint_[7]->m_State.m_rValue[1];
}

void Mercury_Dyn_environment::FixXY(){
  double pos,vel;

  double kp(2000.0);
  double kd(300.0);

  int idx(0);
  pos = m_Mercury->vp_joint_[idx]->m_State.m_rValue[0];
  vel = m_Mercury->vp_joint_[idx]->m_State.m_rValue[1];
  m_Mercury->vp_joint_[idx]->m_State.m_rCommand = -kp * pos - kd * vel;

  idx = 1;
  pos = m_Mercury->vp_joint_[idx]->m_State.m_rValue[0];
  vel = m_Mercury->vp_joint_[idx]->m_State.m_rValue[1];

  m_Mercury->vp_joint_[idx]->m_State.m_rCommand = -kp * pos - kd * vel;

}

void Mercury_Dyn_environment::Rendering_Fnc(){}


void Mercury_Dyn_environment::_ParamterSetup(){
  ParamHandler handler(CONFIG_PATH"SIM_sr_sim_setting.yaml");

  handler.getInteger("num_substep_rendering", num_substep_rendering_);
  handler.getValue("releasing_time", release_time_);
  handler.getVector("imu_angular_velocity_bias", imu_ang_vel_bias_);
  handler.getVector("imu_angular_velocity_noise_variance", imu_ang_vel_var_);

  handler.getVector("push_timing", push_time_);
  handler.getVector("push_force", push_force_);
  handler.getVector("push_direction", push_direction_);
}

void Mercury_Dyn_environment::getIMU_Data(std::vector<double> & imu_acc,
                                          std::vector<double> & imu_ang_vel){
  // IMU data
  se3 imu_se3_vel = m_Mercury->link_[m_Mercury->link_idx_map_.find("imu")->second]->GetVel();
  se3 imu_se3_acc = m_Mercury->link_[m_Mercury->link_idx_map_.find("imu")->second]->GetAcc();
  SE3 imu_frame = m_Mercury->link_[m_Mercury->link_idx_map_.find("imu")->second]->GetFrame();
  SO3 imu_ori = m_Mercury->link_[m_Mercury->link_idx_map_.find("imu")->second]->GetOrientation();

  Eigen::Matrix3d Rot;
  Rot<<
    imu_frame(0,0), imu_frame(0,1), imu_frame(0,2),
    imu_frame(1,0), imu_frame(1,1), imu_frame(1,2),
    imu_frame(2,0), imu_frame(2,1), imu_frame(2,2);

  sejong::Vect3 grav; grav.setZero();
  grav[2] = 9.81;
  sejong::Vect3 local_grav = Rot.transpose() * grav;

  for(int i(0); i<3; ++i){
    imu_ang_vel[i] = imu_se3_vel[i] + imu_ang_vel_bias_[i] +
      sejong::generator_white_noise(0., imu_ang_vel_var_[i]);
    imu_acc[i] = imu_se3_acc[i+3] + local_grav[i];
  }

  Eigen::Matrix<double, 3, 1> ang_vel;
  ang_vel<<imu_se3_vel[0], imu_se3_vel[1], imu_se3_vel[2];
  sejong::Vect3 global_ang_vel = Rot * ang_vel;
  Eigen::Quaterniond quat(Rot);

  bool b_printout(false);
  if(b_printout){
    printf("imu info: \n");
    std::cout<<imu_se3_vel<<std::endl;
    std::cout<<imu_se3_acc<<std::endl;
    std::cout<<imu_frame<<std::endl;

    sejong::pretty_print(imu_ang_vel, "imu ang vel");
    sejong::pretty_print(imu_acc, "imu_acc");

    printf("global ang vel\n");
    std::cout<<global_ang_vel<<std::endl;

    printf("quat global:\n");
    std::cout<<quat.w()<<std::endl;
    std::cout<<quat.vec()<<std::endl;
  }
  interface_->global_ori_ = quat;
}
