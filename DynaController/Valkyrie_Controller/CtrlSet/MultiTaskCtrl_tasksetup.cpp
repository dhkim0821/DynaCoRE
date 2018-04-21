#include "MultiTaskCtrl.hpp"
#include <Utils/utilities.hpp>
#include <Valkyrie/Valkyrie_Definition.h>
#include "../Valkyrie_StateProvider.hpp"
#include <WBDC/WBDC.hpp>
#include <Valkyrie_Controller/ContactSet/DoubleContact.hpp>
#include <Valkyrie_Controller/TaskSet/JPosTask.hpp>
#include <Valkyrie_Controller/TaskSet/CentroidTask.hpp>
#include <Valkyrie_Controller/TaskSet/LinkPosTask.hpp>

void MultiTaskCtrl::_jpos_task_setup(){
  int jidx (valkyrie_joint::torsoYaw);
  double amp(0.);
  double omega(2.*M_PI * 1.0);

  dynacore::Vector jpos_des = ini_jpos_;
  dynacore::Vector jvel_des(valkyrie::num_act_joint);
  dynacore::Vector jacc_des(valkyrie::num_act_joint);
  jvel_des.setZero(); jacc_des.setZero();

  jpos_des[jidx] += amp * sin(omega * state_machine_time_);
  jvel_des[jidx] = amp * omega * cos(omega * state_machine_time_);
  jacc_des[jidx] = -amp * omega * omega * sin(omega * state_machine_time_);

  jpos_task_->UpdateTask(&jpos_des, jvel_des, jacc_des);
  task_list_.push_back(jpos_task_);

  if(data_save_ == true){
    dynacore::Vector q_actual;
    q_actual = sp_->q_;
    dynacore::saveVector(jpos_des, "q_des");
    dynacore::saveVector(q_actual, "q_act");
  }
}

void MultiTaskCtrl::_centroid_task_setup(){
    dynacore::Vector cm_pos_des(centroid_task_->getDim());
    dynacore::Vector cm_vel_des(centroid_task_->getDim());
    dynacore::Vector cm_acc_des(centroid_task_->getDim());

    cm_pos_des.setZero(); cm_vel_des.setZero(); cm_acc_des.setZero();
    cm_pos_des.tail(3) = ini_com_;

    // double amp(0.075);
    double amp(0.1);
    double omega(2.*M_PI * 1.0);

    int dir(5);
    // cm_pos_des[dir] += amp * sin(omega * state_machine_time_);
    // cm_vel_des[dir] = amp * omega * cos(omega * state_machine_time_);
    // cm_acc_des[dir] = -amp * omega * omega * sin(omega * state_machine_time_);

    cm_pos_des[dir] += amp * cos(omega * state_machine_time_) - amp;
    cm_vel_des[dir] = -amp * omega * sin(omega * state_machine_time_);
    cm_acc_des[dir] = -amp * omega * omega * cos(omega * state_machine_time_);

    centroid_task_->UpdateTask(&cm_pos_des, cm_vel_des, cm_acc_des);
    task_list_.push_back(centroid_task_);

    if (data_save_ == true){
        dynacore::Vect3 com_pos;
        robot_sys_->getCoMPosition(com_pos);

        dynacore::saveVector(cm_pos_des, "cm_des");
        dynacore::saveVector(com_pos, "com");
    }
}
void MultiTaskCtrl::_pelvis_ori_task_setup(){
    dynacore::Quaternion pelvis_ori_des;
    dynacore::Vector pelvis_angvel_des(pelvis_ori_task_->getDim());
    dynacore::Vector pelvis_angacc_des(pelvis_ori_task_->getDim());

    pelvis_ori_des = ini_pelvis_ori_;
    pelvis_angvel_des.setZero(); 
    pelvis_angacc_des.setZero();

    pelvis_ori_task_->UpdateTask(&pelvis_ori_des, pelvis_angvel_des, pelvis_angacc_des);
    task_list_.push_back(pelvis_ori_task_);
}


void MultiTaskCtrl::_body_ori_task_setup(){
    dynacore::Quaternion body_ori_des;
    dynacore::Vector body_angvel_des(body_ori_task_->getDim());
    dynacore::Vector body_angacc_des(body_ori_task_->getDim());

    body_ori_des = ini_body_ori_;
    body_angvel_des.setZero(); 
    body_angacc_des.setZero();

    //double amp(-0.1);
    //double omega(2.*M_PI * 0.1);

    //int dir(5);
    //body_angpos_des[dir] += amp * sin(omega * state_machine_time_);
    //body_angvel_des[dir] = amp * omega * cos(omega * state_machine_time_);
    //body_angacc_des[dir] = -amp * omega * omega * sin(omega * state_machine_time_);

    body_ori_task_->UpdateTask(&body_ori_des, body_angvel_des, body_angacc_des);
    task_list_.push_back(body_ori_task_);
}

void MultiTaskCtrl::_head_ori_task_setup(){
    dynacore::Quaternion head_ori_des;
    dynacore::Vector head_angvel_des(head_ori_task_->getDim());
    dynacore::Vector head_angacc_des(head_ori_task_->getDim());

    head_ori_des = ini_head_ori_;
    head_angvel_des.setZero(); 
    head_angacc_des.setZero();

    //double amp(-0.1);
    //double omega(2.*M_PI * 0.1);

    //int dir(5);
    //body_angpos_des[dir] += amp * sin(omega * state_machine_time_);
    //body_angvel_des[dir] = amp * omega * cos(omega * state_machine_time_);
    //body_angacc_des[dir] = -amp * omega * omega * sin(omega * state_machine_time_);

    head_ori_task_->UpdateTask(&head_ori_des, head_angvel_des, head_angacc_des);
    task_list_.push_back(head_ori_task_); 
}

void MultiTaskCtrl::_right_palm_task_setup(){
    dynacore::Vector rpalm_pos_des(rpalm_pos_task_->getDim());
    dynacore::Vector rpalm_vel_des(rpalm_pos_task_->getDim());
    dynacore::Vector rpalm_acc_des(rpalm_pos_task_->getDim());

    rpalm_pos_des = ini_rpalm_;
    rpalm_vel_des.setZero(); 
    rpalm_acc_des.setZero();

    //double amp(-0.1);
    //double omega(2.*M_PI * 0.1);

    //int dir(5);
    //rpalm_angpos_des[dir] += amp * sin(omega * state_machine_time_);
    //rpalm_vel_des[dir] = amp * omega * cos(omega * state_machine_time_);
    //rpalm_acc_des[dir] = -amp * omega * omega * sin(omega * state_machine_time_);

    rpalm_pos_task_->UpdateTask(&rpalm_pos_des, rpalm_vel_des, rpalm_acc_des);
    task_list_.push_back(rpalm_pos_task_);

    if (data_save_ == true){
        dynacore::Vect3 rp_pos; 
        robot_sys_->getPos(valkyrie_link::rightPalm, rp_pos);
        dynacore::Vector rp_error(3);
        rp_error = rpalm_pos_des - rp_pos ; 
        dynacore::saveVector(rp_error,"rp error");
    }
}

void MultiTaskCtrl::_left_palm_task_setup(){
    dynacore::Vector lpalm_pos_des(lpalm_pos_task_->getDim());
    dynacore::Vector lpalm_vel_des(lpalm_pos_task_->getDim());
    dynacore::Vector lpalm_acc_des(lpalm_pos_task_->getDim());

    lpalm_pos_des = ini_lpalm_;
    lpalm_vel_des.setZero();
    lpalm_acc_des.setZero();

    lpalm_pos_task_->UpdateTask(&lpalm_pos_des, lpalm_vel_des, lpalm_acc_des);
    task_list_.push_back(lpalm_pos_task_);

    if(data_save_ == true){
        dynacore::Vect3 lp_pos; 
        robot_sys_->getPos(valkyrie_link::leftPalm, lp_pos);
        dynacore::Vector lp_error(3);
        lp_error = lpalm_pos_des - lp_pos ; 
        dynacore::saveVector(lp_error,"lp_error");
    }
}
