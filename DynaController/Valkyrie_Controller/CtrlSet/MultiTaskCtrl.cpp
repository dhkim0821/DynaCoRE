#include "MultiTaskCtrl.hpp"
#include <Utils/utilities.hpp>
#include <Valkyrie/Valkyrie_Definition.h>
#include "../Valkyrie_StateProvider.hpp"
#include <WBDC/WBDC.hpp>
#include <Valkyrie_Controller/ContactSet/DoubleContact.hpp>
#include <Valkyrie_Controller/TaskSet/JPosTask.hpp>
#include <Valkyrie_Controller/TaskSet/CentroidTask.hpp>
#include <Valkyrie_Controller/TaskSet/LinkOriTask.hpp>
#include <Valkyrie_Controller/TaskSet/LinkPosTask.hpp>
#include <chrono>

MultiTaskCtrl::MultiTaskCtrl(RobotSystem* robot):Controller(robot),
    ini_jpos_(valkyrie::num_act_joint)
{
    std::vector<bool> act_list;
    act_list.resize(valkyrie::num_qdot, true);
    for(int i(0); i<valkyrie::num_virtual; ++i) act_list[i] = false;

    wbdc_ = new WBDC(act_list);
    wbdc_data_ = new WBDC_ExtraData();

    jpos_task_ = new JPosTask();
    centroid_task_ = new CentroidTask(robot);
    body_ori_task_ = new LinkOriTask(robot, valkyrie_link::torso);
    head_ori_task_ = new LinkOriTask(robot, valkyrie_link::head);
    pelvis_ori_task_ = new LinkOriTask(robot, valkyrie_link::pelvis);
    rpalm_pos_task_ = new LinkPosTask(robot, valkyrie_link::rightPalm);
    lpalm_pos_task_ = new LinkPosTask(robot, valkyrie_link::leftPalm);
    double_contact_ = new DoubleContact(robot);

    sp_ = Valkyrie_StateProvider::getStateProvider();
    time_measure_ = false;
    data_save_ = true;
}

MultiTaskCtrl::~MultiTaskCtrl(){
    delete jpos_task_;
    delete centroid_task_;
    delete double_contact_;
    delete body_ori_task_;
    delete pelvis_ori_task_;
    delete wbdc_;
}

void MultiTaskCtrl::_double_contact_setup(){
    double_contact_->UpdateContactSpec();
    contact_list_.push_back(double_contact_);
}

void MultiTaskCtrl::_wblc_ctrl(dynacore::Vector & gamma){
    wbdc_->UpdateSetting(A_, Ainv_, coriolis_, grav_);
    
    int dim_first_task = task_list_[0]->getDim();

    wbdc_data_->cost_weight = dynacore::Vector(
            dim_first_task + double_contact_->getDim());

    for(int i(0); i<dim_first_task; ++i) wbdc_data_->cost_weight[i] = 10000.0;
    for(int i(0); i<double_contact_->getDim(); ++i){
        wbdc_data_->cost_weight[dim_first_task + i] = 1.0;
    }
    wbdc_data_->cost_weight[dim_first_task + 5] = 0.001; // vertical reaction
    wbdc_data_->cost_weight[dim_first_task + 11] = 0.001; // vertical reaction

    //std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
    wbdc_->MakeTorque(task_list_, contact_list_, gamma, wbdc_data_);
    
    //std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
    //std::chrono::duration<double> time_span1 = std::chrono::duration_cast< std::chrono::duration<double> >(t2 - t1);
    //std::cout << "[multitaskctrl] All process took me " << time_span1.count()*1000.0 << "ms."<<std::endl;;
}

void MultiTaskCtrl::OneStep(dynacore::Vector & gamma){
    if(time_measure_ == true)
        t_s_ = std::chrono::high_resolution_clock::now();
 
     _PreProcessing_Command();
    _double_contact_setup();  

    int i(3);

    switch(i){
    case 1:
        _centroid_task_setup();
        _jpos_task_setup();
        break;
    case 2:
        _centroid_task_setup();
        _right_palm_task_setup();
        _jpos_task_setup();
        break;
    case 3:
        _centroid_task_setup();
        _right_palm_task_setup();
        _left_palm_task_setup();
        _jpos_task_setup();
        break;
    case 4:
        _centroid_task_setup();
        _body_ori_task_setup();
        _right_palm_task_setup();
        _left_palm_task_setup();
        _head_ori_task_setup();
        _jpos_task_setup();
        break;
    case 5:
        _centroid_task_setup();
        _pelvis_ori_task_setup();
        _body_ori_task_setup();
        _right_palm_task_setup();
        _left_palm_task_setup();
        _head_ori_task_setup();
        _jpos_task_setup();
        break;
    }
   
    _wblc_ctrl(gamma);

    _PostProcessing_Command();
    state_machine_time_  += valkyrie::servo_rate;

    if(time_measure_==true){
        t_f_ = std::chrono::high_resolution_clock::now();
        time_span_ = std::chrono::duration_cast< std::chrono::duration<double> >(t_f_ - t_s_);
        dynacore::saveValue(time_span_.count(),"time");
    }
}

void MultiTaskCtrl::FirstVisit(){
    for(int i(0); i<valkyrie::num_act_joint; ++i) 
        ini_jpos_[i] = sp_->q_[i+valkyrie::num_virtual];
    
    robot_sys_->getCoMPosition(ini_com_);
    robot_sys_->getOri(valkyrie_link::torso, ini_body_ori_);
    robot_sys_->getOri(valkyrie_link::pelvis, ini_pelvis_ori_);
    robot_sys_->getPos(valkyrie_link::rightPalm, ini_rpalm_);
    robot_sys_->getPos(valkyrie_link::leftPalm, ini_lpalm_);
    robot_sys_->getOri(valkyrie_link::head, ini_head_ori_);
}

void MultiTaskCtrl::LastVisit(){
}

bool MultiTaskCtrl::EndOfPhase(){
    return false;
}

void MultiTaskCtrl::CtrlInitialization(const std::string & setting_file_name){
}
