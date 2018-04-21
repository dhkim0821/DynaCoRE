#include "JPosCtrl.hpp"
#include <Utils/utilities.hpp>
#include <Valkyrie/Valkyrie_Definition.h>
#include "../Valkyrie_StateProvider.hpp"
#include <WBDC/WBDC.hpp>
#include <Valkyrie_Controller/ContactSet/DoubleContact.hpp>
#include <Valkyrie_Controller/TaskSet/JPosTask.hpp>

JPosCtrl::JPosCtrl(RobotSystem* robot):Controller(robot),
    ini_jpos_(valkyrie::num_act_joint)
{
    std::vector<bool> act_list;
    act_list.resize(valkyrie::num_qdot, true);
    for(int i(0); i<valkyrie::num_virtual; ++i) act_list[i] = false;

    wbdc_ = new WBDC(act_list);
    wbdc_data_ = new WBDC_ExtraData();

    jpos_task_ = new JPosTask();
    double_contact_ = new DoubleContact(robot);
    sp_ = Valkyrie_StateProvider::getStateProvider();

    time_measure_ = true;
}

JPosCtrl::~JPosCtrl(){
    delete jpos_task_;
    delete double_contact_;
    delete wbdc_;
}

void JPosCtrl::_jpos_task_setup(){
  dynacore::Vector jpos_des = ini_jpos_;
  dynacore::Vector jvel_des(valkyrie::num_act_joint);
  dynacore::Vector jacc_des(valkyrie::num_act_joint);
  jvel_des.setZero(); jacc_des.setZero();

  double amp(0.6);
  double omega(2.*M_PI * 1.0);
 
  int jidx (valkyrie_joint::rightShoulderRoll);
  jpos_des[jidx] += amp * sin(omega * state_machine_time_);
  jvel_des[jidx] = amp * omega * cos(omega * state_machine_time_);
  jacc_des[jidx] = -amp * omega * omega * sin(omega * state_machine_time_);

  jidx  = valkyrie_joint::leftShoulderRoll;
  amp = -0.6;
  jpos_des[jidx] += amp * sin(omega * state_machine_time_);
  jvel_des[jidx] = amp * omega * cos(omega * state_machine_time_);
  jacc_des[jidx] = -amp * omega * omega * sin(omega * state_machine_time_);


  jpos_task_->UpdateTask(&jpos_des, jvel_des, jacc_des);
  task_list_.push_back(jpos_task_);
}

void JPosCtrl::_double_contact_setup(){
    double_contact_->UpdateContactSpec();
    contact_list_.push_back(double_contact_);
}

void JPosCtrl::_jpos_ctrl(dynacore::Vector & gamma){
    //printf("ctrl 1\n");
    wbdc_->UpdateSetting(A_, Ainv_, coriolis_, grav_);

    //printf("ctrl 2\n");
    wbdc_data_->cost_weight = dynacore::Vector(
            jpos_task_->getDim() + double_contact_->getDim());

    //printf("ctrl 3\n");
    for(int i(0); i<jpos_task_->getDim(); ++i) wbdc_data_->cost_weight[i] = 10000.0;
    for(int i(0); i<double_contact_->getDim(); ++i){
        wbdc_data_->cost_weight[jpos_task_->getDim() + i] = 1.0;
    }
    //printf("ctrl 4\n");
    wbdc_data_->cost_weight[jpos_task_->getDim() + 5] = 0.001; // vertical reaction
    wbdc_data_->cost_weight[jpos_task_->getDim() + 11] = 0.001; // vertical reaction
    
    //printf("ctrl 5\n");
    wbdc_->MakeTorque(task_list_, contact_list_, gamma, wbdc_data_);
    //printf("ctrl 6\n");
    //dynacore::pretty_print(grav_, std::cout, "gravity");
}

void JPosCtrl::_SvNcmCheckTest(){
     dynacore::Matrix eye(valkyrie::num_qdot, valkyrie::num_qdot);
    eye.setIdentity();

   dynacore::Matrix Jcm, Icm;
    robot_sys_->getCentroidJacobian(Jcm);
    robot_sys_->getCentroidInertia(Icm);
    dynacore::Matrix JcmBar, Ncm, Acm, AcmBar, NAcm;
    _DynConsistent_Inverse(Jcm, JcmBar);
    Ncm = eye - JcmBar * Jcm;

    Acm = Icm * Jcm;
    _DynConsistent_Inverse(Acm, AcmBar);
    NAcm = eye - AcmBar * Acm;

    // S virtual
    dynacore::Matrix Sv(valkyrie::num_virtual, valkyrie::num_qdot); 
    Sv.setZero();
    (Sv.block(0, 0, valkyrie::num_virtual, valkyrie::num_virtual)).setIdentity();
    dynacore::Matrix SvA = Sv * A_;

    dynacore::Matrix SvNcm = Sv * Ncm;
    dynacore::Matrix JcmNcm = Jcm * Ncm;

    dynacore::Matrix SvANcm = SvA  * Ncm;
    dynacore::Matrix SvANAcm = SvA * NAcm;

    dynacore::Matrix SvBar, Nsv, SvABar, NSvA;
    _DynConsistent_Inverse(Sv, SvBar);
    _DynConsistent_Inverse(SvA, SvABar);
    Nsv = eye - SvBar * Sv;
    NSvA = eye - SvABar * SvA;

    dynacore::Matrix JcmNsv = Jcm * Nsv;
    dynacore::Matrix JcmNSvA = Jcm* NSvA;

    dynacore::pretty_print(JcmNsv, std::cout, "JcmNsv");
    dynacore::pretty_print(SvNcm, std::cout, "SvNcm");
    dynacore::pretty_print(JcmNcm, std::cout, "JcmNcm");
    dynacore::pretty_print(Jcm, std::cout, "Jcm");
    dynacore::pretty_print(Sv, std::cout, "Sv");
    dynacore::pretty_print(Ncm, std::cout, "Ncm");
    dynacore::pretty_print(SvANcm, std::cout, "SvANcm");
    dynacore::pretty_print(SvANAcm, std::cout, "SvANAcm");
    dynacore::pretty_print(JcmNSvA, std::cout, "JcmNsvA");
    //dynacore::pretty_print(Icm, std::cout, "Icm");
}
void JPosCtrl::OneStep(dynacore::Vector & gamma){

    if(time_measure_ == true)
        t_s_ = std::chrono::high_resolution_clock::now();

    _PreProcessing_Command();
   bool verbose(false); 
    gamma.setZero();

    // TEST
    //_SvNcmCheckTest();
    if(verbose) printf("1\n");
    _double_contact_setup();
    if(verbose)printf("2\n");
    _jpos_task_setup();
    if(verbose)printf("3\n");
    _jpos_ctrl(gamma);
    if(verbose)printf("4\n");
    _PostProcessing_Command();
    if(verbose)printf("5\n");
    state_machine_time_  += valkyrie::servo_rate;

    if(time_measure_==true){
        t_f_ = std::chrono::high_resolution_clock::now();
        time_span_ = std::chrono::duration_cast< std::chrono::duration<double> >(t_f_ - t_s_);
        dynacore::saveValue(time_span_.count(),"time");
    }
}

void JPosCtrl::FirstVisit(){
    for(int i(0); i<valkyrie::num_act_joint; ++i) ini_jpos_[i] = sp_->q_[i+6];
}

void JPosCtrl::LastVisit(){
}

bool JPosCtrl::EndOfPhase(){
    return false;
}

void JPosCtrl::CtrlInitialization(const std::string & setting_file_name){
}
