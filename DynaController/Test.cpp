#include "Test.hpp"
#include "Controller.hpp"
#include <Utils/DataManager.hpp>

Test::Test(RobotSystem* robot):b_first_visit_(true){
  DataManager::GetDataManager()->RegisterData(&phase_, INT, "phase");
}

Test::~Test(){
}

void Test::getTorqueInput(dynacore::Vector & gamma){
  if(b_first_visit_){
    state_list_[phase_]->FirstVisit();
    b_first_visit_ = false;
  }

  state_list_[phase_]->OneStep(gamma);

  if(state_list_[phase_]->EndOfPhase()){
    state_list_[phase_]->LastVisit();
    phase_ = _NextPhase(phase_);
    b_first_visit_ = true;
  }
}
