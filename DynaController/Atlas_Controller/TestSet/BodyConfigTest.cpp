#include "BodyConfigTest.hpp"

#include <Atlas_Controller/CtrlSet/JPosTargetCtrl.hpp>
#include <Atlas_Controller/CtrlSet/ContactTransConfigCtrl.hpp>
#include <Atlas_Controller/CtrlSet/ConfigBodyCtrl.hpp>
#include <Atlas_Controller/Atlas_DynaControl_Definition.h>

#include <ParamHandler/ParamHandler.hpp>
#include <Atlas/Atlas_Model.hpp>

BodyConfigTest::BodyConfigTest(RobotSystem* robot):Test(robot){
  //phase_ = BodyConfigPhase::BCJPOS_initial_jpos;
  phase_ = BodyConfigPhase::BCJPOS_body_ctrl;
  state_list_.clear();

  jpos_ctrl_ = new JPosTargetCtrl(robot);
  body_up_ctrl_ = new ContactTransConfigCtrl(robot);
  body_jpos_ctrl_ = new ConfigBodyCtrl(robot);

  state_list_.push_back(jpos_ctrl_);
  state_list_.push_back(body_up_ctrl_);
  state_list_.push_back(body_jpos_ctrl_);

  _SettingParameter();

  printf("[Body Joint Position Control Test] Constructed\n");
}

BodyConfigTest::~BodyConfigTest(){
  for(int i(0); i<state_list_.size(); ++i){
    delete state_list_[i];
  }
}

void BodyConfigTest::TestInitialization(){
  // Yaml file name
  jpos_ctrl_->CtrlInitialization("CTRL_jpos_initialization");
  body_up_ctrl_->CtrlInitialization("CTRL_move_to_target_height");
  body_jpos_ctrl_->CtrlInitialization("CTRL_fix_config");
}

int BodyConfigTest::_NextPhase(const int & phase){
  int next_phase = phase + 1;
  printf("next phase: %i\n", next_phase);
  if (next_phase == NUM_BCJPOS_PHASE) {
    return BodyConfigPhase::BCJPOS_body_ctrl;
  }
  else return next_phase;
}


void BodyConfigTest::_SettingParameter(){
  ParamHandler handler(AtlasConfigPath"TEST_body_ctrl.yaml");

  double tmp;
  std::vector<double> tmp_vec;
  handler.getVector("initial_jpos", tmp_vec);
  ((JPosTargetCtrl*)jpos_ctrl_)->setTargetPosition(tmp_vec);

  // Body Height
  handler.getValue("body_height", tmp);
  ((ContactTransConfigCtrl*)body_up_ctrl_)->setStanceHeight(tmp);
  ((ConfigBodyCtrl*)body_jpos_ctrl_)->setStanceHeight(tmp);

  //// Timing Setup
  handler.getValue("jpos_initialization_time", tmp);
  ((JPosTargetCtrl*)jpos_ctrl_)->setMovingTime(tmp);
  handler.getValue("body_lifting_time", tmp);
  ((ContactTransConfigCtrl*)body_up_ctrl_)->setStanceTime(tmp);

  // Stance Time
  handler.getValue("body_ctrl_time", tmp);
  ((ConfigBodyCtrl*)body_jpos_ctrl_)->setStanceTime(tmp);
}
