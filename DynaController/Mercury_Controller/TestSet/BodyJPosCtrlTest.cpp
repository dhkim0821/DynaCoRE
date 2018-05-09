#include "BodyJPosCtrlTest.hpp"

#include <Mercury_Controller/CtrlSet/JPosTargetCtrl.hpp>
#include <Mercury_Controller/CtrlSet/ContactTransBodyCtrl.hpp>
#include <Mercury_Controller/CtrlSet/CoMzRxRyRzCtrl.hpp>
#include <Mercury_Controller/CtrlSet/BodyJPosCtrl.hpp>

#include <ParamHandler/ParamHandler.hpp>
#include <Mercury/Mercury_Model.hpp>
#include <Mercury/Mercury_Definition.h>

BodyJPosCtrlTest::BodyJPosCtrlTest(RobotSystem* robot):Test(robot){
  phase_ = BCJPosPhase::BCJPOS_initial_jpos;
  state_list_.clear();

  jpos_ctrl_ = new JPosTargetCtrl(robot);
  body_up_ctrl_ = new ContactTransBodyCtrl(robot);
  body_fix_ctrl_ = new CoMzRxRyRzCtrl(robot);
  body_shake_ctrl_ = new CoMzRxRyRzCtrl(robot);
  body_jpos_ctrl_ = new BodyJPosCtrl(robot);

  state_list_.push_back(jpos_ctrl_);
  state_list_.push_back(body_up_ctrl_);
  state_list_.push_back(body_fix_ctrl_);
  //state_list_.push_back(body_shake_ctrl_);
  state_list_.push_back(body_jpos_ctrl_);


  _SettingParameter();

  printf("[Body Joint Position Control Test] Constructed\n");
}

BodyJPosCtrlTest::~BodyJPosCtrlTest(){
  for(int i(0); i<state_list_.size(); ++i){
    delete state_list_[i];
  }
}

void BodyJPosCtrlTest::TestInitialization(){
  // Yaml file name
  jpos_ctrl_->CtrlInitialization("CTRL_jpos_initialization");
  body_up_ctrl_->CtrlInitialization("CTRL_move_to_target_height");
  body_fix_ctrl_->CtrlInitialization("CTRL_fix_des_pos");
  body_shake_ctrl_->CtrlInitialization("CTRL_body_shake_ctrl");
  body_jpos_ctrl_->CtrlInitialization("CTRL_body_jpos");
}

int BodyJPosCtrlTest::_NextPhase(const int & phase){
  int next_phase = phase + 1;
  printf("next phase: %i\n", next_phase);
  if (next_phase == NUM_BCJPOS_PHASE) {
    return BCJPosPhase::BCJPOS_stay_up;
  }
  else return next_phase;
}


void BodyJPosCtrlTest::_SettingParameter(){
  ParamHandler handler(MercuryConfigPath"TEST_body_jpos_ctrl.yaml");

  double tmp;
  std::vector<double> tmp_vec;

  handler.getVector("initial_jpos", tmp_vec);
  ((JPosTargetCtrl*)jpos_ctrl_)->setTargetPosition(tmp_vec);
  ((BodyJPosCtrl*)body_jpos_ctrl_)->setPosture(tmp_vec); // desired joint position posture

  // CoM Height
  handler.getValue("com_height", tmp);
  ((ContactTransBodyCtrl*)body_up_ctrl_)->setStanceHeight(tmp);
  ((CoMzRxRyRzCtrl*)body_fix_ctrl_)->setStanceHeight(tmp);
  ((CoMzRxRyRzCtrl*)body_shake_ctrl_)->setStanceHeight(tmp);

  //// Timing Setup
  handler.getValue("jpos_initialization_time", tmp);
  ((JPosTargetCtrl*)jpos_ctrl_)->setMovingTime(tmp);
  handler.getValue("com_lifting_time", tmp);
  ((ContactTransBodyCtrl*)body_up_ctrl_)->setStanceTime(tmp);

  // Stance Time
  handler.getValue("body_stay_time", tmp);
  ((CoMzRxRyRzCtrl*)body_fix_ctrl_)->setStanceTime(tmp);
  handler.getValue("body_ctrl_time", tmp);
  ((CoMzRxRyRzCtrl*)body_shake_ctrl_)->setStanceTime(tmp);

  // Motion Setup
  handler.getVector("amp", tmp_vec);
  ((CoMzRxRyRzCtrl*)body_shake_ctrl_)->setAmp(tmp_vec);
  handler.getVector("frequency", tmp_vec);
  ((CoMzRxRyRzCtrl*)body_shake_ctrl_)->setFrequency(tmp_vec);
  handler.getVector("phase", tmp_vec);
  ((CoMzRxRyRzCtrl*)body_shake_ctrl_)->setPhase(tmp_vec);
}
