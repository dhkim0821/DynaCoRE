#include "JointCtrlTest.hpp"
#include <Mercury_Controller/CtrlSet/JPosCtrl.hpp>
#include <Mercury_Controller/CtrlSet/JPosTargetCtrl.hpp>
#include <ParamHandler/ParamHandler.hpp>

JointCtrlTest::JointCtrlTest(RobotSystem* robot):Test(robot){
  phase_ = 0;
  state_list_.clear();

  jpos_ctrl_ini_ = new JPosTargetCtrl(robot);
  jpos_ctrl_ = new JPosCtrl(robot);

  state_list_.push_back(jpos_ctrl_ini_);
  state_list_.push_back(jpos_ctrl_);

  _ParameterSetting();
  printf("[Joint Ctrl Test] Constructed\n");
}
JointCtrlTest::~JointCtrlTest(){
  delete jpos_ctrl_ini_;
  delete jpos_ctrl_;
}
void JointCtrlTest::TestInitialization(){
  jpos_ctrl_ini_->CtrlInitialization("CTRL_jpos_initialization");
  jpos_ctrl_->CtrlInitialization("CTRL_jpos_swing");
}

int JointCtrlTest::_NextPhase(const int & phase){
  int nx_phase = phase + 1;
  if(phase == NUM_JPOS_TEST){
    nx_phase = JPOS_TEST_SWING;
  }
  return nx_phase;
}

void JointCtrlTest::_ParameterSetting(){
  ParamHandler handle(MercuryConfigPath"TEST_jpos_ctrl.yaml");
  std::vector<double> tmp_vec;
  double tmp_value;

  // JPos initialization
  handle.getVector("initial_jpos", tmp_vec);
  ((JPosTargetCtrl*)jpos_ctrl_ini_)->setTargetPosition(tmp_vec);
  ((JPosCtrl*)jpos_ctrl_)->setPosture(tmp_vec); // swing test middle posture set

  handle.getValue("initialization_time", tmp_value);
  ((JPosTargetCtrl*)jpos_ctrl_ini_)->setMovingTime(tmp_value);

  // JPos ctrl swing
  handle.getValue("swing_test_time", tmp_value);
  ((JPosCtrl*)jpos_ctrl_)->setMovingTime(tmp_value);

  handle.getVector("amplitude", tmp_vec);
  ((JPosCtrl*)jpos_ctrl_)->setAmplitude(tmp_vec);
  handle.getVector("frequency", tmp_vec);
  ((JPosCtrl*)jpos_ctrl_)->setFrequency(tmp_vec);
  handle.getVector("phase", tmp_vec);
  ((JPosCtrl*)jpos_ctrl_)->setPhase(tmp_vec);
}
