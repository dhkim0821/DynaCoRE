#include "JointCtrlTest.hpp"
#include <Valkyrie_Controller/CtrlSet/JPosCtrl.hpp>

JointCtrlTest::JointCtrlTest(RobotSystem* robot):Test(robot){
  phase_ = 0;
  state_list_.clear();

  jpos_ctrl_ = new JPosCtrl(robot);
  state_list_.push_back(jpos_ctrl_);

  printf("[Joint Ctrl Test] Constructed\n");
}
JointCtrlTest::~JointCtrlTest(){
  delete jpos_ctrl_;
}
void JointCtrlTest::TestInitialization(){
  jpos_ctrl_->CtrlInitialization("CTRL_jpos_swing");
}

int JointCtrlTest::_NextPhase(const int & phase){
  int nx_phase = phase + 1;
  if(phase == valkyrie_jpos_test_phase::NUM_JPOS_TEST){
    nx_phase = valkyrie_jpos_test_phase::JPOS_TEST_SWING;
  }
  return nx_phase;
}

