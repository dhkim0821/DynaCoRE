#include "MultiTaskTest.hpp"
#include <Valkyrie_Controller/CtrlSet/MultiTaskCtrl.hpp>

MultiTaskTest::MultiTaskTest(RobotSystem* robot):Test(robot){
  phase_ = 0;
  state_list_.clear();

  multi_task_ctrl_ = new MultiTaskCtrl(robot);
  state_list_.push_back(multi_task_ctrl_);

  printf("[Joint Ctrl Test] Constructed\n");
}
MultiTaskTest::~MultiTaskTest(){
  delete multi_task_ctrl_;
}
void MultiTaskTest::TestInitialization(){
  multi_task_ctrl_->CtrlInitialization("CTRL_multi_task");
}

int MultiTaskTest::_NextPhase(const int & phase){
  int nx_phase = phase + 1;
  if(phase == valkyrie_multi_task_phase::NUM_MULTI_TASK_PHASE){
    nx_phase = valkyrie_multi_task_phase::MULTI_TASK_TEST;
  }
  return nx_phase;
}

