#ifndef VALKYRIE_MULTIPLE_TASKS_TEST
#define VALKYRIE_MULTIPLE_TASKS_TEST

#include <Test.hpp>

namespace valkyrie_multi_task_phase{
  constexpr int MULTI_TASK_TEST = 0;
  constexpr int NUM_MULTI_TASK_PHASE = 1;
};

class MultiTaskTest: public Test{
public:
  MultiTaskTest(RobotSystem* );
  virtual ~MultiTaskTest();

  virtual void TestInitialization();

protected:
  virtual int _NextPhase(const int & phase);
  Controller* multi_task_ctrl_;
};

#endif
