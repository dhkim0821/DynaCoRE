#ifndef VALKYRIE_JOINT_POSITION_TEST
#define VALKYRIE_JOINT_POSITION_TEST

#include <Test.hpp>

namespace valkyrie_jpos_test_phase{
  constexpr int JPOS_TEST_SWING = 0;
  constexpr int NUM_JPOS_TEST = 1;
};

class JointCtrlTest: public Test{
public:
  JointCtrlTest(RobotSystem* );
  virtual ~JointCtrlTest();

  virtual void TestInitialization();

protected:
  virtual int _NextPhase(const int & phase);
  Controller* jpos_ctrl_;
};

#endif
