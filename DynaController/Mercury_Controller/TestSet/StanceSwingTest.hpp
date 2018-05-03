#ifndef STANCE_SWING_TEST
#define STANCE_SWING_TEST

#include <Test.hpp>
#include <ParamHandler/ParamHandler.hpp>
class Mercury_StateProvider;

enum StanceSwingPhase{
  stance_swing_initiation = 0,
  stance_swing_lift_up = 1,
  stance_swing_double_contact_1 = 2,
  stance_swing_swing_start_trans = 3,
  stance_swing_swing = 4,
  NUM_STANCE_SWING_PHASE
};

class StanceSwingTest: public Test{
public:
  StanceSwingTest(RobotSystem*);
  virtual ~StanceSwingTest();
  virtual void TestInitialization();

protected:
  int swing_foot_;

  Mercury_StateProvider* sp_;
  virtual int _NextPhase(const int & phase);
  void _SettingParameter(ParamHandler & );

  Controller* jpos_ctrl_;
  Controller* body_up_ctrl_;
  Controller* body_fix_ctrl_;
  Controller* swing_start_trans_ctrl_;
  Controller* swing_ctrl_;
  
  const RobotSystem* robot_sys_;
};
#endif
