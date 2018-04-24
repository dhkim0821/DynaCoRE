#ifndef JOINT_POSITION_CTRL
#define JOINT_POSITION_CTRL

#include <Controller.hpp>

class Mercury_StateProvider;
class RobotSystem;
class WBDC_Relax;
class WBDC_Relax_ExtraData;
class WBDC_Relax_Task;
class WBDC_ContactSpec;

class JPosCtrl: public Controller{
public:
  JPosCtrl(RobotSystem* );
  virtual ~JPosCtrl();

  virtual void OneStep(dynacore::Vector & gamma);
  virtual void FirstVisit();
  virtual void LastVisit();
  virtual bool EndOfPhase();

  virtual void CtrlInitialization(const std::string & setting_file_name);

  void setMovingTime(double time) { end_time_ = time; }
 void setPosture(const std::vector<double> & set_jpos){
    set_jpos_ = set_jpos;
    b_jpos_set_ = true;
  }

 void selectTrajectoryType(int type){ trj_type_ = type; }
 // For sinusoidal trajectory test
 void setAmplitude(const std::vector<double> & amp){ amp_ = amp; }
  void setFrequency(const std::vector<double> & freq){ freq_ = freq; }
  void setPhase(const std::vector<double> & phase){ phase_ = phase; }

  // For ramp trajectory test
   void setJPosDelta(const std::vector<double> & jpos_delta){ jpos_delta_ = jpos_delta; }
  void setStartTime(const std::vector<double> & t){ start_time_ = t; }
  void setDeltaTime(const std::vector<double> & dt){ delta_time_ = dt; }
 
protected:
  int trj_type_;
  double end_time_;

  WBDC_Relax* wbdc_;
  WBDC_Relax_ExtraData* wbdc_data_;
  WBDC_Relax_Task* jpos_task_;
  WBDC_ContactSpec* fixed_body_contact_;

  dynacore::Vector jpos_ini_;
  dynacore::Vector jpos_target_;

  bool b_jpos_set_;
  std::vector<double> set_jpos_;

  // For sinusoidal trajectory test
  std::vector<double> amp_;
  std::vector<double> freq_;
  std::vector<double> phase_;

  // For ramp trajectory test
  std::vector<double> jpos_delta_;
  std::vector<double> start_time_;
  std::vector<double> delta_time_;


  void _jpos_task_setup();
  void _fixed_body_contact_setup();
  void _jpos_ctrl(dynacore::Vector & gamma);

  double ctrl_start_time_;
  Mercury_StateProvider* sp_;
};

#endif
