#ifndef FOOT_CTRL
#define FOOT_CTRL

#include <Controller.hpp>

class Mercury_StateProvider;
class RobotSystem;
class WBDC_Relax;
class WBDC_Relax_ExtraData;
class WBDC_Relax_Task;
class WBDC_ContactSpec;

class FootCtrl: public Controller{
public:
  FootCtrl(RobotSystem* robot, int swing_foot);
  virtual ~FootCtrl();

  virtual void OneStep(dynacore::Vector & gamma);
  virtual void FirstVisit();
  virtual void LastVisit();
  virtual bool EndOfPhase();

  virtual void CtrlInitialization(const std::string & setting_file_name);

  void setMovingTime(double time) { end_time_ = time; }
  void setAmplitude(const std::vector<double> & amp){ amp_ = amp; }
  void setFrequency(const std::vector<double> & freq){ freq_ = freq; }
  void setPhase(const std::vector<double> & phase){ phase_ = phase; }

protected:
  int swing_foot_;
  double end_time_;

  WBDC_Relax* wbdc_;
  WBDC_Relax_ExtraData* wbdc_data_;
  WBDC_Relax_Task* foot_task_;
  WBDC_Relax_Task* jpos_task_;
  WBDC_ContactSpec* fixed_body_contact_;

  dynacore::Vect3 foot_pos_ini_;
  dynacore::Vector jpos_ini_;
  
  dynacore::Vector foot_pos_des_;
  dynacore::Vector foot_vel_des_;
  dynacore::Vector foot_acc_des_;

  std::vector<double> amp_;
  std::vector<double> freq_;
  std::vector<double> phase_;

  void _foot_pos_task_setup();
  void _jpos_task_setup();
  void _fixed_body_contact_setup();
  void _foot_pos_ctrl(dynacore::Vector & gamma);

  double ctrl_start_time_;
  Mercury_StateProvider* sp_;
};

#endif
