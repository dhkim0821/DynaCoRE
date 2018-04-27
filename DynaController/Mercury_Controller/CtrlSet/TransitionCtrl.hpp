#ifndef TRANSITION_CONTROLLER
#define TRANSITION_CONTROLLER

#include <Controller.hpp>

class RobotSystem;
class Mercury_StateProvider;
class WBDC_Relax;
class WBDC_Relax_ExtraData;
class WBDC_Relax_Task;
class WBDC_ContactSpec;

class WBDC_Rotor;
class WBDC_Rotor_ExtraData;

class TransitionCtrl: public Controller{
public:
  TransitionCtrl(RobotSystem* robot, int moving_foot, bool b_increase);
  virtual ~TransitionCtrl();

  virtual void OneStep(dynacore::Vector & gamma);
  virtual void FirstVisit();
  virtual void LastVisit();
  virtual bool EndOfPhase();

  virtual void CtrlInitialization(const std::string & setting_file_name);

  void setTransitionTime(double time){ end_time_ = time; }
  void setStanceHeight(double height) {
    des_com_height_ = height;
    b_set_height_target_ = true;
  }

protected:
  bool b_set_height_target_;
  double des_com_height_;

  double end_time_;
  int moving_foot_;
  bool b_increase_; // Increasing or decreasing reaction force
  double max_rf_z_;
  double min_rf_z_;
  
  WBDC_Relax* wbdc_;
  WBDC_Relax_ExtraData* wbdc_data_;
  WBDC_Relax_Task* body_task_;
  WBDC_ContactSpec* double_contact_;
  WBDC_Rotor* wbdc_rotor_;
  WBDC_Rotor_ExtraData* wbdc_rotor_data_;

  dynacore::Vector body_pos_ini_;
  dynacore::Vect3 ini_com_pos_;


  void _body_task_setup();
  void _double_contact_setup();
  void _body_ctrl(dynacore::Vector & gamma);
  void _body_ctrl_wbdc_rotor(dynacore::Vector & gamma);

  Mercury_StateProvider* sp_;
  double ctrl_start_time_;
};
#endif
