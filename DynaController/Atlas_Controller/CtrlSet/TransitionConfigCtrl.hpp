#ifndef TRANSITION_CONFIGURATION_CONTROLLER_ATLAS
#define TRANSITION_CONFIGURATION_CONTROLLER_ATLAS

#include <Controller.hpp>
#include <Atlas_Controller/Atlas_InvKinematics.hpp>

class RobotSystem;
class Atlas_StateProvider;
class WBDC_ContactSpec;
class WBDC;
class WBDC_ExtraData;

class TransitionConfigCtrl: public Controller{
public:
  TransitionConfigCtrl(RobotSystem* robot, int moving_foot, bool b_increase);
  virtual ~TransitionConfigCtrl();

  virtual void OneStep(void* _cmd);
  virtual void FirstVisit();
  virtual void LastVisit();
  virtual bool EndOfPhase();

  virtual void CtrlInitialization(const std::string & setting_file_name);

  void setTransitionTime(double time){ end_time_ = time; }
  void setStanceHeight(double height) {
    des_body_height_ = height;
    b_set_height_target_ = true;
  }

protected:
  bool b_set_height_target_;
  double des_body_height_;

  double end_time_;
  int moving_foot_;
  bool b_increase_; // Increasing or decreasing reaction force
  double max_rf_z_;
  double min_rf_z_;
  
  Task* config_task_;
  WBDC_ContactSpec* double_contact_;
  WBDC* wbdc_;
  WBDC_ExtraData* wbdc_data_;


  dynacore::Vector des_jpos_;
  dynacore::Vector des_jvel_;
  
  dynacore::Vector body_pos_ini_;
  dynacore::Vect3 ini_body_pos_;


  void _body_task_setup();
  void _double_contact_setup();
  void _body_ctrl_wbdc(dynacore::Vector & gamma);

  Atlas_StateProvider* sp_;
  Atlas_InvKinematics* inv_kin_;
  double ctrl_start_time_;
};
#endif
