#include "StanceSwingTest.hpp"
#include <Mercury_Controller/Mercury_StateProvider.hpp>

#include <Mercury_Controller/CtrlSet/JPosTargetCtrl.hpp>
#include <Mercury_Controller/CtrlSet/ContactTransBodyCtrl.hpp>
#include <Mercury_Controller/CtrlSet/CoMzRxRyRzCtrl.hpp>
#include <Mercury_Controller/CtrlSet/BodyFootCtrl.hpp>
#include <Mercury_Controller/CtrlSet/TransitionCtrl.hpp>
#include <Utils/DataManager.hpp>

#include <Mercury/Mercury_Model.hpp>
#include <Mercury/Mercury_Definition.h>

StanceSwingTest::StanceSwingTest(RobotSystem* robot):Test(robot){
  sp_ = Mercury_StateProvider::getStateProvider();
  sp_->global_pos_local_[1] = 0.15;
  robot_sys_ = robot;
  phase_ = StanceSwingPhase::stance_swing_initiation;
  // Setting Parameters
  ParamHandler handler(MercuryConfigPath"TEST_stance_swing.yaml");
  std::string tmp_str;
  handler.getString("swing_foot", tmp_str);
  if(tmp_str == "right"){
      swing_foot_ = mercury_link::rightFoot;
  }else if(tmp_str == "left"){
      swing_foot_ = mercury_link::leftFoot;
  }else {
      printf("[Stance Swing Test] Incorrect Foot Setting\n");
      exit(0);
  }

  jpos_ctrl_ = new JPosTargetCtrl(robot);
  body_up_ctrl_ = new ContactTransBodyCtrl(robot);
  body_fix_ctrl_ = new CoMzRxRyRzCtrl(robot);
  

  sp_->stance_foot_ = mercury_link::leftFoot;
  swing_start_trans_ctrl_ = 
      new TransitionCtrl(robot, swing_foot_, false);
  swing_ctrl_ = 
      new BodyFootCtrl(robot, swing_foot_);

  state_list_.clear();
  state_list_.push_back(jpos_ctrl_);
  state_list_.push_back(body_up_ctrl_);
  state_list_.push_back(body_fix_ctrl_);
  state_list_.push_back(swing_start_trans_ctrl_);
  state_list_.push_back(swing_ctrl_);


  _SettingParameter(handler);

  if(swing_foot_ = mercury_link::rightFoot){
   DataManager::GetDataManager()->RegisterData(
          &(((BodyFootCtrl*)swing_ctrl_)->curr_foot_pos_des_), 
          VECT3, "rfoot_pos_des", 3);
  DataManager::GetDataManager()->RegisterData(
          &(((BodyFootCtrl*)swing_ctrl_)->curr_foot_vel_des_), 
          VECT3, "rfoot_vel_des", 3);
  } else if (swing_foot_ == mercury_link::leftFoot){
   DataManager::GetDataManager()->RegisterData(
          &(((BodyFootCtrl*)swing_ctrl_)->curr_foot_pos_des_), 
          VECT3, "lfoot_pos_des", 3);

   DataManager::GetDataManager()->RegisterData(
          &(((BodyFootCtrl*)swing_ctrl_)->curr_foot_vel_des_), 
          VECT3, "lfoot_vel_des", 3);
  }

  printf("[Stance Swing Test] Constructed\n");
}

StanceSwingTest::~StanceSwingTest(){
  for(int i(0); i<state_list_.size(); ++i){
    delete state_list_[i];
  }
}

void StanceSwingTest::TestInitialization(){
  // Yaml file name
  jpos_ctrl_->CtrlInitialization("CTRL_jpos_initialization");
  body_up_ctrl_->CtrlInitialization("CTRL_move_to_target_height");
  body_fix_ctrl_->CtrlInitialization("CTRL_fix_des_pos");
  // Transition
  swing_start_trans_ctrl_->CtrlInitialization("CTRL_trans");
  // Swing
  swing_ctrl_->CtrlInitialization("CTRL_stance_swing");
}

int StanceSwingTest::_NextPhase(const int & phase){
  int next_phase = phase + 1;
  // printf("next phase: %i\n", next_phase);
  
  //if(phase == WKPhase::wk_double_contact_1) {
  if(next_phase == NUM_STANCE_SWING_PHASE) {
      printf("[Stance Swing Test] No more state \n");
      exit(0);
  }
  else{ return next_phase; }
}

void StanceSwingTest::_SettingParameter(ParamHandler& handler){
  double tmp; bool b_tmp;
  std::vector<double> tmp_vec;
  std::string tmp_str;
  //// Posture Setup
  // Initial JPos
  handler.getVector("initial_jpos", tmp_vec);
  ((JPosTargetCtrl*)jpos_ctrl_)->setTargetPosition(tmp_vec);
  // CoM Height
  handler.getValue("com_height", tmp);
  ((ContactTransBodyCtrl*)body_up_ctrl_)->setStanceHeight(tmp);
  ((CoMzRxRyRzCtrl*)body_fix_ctrl_)->setStanceHeight(tmp);

  ((TransitionCtrl*)swing_start_trans_ctrl_)->setStanceHeight(tmp);
  ((BodyFootCtrl*)swing_ctrl_)->setStanceHeight(tmp);

  //// Timing Setup
  handler.getValue("jpos_initialization_time", tmp);
  ((JPosTargetCtrl*)jpos_ctrl_)->setMovingTime(tmp);
  handler.getValue("com_lifting_time", tmp);
  ((ContactTransBodyCtrl*)body_up_ctrl_)->setStanceTime(tmp);
handler.getValue("transition_time", tmp);
((TransitionCtrl*)swing_start_trans_ctrl_)->setTransitionTime(tmp);
  // Stance Time
  handler.getValue("stance_time", tmp);
  ((CoMzRxRyRzCtrl*)body_fix_ctrl_)->setStanceTime(tmp);

  // Swing
  handler.getValue("swing_duration", tmp);
  ((BodyFootCtrl*)swing_ctrl_)->setSwingTime(tmp);
  handler.getValue("moving_preparation_time", tmp);
  ((BodyFootCtrl*)swing_ctrl_)->setMovingTime(tmp);
  handler.getValue("swing_height", tmp);
  ((BodyFootCtrl*)swing_ctrl_)->setSwingHeight(tmp);
  
  handler.getVector("amplitude", tmp_vec);
  ((BodyFootCtrl*)swing_ctrl_)->setAmplitude(tmp_vec);
  handler.getVector("frequency", tmp_vec);
  ((BodyFootCtrl*)swing_ctrl_)->setFrequency(tmp_vec);
  handler.getVector("phase", tmp_vec);
  ((BodyFootCtrl*)swing_ctrl_)->setPhase(tmp_vec);

  printf("[Stance Swing Test] Complete to Setup Parameters\n");
}
