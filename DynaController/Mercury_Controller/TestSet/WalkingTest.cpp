#include "WalkingTest.hpp"
#include <Mercury_Controller/Mercury_StateProvider.hpp>

#include <Mercury_Controller/CtrlSet/JPosTargetCtrl.hpp>
#include <Mercury_Controller/CtrlSet/ContactTransBodyCtrl.hpp>
#include <Mercury_Controller/CtrlSet/CoMzRxRyRzCtrl.hpp>
#include <Mercury_Controller/CtrlSet/BodyFootPlanningCtrl.hpp>
#include <Mercury_Controller/CtrlSet/TransitionCtrl.hpp>
#include <ParamHandler/ParamHandler.hpp>
#include <Planner/PIPM_FootPlacementPlanner/Reversal_LIPM_Planner.hpp>
#include <Utils/DataManager.hpp>

#include <Mercury/Mercury_Model.hpp>
#include <Mercury/Mercury_Definition.h>

WalkingTest::WalkingTest(RobotSystem* robot):Test(robot),
                           num_step_(0)
{
  sp_ = Mercury_StateProvider::getStateProvider();
  sp_->stance_foot_ = mercury_link::leftFoot;
  sp_->global_pos_local_[1] = 0.15;
robot_sys_ = robot;
  reversal_planner_ = new Reversal_LIPM_Planner();
  // phase_ = WKPhase::wk_lift_up;
  phase_ = WKPhase::wk_initiation;

  state_list_.clear();

  jpos_ctrl_ = new JPosTargetCtrl(robot);
  body_up_ctrl_ = new ContactTransBodyCtrl(robot);
  body_fix_ctrl_ = new CoMzRxRyRzCtrl(robot);
  // Right
  right_swing_start_trans_ctrl_ = 
      new TransitionCtrl(robot, mercury_link::rightFoot, false);
  right_swing_ctrl_ = 
      new BodyFootPlanningCtrl(robot, mercury_link::rightFoot, reversal_planner_);
  right_swing_end_trans_ctrl_ = 
      new TransitionCtrl(robot, mercury_link::rightFoot, true);
  // Left
  left_swing_start_trans_ctrl_ = 
      new TransitionCtrl(robot, mercury_link::leftFoot, false);
  left_swing_ctrl_ = 
      new BodyFootPlanningCtrl(robot, mercury_link::leftFoot, reversal_planner_);
  left_swing_end_trans_ctrl_ = 
      new TransitionCtrl(robot, mercury_link::leftFoot, true);

  state_list_.push_back(jpos_ctrl_);
  state_list_.push_back(body_up_ctrl_);
  state_list_.push_back(body_fix_ctrl_);
  state_list_.push_back(right_swing_start_trans_ctrl_);
  state_list_.push_back(right_swing_ctrl_);
  state_list_.push_back(right_swing_end_trans_ctrl_);
  state_list_.push_back(body_fix_ctrl_);
  state_list_.push_back(left_swing_start_trans_ctrl_);
  state_list_.push_back(left_swing_ctrl_);
  state_list_.push_back(left_swing_end_trans_ctrl_);

  _SettingParameter();

  DataManager::GetDataManager()->RegisterData(&(((BodyFootPlanningCtrl*)right_swing_ctrl_)->curr_foot_pos_des_), VECT3, "rfoot_pos_des", 3);
  DataManager::GetDataManager()->RegisterData(&(((BodyFootPlanningCtrl*)left_swing_ctrl_)->curr_foot_pos_des_), VECT3, "lfoot_pos_des", 3);


  DataManager::GetDataManager()->RegisterData(&(((BodyFootPlanningCtrl*)right_swing_ctrl_)->curr_foot_vel_des_), VECT3, "rfoot_vel_des", 3);
  DataManager::GetDataManager()->RegisterData(&(((BodyFootPlanningCtrl*)left_swing_ctrl_)->curr_foot_vel_des_), VECT3, "lfoot_vel_des", 3);

  printf("[Walking Test] Constructed\n");
}

WalkingTest::~WalkingTest(){
  for(int i(0); i<state_list_.size(); ++i){
    delete state_list_[i];
  }
}

void WalkingTest::TestInitialization(){
  // Planner
  reversal_planner_->PlannerInitialization("PLANNER_velocity_reversal");

  // Yaml file name
  jpos_ctrl_->CtrlInitialization("CTRL_jpos_initialization");
  body_up_ctrl_->CtrlInitialization("CTRL_move_to_target_height");
  body_fix_ctrl_->CtrlInitialization("CTRL_fix_des_pos");

  // Transition
  right_swing_start_trans_ctrl_->CtrlInitialization("CTRL_trans");
  right_swing_end_trans_ctrl_->CtrlInitialization("CTRL_trans");
  left_swing_start_trans_ctrl_->CtrlInitialization("CTRL_trans");
  left_swing_end_trans_ctrl_->CtrlInitialization("CTRL_trans");

  // Swing
  right_swing_ctrl_->CtrlInitialization("CTRL_right_walking_swing");
  left_swing_ctrl_->CtrlInitialization("CTRL_left_walking_swing");
}

int WalkingTest::_NextPhase(const int & phase){
  int next_phase = phase + 1;
  // printf("next phase: %i\n", next_phase);
  
  if(phase == WKPhase::wk_double_contact_1) {
    ++num_step_;
    printf("%i th step:\n", num_step_);
    // printf("One swing done: Next Right Leg Swing\n");
    sp_->stance_foot_ = mercury_link::leftFoot;

    // Global Frame Update
    dynacore::Vect3 next_local_frame_location;
    robot_sys_->getPos(mercury_link::leftFoot, next_local_frame_location);
    sp_->global_pos_local_.head(2) += next_local_frame_location.head(2);
  }
  if(phase == WKPhase::wk_double_contact_2){
    ++num_step_;
    printf("%i th step:\n", num_step_);

    // printf("One swing done: Next Left Leg Swing\n");
    sp_->stance_foot_ = mercury_link::rightFoot;

    // Global Frame Update
    dynacore::Vect3 next_local_frame_location;
    robot_sys_->getPos(mercury_link::rightFoot, next_local_frame_location);
    sp_->global_pos_local_.head(2) += next_local_frame_location.head(2);
  }
  if(next_phase == NUM_WALKING_PHASE) {
    return WKPhase::wk_double_contact_1;
  }
  else{ return next_phase; }
}

void WalkingTest::_SettingParameter(){
  // Setting Parameters
  ParamHandler handler(MercuryConfigPath"TEST_walking.yaml");

  double tmp;
  std::vector<double> tmp_vec;
  std::string tmp_str;
  // Start Phase
  handler.getInteger("start_phase", phase_);

  //// Posture Setup
  // Initial JPos
  handler.getVector("initial_jpos", tmp_vec);
  ((JPosTargetCtrl*)jpos_ctrl_)->setTargetPosition(tmp_vec);
  // CoM Height
  handler.getValue("com_height", tmp);
  ((ContactTransBodyCtrl*)body_up_ctrl_)->setStanceHeight(tmp);
  ((CoMzRxRyRzCtrl*)body_fix_ctrl_)->setStanceHeight(tmp);

  ((TransitionCtrl*)right_swing_start_trans_ctrl_)->setStanceHeight(tmp);
  ((TransitionCtrl*)right_swing_end_trans_ctrl_)->setStanceHeight(tmp);
  ((TransitionCtrl*)left_swing_start_trans_ctrl_)->setStanceHeight(tmp);
  ((TransitionCtrl*)left_swing_end_trans_ctrl_)->setStanceHeight(tmp);

  ((BodyFootPlanningCtrl*)right_swing_ctrl_)->setStanceHeight(tmp);
  ((BodyFootPlanningCtrl*)left_swing_ctrl_)->setStanceHeight(tmp);
  ((Reversal_LIPM_Planner*)reversal_planner_)->setOmega(tmp);

  //// Timing Setup
  handler.getValue("jpos_initialization_time", tmp);
  ((JPosTargetCtrl*)jpos_ctrl_)->setMovingTime(tmp);
  handler.getValue("com_lifting_time", tmp);
  ((ContactTransBodyCtrl*)body_up_ctrl_)->setStanceTime(tmp);

  // Stance Time
  handler.getValue("stance_time", tmp);
  ((CoMzRxRyRzCtrl*)body_fix_ctrl_)->setStanceTime(tmp);
  ((BodyFootPlanningCtrl*)right_swing_ctrl_)->notifyStanceTime(tmp);
  ((BodyFootPlanningCtrl*)left_swing_ctrl_)->notifyStanceTime(tmp);

  // Swing & prime Time
  handler.getValue("swing_time", tmp);
  ((BodyFootPlanningCtrl*)right_swing_ctrl_)->setSwingTime(tmp);
  ((BodyFootPlanningCtrl*)left_swing_ctrl_)->setSwingTime(tmp);

  // Transition Time
  handler.getValue("st_transition_time", tmp);
  ((TransitionCtrl*)right_swing_start_trans_ctrl_)->setTransitionTime(tmp);
  ((TransitionCtrl*)right_swing_end_trans_ctrl_)->setTransitionTime(tmp);
  ((TransitionCtrl*)left_swing_start_trans_ctrl_)->setTransitionTime(tmp);
  ((TransitionCtrl*)left_swing_end_trans_ctrl_)->setTransitionTime(tmp);

  ((BodyFootPlanningCtrl*)right_swing_ctrl_)->notifyTransitionTime(tmp);
  ((BodyFootPlanningCtrl*)left_swing_ctrl_)->notifyTransitionTime(tmp);

  //// Planner Setup
  handler.getValue("planning_frequency", tmp);
  ((BodyFootPlanningCtrl*)right_swing_ctrl_)->setPlanningFrequency(tmp);
  ((BodyFootPlanningCtrl*)left_swing_ctrl_)->setPlanningFrequency(tmp);

  handler.getValue("double_stance_mix_ratio", tmp);
  ((BodyFootPlanningCtrl*)right_swing_ctrl_)->setDoubleStanceRatio(tmp);
  ((BodyFootPlanningCtrl*)left_swing_ctrl_)->setDoubleStanceRatio(tmp);

  handler.getValue("transition_phase_mix_ratio", tmp);
  ((BodyFootPlanningCtrl*)right_swing_ctrl_)->setTransitionPhaseRatio(tmp);
  ((BodyFootPlanningCtrl*)left_swing_ctrl_)->setTransitionPhaseRatio(tmp);

  printf("[Walking Test] Complete to Setup Parameters\n");
}
