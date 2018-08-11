#ifndef CONFIGURATION_BODY_FOOT_WALKING_CONTROL
#define CONFIGURATION_BODY_FOOT_WALKING_CONTROL

#include "SwingPlanningCtrl.hpp"
#include <Mercury_Controller/Mercury_InvKinematics.hpp>
#include <Utils/minjerk_one_dim.hpp>

class WBWC;

class ConfigBodyFootPlanningCtrl:public SwingPlanningCtrl{
   public:
        ConfigBodyFootPlanningCtrl(const RobotSystem* robot, int swing_foot, Planner* planner);
        ~ConfigBodyFootPlanningCtrl();
        virtual void OneStep(void* _cmd);
        virtual void FirstVisit();
        virtual void LastVisit(){}
        virtual bool EndOfPhase();

        virtual void CtrlInitialization(const std::string & setting_file_name);
    protected:
        int swing_leg_jidx_;
        double push_down_height_; // push foot below the ground at landing
        dynacore::Vect3 default_target_loc_;
    
        void _CheckPlanning();
        void _Replanning(dynacore::Vect3 & target_loc);
        void _task_setup();
        void _body_foot_ctrl(dynacore::Vector & gamma);
        void _SetMinJerkOffset(const dynacore::Vect3 & offset);

        WBWC* wbwc_;

        dynacore::Vector des_jpos_;
        dynacore::Vector des_jvel_;
        dynacore::Vector des_jacc_;

        dynacore::Vect3 ini_body_pos_;
        dynacore::Vect3 ini_com_pos_;
        dynacore::Vect3 ini_foot_pos_;
        dynacore::Vect2 body_pt_offset_;
        
        dynacore::Vector ini_config_;

        std::vector<double> foot_landing_offset_;

        Mercury_InvKinematics inv_kin_;
        std::vector<MinJerk_OneDimension*> min_jerk_offset_;
};

#endif
