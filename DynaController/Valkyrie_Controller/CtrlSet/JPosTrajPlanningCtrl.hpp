#ifndef JOINT_POSITION_TRAJECTORY_WALKING_CONTROL_VALKYRIE
#define JOINT_POSITION_TRAJECTORY_WALKING_CONTROL_VALKYRIE

#include "SwingPlanningCtrl.hpp"
#include <Valkyrie_Controller/Valkyrie_InvKinematics.hpp>
#include <Utils/minjerk_one_dim.hpp>

class Valkyrie_StateProvider;
class WBDC_ContactSpec;

class JPosTrajPlanningCtrl:public SwingPlanningCtrl{
    public:
        JPosTrajPlanningCtrl(const RobotSystem* robot, int swing_foot, Planner* planner);
        virtual ~JPosTrajPlanningCtrl();
        virtual void OneStep(void* _cmd);
        virtual void FirstVisit();
        virtual void LastVisit();
        virtual bool EndOfPhase();
        virtual void CtrlInitialization(const std::string & setting_file_name);

    protected:
        double pitch_offset_gain_;
        double roll_offset_gain_;
        dynacore::Vect2 body_pt_offset_;

        int swing_leg_roll_jidx_;
        int swing_leg_pitch_jidx_;

        void _SetMinJerkTraj(
                double moving_duration,
                const dynacore::Vector & st_pos,
                const dynacore::Vector & st_vel,
                const dynacore::Vector & st_acc,
                const dynacore::Vector & target_pos,
                const dynacore::Vector & target_vel,
                const dynacore::Vector & target_acc);


        dynacore::Vector ini_swing_leg_config_;
        dynacore::Vector mid_swing_leg_config_;
        dynacore::Vector target_swing_leg_config_;

        double initial_traj_mix_ratio_;
        double kp_x_;
        double kp_y_;

        int swing_leg_jidx_;
        double push_down_height_; // push foot below the ground at landing
        dynacore::Vect3 default_target_loc_;

        dynacore::Vector task_kp_;
        dynacore::Vector task_kd_;

        void _setTaskGain(const dynacore::Vector & Kp, const dynacore::Vector & Kd);

        Task* config_body_foot_task_;
        void _CheckPlanning();
        void _Replanning(dynacore::Vect3 & target_loc);


        dynacore::Vector des_jpos_;
        dynacore::Vector des_jvel_;

        dynacore::Vect3 ini_body_pos_;
        dynacore::Vect3 ini_com_pos_;
        dynacore::Vect3 ini_foot_pos_;
        dynacore::Vect3 target_foot_pos_;
        
        dynacore::Vector ini_config_;
        std::vector<MinJerk_OneDimension*> min_jerk_jpos_initial_;

        void _task_setup();
        void _single_contact_setup();
        void _body_foot_ctrl(dynacore::Vector & gamma);

        Valkyrie_InvKinematics* inv_kin_;
        double ctrl_start_time_;
};

#endif
