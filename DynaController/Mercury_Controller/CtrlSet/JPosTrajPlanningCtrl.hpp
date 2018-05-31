#ifndef JOINT_POSITION_TRAJECTORY_WALKING_CONTROL
#define JOINT_POSITION_TRAJECTORY_WALKING_CONTROL

#include "SwingPlanningCtrl.hpp"
#include <Utils/BSplineBasic.h>
#include <Mercury_Controller/StateEstimator/LIPM_KalmanFilter.hpp>
#include <Mercury_Controller/Mercury_InvKinematics.hpp>

class Mercury_StateProvider;
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
        void _SetBspline(
                const dynacore::Vect3 & st_pos,
                const dynacore::Vect3 & st_vel,
                const dynacore::Vect3 & st_acc,
                const dynacore::Vect3 & target_pos);
        void _SetJPosBspline(const dynacore::Vector & st_pos, 
                const dynacore::Vector & target_pos, BS_Basic<4,4,0,3,3> & spline);

        dynacore::Vector ini_swing_leg_config_;
        dynacore::Vector mid_swing_leg_config_;
        dynacore::Vector target_swing_leg_config_;

        double kp_x_;
        double kp_y_;

        int swing_leg_jidx_;
        double push_down_height_; // push foot below the ground at landing
        dynacore::Vect3 default_target_loc_;

        std::vector<double> foot_landing_offset_;

        dynacore::Vector task_kp_;
        dynacore::Vector task_kd_;

        double gain_decreasing_ratio_;
        double gain_decreasing_period_portion_;
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
        dynacore::Vect2 body_pt_offset_;

        dynacore::Vect2 prev_ekf_vel;
        dynacore::Vect2 acc_err_ekf;
        
        dynacore::Vector ini_config_;
        BS_Basic<3, 3, 1, 2, 2> foot_traj_;
        BS_Basic<4, 4, 0, 3, 3> mid_jpos_traj_;
        BS_Basic<4, 4, 0, 3, 3> end_jpos_traj_;

        double swing_time_reduction_;

        void _task_setup();
        void _single_contact_setup();
        void _body_foot_ctrl(dynacore::Vector & gamma);

        Mercury_InvKinematics inv_kin_;
        double ctrl_start_time_;
};

#endif
