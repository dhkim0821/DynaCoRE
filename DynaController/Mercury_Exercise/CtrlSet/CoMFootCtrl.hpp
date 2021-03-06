#ifndef COM_STANCE_FOOT_CONTROL
#define COM_STANCE_FOOT_CONTROL

#include <Controller.hpp>
#include <Utils/BSplineBasic.h>

class Mercury_StateProvider;
class RobotSystem;
class WBDC_ContactSpec;
class WBDC_Rotor;
class WBDC_Rotor_ExtraData;

class CoMFootCtrl:public Controller{
    public:
        CoMFootCtrl(RobotSystem* robot, int swing_foot);
        ~CoMFootCtrl();
        virtual void OneStep(dynacore::Vector & gamma);
        virtual void FirstVisit();
        virtual void LastVisit();
        virtual bool EndOfPhase();

        virtual void CtrlInitialization(const std::string & setting_file_name);

        void setStanceHeight(double height) {
            des_com_height_ = height;
            b_set_height_target_ = true;
        }

        dynacore::Vect3 curr_foot_pos_des_;
        dynacore::Vect3 curr_foot_vel_des_;
        dynacore::Vect3 curr_foot_acc_des_;

        void setAmplitude(const std::vector<double> & amp){ amp_ = amp; }
        void setFrequency(const std::vector<double> & freq){ freq_ = freq; }
        void setPhase(const std::vector<double> & phase){ phase_ = phase; }
        void setMovingTime(double time){ moving_time_ = time; }
        void setSwingTime(double time){ swing_time_ = time; }
        void setSwingHeight(double height){ swing_height_ = height; }

    protected:
        void _SetBspline(const dynacore::Vect3 & st_pos,
                const dynacore::Vect3 & st_vel,
                const dynacore::Vect3 & st_acc,
                const dynacore::Vect3 & target_pos,
                const dynacore::Vect3 & target_vel,
                const dynacore::Vect3 & target_acc);

        std::vector<double> amp_;
        std::vector<double> freq_;
        std::vector<double> phase_;

        bool b_set_height_target_;
        double des_com_height_;

        int swing_foot_;
        double swing_height_;
        dynacore::Vect3 foot_pos_set_;

        Task* com_foot_task_;
        WBDC_ContactSpec* single_contact_;

        WBDC_Rotor* wbdc_rotor_;
        WBDC_Rotor_ExtraData* wbdc_rotor_data_;

        dynacore::Vect3 ini_com_pos_;
        dynacore::Vect3 ini_foot_pos_;
        dynacore::Vect3 target_foot_pos_;

        BS_Basic<3, 3, 0, 2, 2> foot_traj_;

        double swing_time_;
        double moving_time_;

        void _task_setup();
        void _single_contact_setup();
        void _com_foot_ctrl(dynacore::Vector & gamma);

        Mercury_StateProvider* sp_;
        double ctrl_start_time_;
};
#endif
