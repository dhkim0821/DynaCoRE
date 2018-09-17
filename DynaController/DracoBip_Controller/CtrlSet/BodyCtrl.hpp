#ifndef BODY_CTRL
#define BODY_CTRL

#include <Controller.hpp>

class DracoBip_StateProvider;
class RobotSystem;
class WBLC;
class WBLC_ExtraData;
class KinTask;
class WBDC_ContactSpec;

class BodyCtrl: public Controller{
    public:
        BodyCtrl(RobotSystem* );
        virtual ~BodyCtrl();

        virtual void OneStep(void* _cmd);
        virtual void FirstVisit();
        virtual void LastVisit();
        virtual bool EndOfPhase();

        virtual void CtrlInitialization(const std::string & setting_file_name);

        void setStanceTime(double time) { end_time_ = time; }
        void setStanceHeight(double height){ 
            target_body_height_ = height;
            b_set_height_target_ = true;
         }

    protected:
        dynacore::Vector Kp_, Kd_;
        dynacore::Vector des_jpos_; 
        dynacore::Vector des_jvel_; 
        dynacore::Vector des_jacc_;

        dynacore::Vector jpos_ini_;
        bool b_set_height_target_;
        int trj_type_;
        double end_time_;

        KinTask* body_task_;
        WBDC_ContactSpec* double_contact_;
        WBLC* wblc_;
        WBLC_ExtraData* wblc_data_;

        double target_body_height_;
        double ini_body_height_;

        bool b_body_set_;

        void _body_task_setup();
        void _double_contact_setup();
        void _compute_torque_wblc(dynacore::Vector & gamma);

        double ctrl_start_time_;
        DracoBip_StateProvider* sp_;
};

#endif
