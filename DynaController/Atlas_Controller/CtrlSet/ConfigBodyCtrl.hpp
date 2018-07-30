#ifndef CONFIGURATION_BODY_CTRL_ATLAS
#define CONFIGURATION_BODY_CTRL_ATLAS

#include <Controller.hpp>
#include <Atlas_Controller/Atlas_InvKinematics.hpp>


class Atlas_StateProvider;
class RobotSystem;
class WBDC_ContactSpec;
class WBDC;
class WBDC_ExtraData;


class ConfigBodyCtrl: public Controller{
    public:
        ConfigBodyCtrl(RobotSystem* );
        virtual ~ConfigBodyCtrl();

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
        bool b_set_height_target_;
        int trj_type_;
        double end_time_;

        Task* jpos_task_;
        WBDC_ContactSpec* double_body_contact_;
        WBDC* wbdc_;
        WBDC_ExtraData* wbdc_data_;

        dynacore::Vector jpos_ini_;
        dynacore::Vector jpos_target_;
        dynacore::Vector des_jpos_;
        dynacore::Vector des_jvel_;

        double target_body_height_;
        double ini_body_height_;

        bool b_jpos_set_;

        void _jpos_task_setup();
        void _double_body_contact_setup();
        void _jpos_ctrl_wbdc(dynacore::Vector & gamma);

        double ctrl_start_time_;
        Atlas_StateProvider* sp_;
        Atlas_InvKinematics* inv_kin_;
};

#endif
