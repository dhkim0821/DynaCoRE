#ifndef CONTACT_TRANSITION_CONFIGURATION_CTRL_VALKYRIE
#define CONTACT_TRANSITION_CONFIGURATION_CTRL_VALKYRIE

#include <Controller.hpp>

class Valkyrie_StateProvider;
class RobotSystem;
class WBDC_ContactSpec;
class WBDC;
class WBDC_ExtraData;
class Valkyrie_InvKinematics;

class ContactTransConfigCtrl: public Controller{
    public:
        ContactTransConfigCtrl(RobotSystem* );
        virtual ~ContactTransConfigCtrl();

        virtual void OneStep(void* _cmd);
        virtual void FirstVisit();
        virtual void LastVisit();
        virtual bool EndOfPhase();

        virtual void CtrlInitialization(const std::string & setting_file_name);

        void setStanceTime(double stance_time){ end_time_ = stance_time; }
        void setStanceHeight(double height) {
            des_body_height_ = height;
            b_set_height_target_ = true;
        }

    protected:
        bool b_set_height_target_;
        double des_body_height_;
        double max_rf_z_;
        double min_rf_z_;

        Task* body_task_;
        WBDC_ContactSpec* double_contact_;
        WBDC* wbdc_;
        WBDC_ExtraData* wbdc_data_;

        dynacore::Vector body_pos_ini_;
        dynacore::Vect3 ini_body_pos_;

        dynacore::Vector des_jpos_;
        dynacore::Vector des_jvel_;
        
        
        dynacore::Vector config_des_;


        double end_time_;
        void _body_task_setup();
        void _double_contact_setup();
        void _body_ctrl_wbdc(dynacore::Vector & gamma);
        double ctrl_start_time_;
        Valkyrie_StateProvider* sp_;
        Valkyrie_InvKinematics* inv_kin_;
};

#endif
