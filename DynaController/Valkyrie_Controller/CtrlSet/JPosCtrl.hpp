#ifndef VALKYRIE_JOINT_POSITION_CTRL
#define VALKYRIE_JOINT_POSITION_CTRL

#include <Controller.hpp>
#include <chrono>

class Valkyrie_StateProvider;
class WBDC;
class WBDC_ExtraData;
class Task;
class WBDC_ContactSpec;

class JPosCtrl: public Controller{
    public:
        JPosCtrl(RobotSystem*);
        virtual ~JPosCtrl();

        virtual void OneStep(dynacore::Vector & gamma);
        virtual void FirstVisit();
        virtual void LastVisit();
        virtual bool EndOfPhase();

        virtual void CtrlInitialization(const std::string & setting_file_name);

    protected:
        WBDC* wbdc_;
        WBDC_ExtraData* wbdc_data_;
        Task* jpos_task_;
        WBDC_ContactSpec* double_contact_;

        void _jpos_ctrl(dynacore::Vector & gamma);
        void _double_contact_setup();
        void _jpos_task_setup();

        void _SvNcmCheckTest();

        Valkyrie_StateProvider* sp_;
        dynacore::Vector ini_jpos_;

        bool time_measure_;
        std::chrono::high_resolution_clock::time_point t_s_;
        std::chrono::high_resolution_clock::time_point t_f_;
        std::chrono::duration<double> time_span_;
};

#endif
