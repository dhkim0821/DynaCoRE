#ifndef VALKYRIE_MULTI_TASK_CTRL
#define VALKYRIE_MULTI_TASK_CTRL

#include <Controller.hpp>
#include <chrono>

class Valkyrie_StateProvider;
class WBDC;
class WBDC_ExtraData;
class Task;
class WBDC_ContactSpec;

class MultiTaskCtrl: public Controller{
    public:
        MultiTaskCtrl(RobotSystem*);
        virtual ~MultiTaskCtrl();

        virtual void OneStep(dynacore::Vector & gamma);
        virtual void FirstVisit();
        virtual void LastVisit();
        virtual bool EndOfPhase();

        virtual void CtrlInitialization(const std::string & setting_file_name);

    protected:
        WBDC* wbdc_;
        WBDC_ExtraData* wbdc_data_;
        Task* centroid_task_;
        Task* jpos_task_;
        Task* body_ori_task_;
        Task* pelvis_ori_task_;
        Task* rpalm_pos_task_;
        Task* lpalm_pos_task_;
        Task* head_ori_task_;
        WBDC_ContactSpec* double_contact_;

        void _wblc_ctrl(dynacore::Vector & gamma);
        void _double_contact_setup();
        void _centroid_task_setup();
        void _body_ori_task_setup();
        void _pelvis_ori_task_setup();
        void _right_palm_task_setup();
        void _left_palm_task_setup();
        void _jpos_task_setup();
        void _head_ori_task_setup();
    
        Valkyrie_StateProvider* sp_;
        dynacore::Vector ini_jpos_;
        dynacore::Vect3 ini_com_;
        dynacore::Vect3 ini_rpalm_;
        dynacore::Vect3 ini_lpalm_;
        dynacore::Quaternion ini_body_ori_;
        dynacore::Quaternion ini_pelvis_ori_;
        dynacore::Quaternion ini_head_ori_;

        bool time_measure_;
        std::chrono::high_resolution_clock::time_point t_s_;
        std::chrono::high_resolution_clock::time_point t_f_;
        std::chrono::duration<double> time_span_;

        bool data_save_;

};

#endif
