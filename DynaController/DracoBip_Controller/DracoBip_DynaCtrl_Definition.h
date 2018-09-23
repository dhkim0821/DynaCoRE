#ifndef DRACO_BIPED_DYNACORE_CONTROL_DEFINITION
#define DRACO_BIPED_DYNACORE_CONTROL_DEFINITION

#include <Configuration.h>
#include <DracoBip/DracoBip_Definition.h>

#define DracoBipConfigPath THIS_COM"DynaController/DracoBip_Controller/DracoBipTestConfig/"

class DracoBip_SensorData{
    public:
        double imu_ang_vel[3];
        double imu_acc[3];

        double torque[dracobip::num_act_joint];
        double jpos[dracobip::num_act_joint];
        double jvel[dracobip::num_act_joint];

        double rotor_inertia[dracobip::num_act_joint];
        bool rfoot_contact;
        bool lfoot_contact;
};

class DracoBip_Command{
    public:
        double jtorque_cmd[dracobip::num_act_joint];
        double jpos_cmd[dracobip::num_act_joint];
        double jvel_cmd[dracobip::num_act_joint];
};


#endif
