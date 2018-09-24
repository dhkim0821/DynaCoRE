#ifndef SagitP3_DYNACORE_CONTROL_DEFINITION
#define SagitP3_DYNACORE_CONTROL_DEFINITION

#include <Configuration.h>
#include <SagitP3/SagitP3_Definition.h>

#define SagitP3ConfigPath THIS_COM"DynaController/SagitP3_Controller/SagitP3TestConfig/"

class SagitP3_SensorData{
    public:
        double imu_ang_vel[3];
        double imu_acc[3];

        double torque[sagitP3::num_act_joint];
        double jpos[sagitP3::num_act_joint];
        double jvel[sagitP3::num_act_joint];

        double rotor_inertia[sagitP3::num_act_joint];
        bool rfoot_contact;
        bool lfoot_contact;
};

class SagitP3_Command{
    public:
        double jtorque_cmd[sagitP3::num_act_joint];
        double jpos_cmd[sagitP3::num_act_joint];
        double jvel_cmd[sagitP3::num_act_joint];
};


#endif
