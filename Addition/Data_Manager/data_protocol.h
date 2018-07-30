#ifndef DATA_PROTOCOL
#define DATA_PROTOCOL
#include <stdint.h>
// Hume System
#define PORT_HUME_SYSTEM       61123
#define PORT_DATA_SETUP        61124
#define PORT_DATA_RECEIVE      61125

#define IP_ADDR_MOCAP "192.168.1.149"
#define IP_ADDR_MYSELF "127.0.0.1"
#define IP_ADDR_MAC    "192.168.1.131"
#define IP_ADDR_DH    "192.168.50.159"

#define IP_ADDR IP_ADDR_DH
//#define IP_ADDR IP_ADDR_MYSELF

#define LEG_PORT 51129
#define NUM_JOINT_Q 13
#define NUM_JOINT_QDOT 12

#define TASK_SIZE 10
#define MAX_NUM_DATA 100

#include <Configuration.h>

namespace DATA_Protocol{
    typedef struct{
        int num_data;
        int tot_num_array_data;
        char data_name[MAX_NUM_DATA][20];
        int num_array_data[MAX_NUM_DATA];
    }DATA_SETUP;
};

namespace HumeProtocol{

    typedef struct{
        int count;
        int phase;

        double tot_conf[NUM_JOINT_Q];
        double tot_vel[NUM_JOINT_QDOT];

        //double torque_input[NUM_ACT_JOINT];
        //double curr_torque[NUM_ACT_JOINT];
        double curr_time;

        double ori[3];
        double ori_vel[3];

        double com_pos[3];
        double com_vel[3];
        double com_vel_filter[3];
        double rfoot_force[3];
        double lfoot_force[3];

        double rfoot_pos[3]; 
        double lfoot_pos[3]; 
        double rfoot_vel[3]; 
        double lfoot_vel[3]; 
        
        double rfoot_pos_des[3]; 
        double lfoot_pos_des[3]; 
        double rfoot_vel_des[3]; 
        double lfoot_vel_des[3];

        bool rcontact;
        bool lcontact;

        bool task_start;
        
        double body_pos[3];
        double ori_mtx[9];

        double kalman_foot_pos[9];
        double com_kalman_filter[3];
        
        double mocap_body_vel[3];
        double mocap_leg_led_x[10];
        double mocap_leg_led_y[10];
        double mocap_leg_led_z[10];

        double plant_observation[21];
        double task_time;
        double task_des[TASK_SIZE];
        double task_curr[TASK_SIZE];
        double task_vel_des[TASK_SIZE];
        double task_vel_curr[TASK_SIZE];

    }hume_system_data;

    typedef struct{
        uint32_t index;
        uint32_t timeStamp;
        double x[32];
        double y[32];
        double z[32];
        uint32_t validBits;
        uint32_t extraInt;
    } message_mocap;
    
    typedef struct
    {
        double body_pos[3];
        double body_vel[3];

        double lfoot_pos[3];
        double rfoot_pos[3];

        double body_des[3];
    }body_ctrl_data;

    typedef struct {
        double body_des[2];
        double body_vel_des[2];
        double body_pos[2];
        double body_vel[2];
        double input[2];
    }body_xz_task;

    typedef struct{  // Yaw, Pitch, Roll
        double ori_des[3];
        double ori_vel_des[3];
        double ori[3];
        double ori_vel[3];
    }ori_task_data;
    
    typedef struct {
        double body_des[3];
        double body_vel_des[3];
        double body_pos[3];
        double body_vel[3];
    }body_task_data;

    typedef struct {
        double foot_des[3];
        double foot_vel_des[3];
        double foot_pos[3];
        double foot_vel[3];
    }foot_task_data;
    
    typedef struct{
        double kp;
        double kd;
    }pd_1_gain;
    
    typedef struct{
        double kp[2];
        double kd[2];
    }pd_2_gain;

    typedef struct{
        double kp[3];
        double kd[3];
    }pd_3_gain;


    typedef struct{
        double kp[4];
        double kd[4];
    }pd_4_gain;

    
    typedef struct{
        double kp[5];
        double kd[5];
    }pd_5_gain;

    typedef struct{
        double kp[6];
        double kd[6];
    }pd_6_gain;
    
    typedef struct {
        double height_des;
        double height_vel_des;
        double height;
        double height_vel;
    }height_task;

    typedef struct{
        double foot_des[3];
        double foot_vel_des[3];
        double foot_pos[3];
        double foot_vel[3];
    }foot_task;
    
    typedef struct {
        double ang_des;
        double ang;
        double ang_vel_des;
        double ang_vel;
        double input;
    }tilting_task;

    typedef struct {
        //int16_t kp[NUM_ACT_JOINT];
        //int16_t ki[NUM_ACT_JOINT];
    }pi_joint_gain;

}

#endif
