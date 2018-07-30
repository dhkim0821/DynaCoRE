#include "valkyrie_Dyn_environment.h"
#include <iostream>
#include <stdio.h>
#include "srDyn/srSpace.h"
#include "common/utils.h"

#include <DynaController/Valkyrie_Controller/Valkyrie_interface.hpp>
#include <DynaController/Valkyrie_Controller/Valkyrie_DynaControl_Definition.h>

#include <srTerrain/Ground.h>
#include <srConfiguration.h>

Valkyrie_Dyn_environment::Valkyrie_Dyn_environment()
{
    /********** Space Setup **********/
    m_Space = new srSpace();
    m_ground = new Ground();

    m_Space->AddSystem(m_ground->BuildGround());

    /********** Robot Set  **********/
    robot_ = new New_Valkyrie();
    //robot_->BuildRobot(Vec3 (0., 0., 0.), srSystem::FIXED, 
    //srJoint::TORQUE, ModelPath"Valkyrie_Model/r5_urdf.urdf");
    robot_->BuildRobot(Vec3 (0., 0., 0.), srSystem::FIXED, 
            srJoint::TORQUE, ModelPath"Valkyrie/valkyrie_simple.urdf");
    m_Space->AddSystem((srSystem*)robot_);

    /******** Interface set ********/
    interface_ = new Valkyrie_interface();
    data_ = new Valkyrie_SensorData();
    cmd_ = new Valkyrie_Command();

    m_Space->DYN_MODE_PRESTEP();
    m_Space->SET_USER_CONTROL_FUNCTION_2(ControlFunction);
    m_Space->SetTimestep(valkyrie::servo_rate);
    m_Space->SetGravity(0.0,0.0,-9.81);

    m_Space->SetNumberofSubstepForRendering(50);

    printf("[Valkyrie Dynamic Environment] Build Dynamic Environment\n");
}

void Valkyrie_Dyn_environment::ControlFunction( void* _data ) {
    static int count(0);
    ++count;

    Valkyrie_Dyn_environment* pDyn_env = (Valkyrie_Dyn_environment*)_data;
    New_Valkyrie* robot = (New_Valkyrie*)(pDyn_env->robot_);
    Valkyrie_SensorData* p_data = pDyn_env->data_;

    //printf("num act joint: %d \n", robot->num_act_joint_);
    std::vector<double> torque_command(robot->num_act_joint_);

    for(int i(0); i<robot->num_act_joint_; ++i){
        p_data->jpos[i] = robot->r_joint_[i]->m_State.m_rValue[0];
        p_data->jvel[i] = robot->r_joint_[i]->m_State.m_rValue[1];
    }


    pDyn_env->_CheckFootContact(
            p_data->rfoot_contact, p_data->lfoot_contact);

    for (int i(0); i<3; ++i){
        p_data->imu_ang_vel[i] = 
            robot->link_[robot->link_idx_map_.find("pelvis")->second]->GetVel()[i];
    }
    pDyn_env->interface_->GetCommand(p_data, pDyn_env->cmd_); 

    // Set Command
    for(int i(0); i<3; ++i){
        robot->vp_joint_[i]->m_State.m_rCommand = 0.0;
        robot->vr_joint_[i]->m_State.m_rCommand = 0.0;
    }

    if( count < 100 ){
        robot->vp_joint_[0]->m_State.m_rCommand = 
            -5000. * robot->vp_joint_[0]->m_State.m_rValue[0]
            - 10. * robot->vp_joint_[0]->m_State.m_rValue[1];
        robot->vp_joint_[1]->m_State.m_rCommand = 
            -5000. * robot->vp_joint_[1]->m_State.m_rValue[0]
            - 10. * robot->vp_joint_[1]->m_State.m_rValue[1];
    }

    double Kp(200.);
    double Kd(5.);
     //double Kp(30.);
     //double Kd(0.5);
    // Right
    double ramp(1.);
    if( count < 10 ){
        ramp = ((double)count)/10.;
    }
    for(int i(0); i<robot->num_act_joint_; ++i){
        robot->r_joint_[i]->m_State.m_rCommand = pDyn_env->cmd_->jtorque_cmd[i] + 
            Kp * (pDyn_env->cmd_->jpos_cmd[i] - p_data->jpos[i]) + 
            Kd * (pDyn_env->cmd_->jvel_cmd[i] - p_data->jvel[i]);
        
        //robot->r_joint_[i]->m_State.m_rCommand *= ramp;
    }
}

void Valkyrie_Dyn_environment::Rendering_Fnc()
{
}


void Valkyrie_Dyn_environment::_DrawHollowCircle(GLfloat x, GLfloat y, GLfloat z, GLfloat radius){
    int i;
    int lineAmount = 200; //# of triangles used to draw circle

    //GLfloat radius = 0.8f; //radius
    GLfloat twicePi = 2.0f * M_PI;

    for (int i(0); i< lineAmount-1; ++i){
        glLineWidth(2.0);
        glBegin(GL_LINES);
        glVertex3f(x + radius * cos(i * twicePi / lineAmount),
                y + radius * sin(i * twicePi / lineAmount), z);
        glVertex3f(x + radius * cos((i+1) * twicePi / lineAmount),
                y + radius * sin((i+1) * twicePi / lineAmount), z);
        glEnd();
    }
}

void Valkyrie_Dyn_environment::_Copy_Array(double * subject, double * data, int num_element){
    // for(int i(0); i< num_element; ++i){
    //     subject[i] = data[i];
    // }
}

Valkyrie_Dyn_environment::~Valkyrie_Dyn_environment()
{
    SR_SAFE_DELETE(interface_);
    SR_SAFE_DELETE(robot_);
    SR_SAFE_DELETE(m_Space);
    SR_SAFE_DELETE(m_ground);
}

void Valkyrie_Dyn_environment::_CheckFootContact(bool & r_contact, bool & l_contact){
    Vec3 lfoot_pos = robot_->
        link_[robot_->link_idx_map_.find("leftCOP_Frame")->second]->GetPosition();
    Vec3 rfoot_pos = robot_->
        link_[robot_->link_idx_map_.find("rightCOP_Frame")->second]->GetPosition();

    //std::cout<<rfoot_pos<<std::endl;
    //std::cout<<lfoot_pos<<std::endl;

    if(  fabs(lfoot_pos[2]) < 0.015){
        l_contact = true;
        //printf("left contact\n");
    }else { l_contact = false; }
    if (fabs(rfoot_pos[2])<0.015  ){
        r_contact = true;
        //printf("right contact\n");
    } else { r_contact = false; }

    //printf("\n");
}
