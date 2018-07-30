#include "Atlas_interface.hpp"
#include <stdio.h>
#include <math.h>
#include <string>
#include <Utils/utilities.hpp>
#include <Utils/pseudo_inverse.hpp>
#include <Utils/DataManager.hpp>
#include <Utils/wrap_eigen.hpp>
#include "Atlas_StateProvider.hpp"
#include "Atlas_StateEstimator.hpp"
#include <ParamHandler/ParamHandler.hpp>
#include <Atlas/Atlas_Model.hpp>

// Test SET LIST
// Basic Test
#include <Atlas_Controller/TestSet/JointCtrlTest.hpp>

// Walking Test
#include <Atlas_Controller/TestSet/WalkingConfigTest.hpp>

// Body Ctrl Test
#include <Atlas_Controller/TestSet/BodyConfigTest.hpp>

Atlas_interface::Atlas_interface():
    interface(),
    jjvel_(atlas::num_act_joint),
    jjpos_(atlas::num_act_joint),
    initial_upper_body_config_(atlas::num_upper_joint),
    waiting_count_(2)
{

    robot_sys_ = new Atlas_Model();
    jjvel_.setZero();
    jjpos_.setZero();

    test_cmd_ = new Atlas_Command();
    sp_ = Atlas_StateProvider::getStateProvider();
    state_estimator_ = new Atlas_StateEstimator(robot_sys_);  
    
    DataManager::GetDataManager()->RegisterData(
            &running_time_, DOUBLE, "running_time");
    DataManager::GetDataManager()->RegisterData(
            &jjpos_, DYN_VEC, "jjpos", atlas::num_act_joint);
    DataManager::GetDataManager()->RegisterData(
            &jjvel_, DYN_VEC, "jjvel", atlas::num_act_joint);
    
    _ParameterSetting();
    
    printf("[Atlas_interface] Contruct\n");
}

Atlas_interface::~Atlas_interface(){
    delete test_;
}

void Atlas_interface::GetCommand( void* _data, void* _command){
    Atlas_Command* cmd = ((Atlas_Command*)_command);
    Atlas_SensorData* data = ((Atlas_SensorData*)_data);

    if(!_Initialization(data)){
        state_estimator_->Update(data);
        test_->getCommand(test_cmd_);
    }
    
    // Update Command (and Data)
    for(int i(0); i<atlas::num_act_joint; ++i){
        cmd->jtorque_cmd[i] = test_cmd_->jtorque_cmd[i];
        cmd->jpos_cmd[i] = test_cmd_->jpos_cmd[i];
        cmd->jvel_cmd[i] = test_cmd_->jvel_cmd[i];

        jjvel_[i] = data->jvel[i];
        jjpos_[i] = data->jpos[i];
    }

    // Fix Upper Body
    for (int i(0); i<atlas::num_upper_joint; ++i){
        cmd->jpos_cmd[atlas::upper_body_start_jidx - atlas::num_virtual + i] 
            = initial_upper_body_config_[i];
        cmd->jvel_cmd[atlas::upper_body_start_jidx - atlas::num_virtual +  i] = 0.;
    }
    running_time_ = (double)(count_) * atlas::servo_rate;
    ++count_;
    // When there is sensed time
    sp_->curr_time_ = running_time_;
    // Stepping forward
    double walking_start(3.);
    double walking_duration(7.);
    double walking_distance(2.5);
    if(sp_->curr_time_ > walking_start){
        double walking_time = sp_->curr_time_ - walking_start;
        sp_->des_location_[0] = walking_distance * 
            (1-cos(walking_time/walking_duration * M_PI))/2.;
    }
    if(sp_->curr_time_ > walking_start + walking_duration){
        sp_->des_location_[0] = walking_distance;
    }
}

bool Atlas_interface::_Initialization(Atlas_SensorData* data){
    static bool test_initialized(false);
    if(!test_initialized) {
        test_->TestInitialization();
        test_initialized = true;
        printf("[Atlas Interface] Test initialization is done\n");
    }
    if(count_ < waiting_count_){
        for(int i(0); i<atlas::num_act_joint; ++i){
            test_cmd_->jtorque_cmd[i] = 0.;
            test_cmd_->jpos_cmd[i] = data->jpos[i];
            test_cmd_->jvel_cmd[i] = 0.;
        }
        state_estimator_->Initialization(data);

        initial_upper_body_config_ = 
            sp_->Q_.segment(atlas::upper_body_start_jidx, atlas::num_upper_joint);
        DataManager::GetDataManager()->start();
        return true;
    }
    return false;
}

void Atlas_interface::_ParameterSetting(){
    ParamHandler handler(AtlasConfigPath"INTERFACE_setup.yaml");
    std::string tmp_string;
    bool b_tmp;
    // Test SETUP
    handler.getString("test_name", tmp_string);
    // Basic Test ***********************************
    if(tmp_string == "joint_ctrl_test"){
        test_ = new JointCtrlTest(robot_sys_);
        // Walking Test ***********************************
    }else if(tmp_string == "walking_config_test"){
        test_ = new WalkingConfigTest(robot_sys_);
        // Body Ctrl Test ***********************************
    }else if(tmp_string == "body_ctrl_test"){
        test_ = new BodyConfigTest(robot_sys_);    
        // Stance and Swing Test ***********************************
    }else {
        printf("[Interfacce] There is no test matching with the name\n");
        exit(0);
    }
    printf("[Atlas_interface] Parameter setup is done\n");
}
