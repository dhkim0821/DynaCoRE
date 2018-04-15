#include "Mercury_interface.hpp"
#include <stdio.h>
#include <math.h>
#include <string>
#include <Utils/utilities.hpp>
#include <Utils/DataManager.hpp>
#include <Utils/wrap_eigen.hpp>
#include "Mercury_StateProvider.hpp"
#include "Mercury_StateEstimator.hpp"
#include <ParamHandler/ParamHandler.hpp>
#include <Mercury/Mercury_Model.hpp>

// TEST SET LIST
#include <Mercury_Controller/TestSet/BodyCtrlTest.hpp>
#include <Mercury_Controller/TestSet/JointCtrlTest.hpp>
#include <Mercury_Controller/TestSet/WalkingTest.hpp>
#include <Mercury_Controller/TestSet/FootCtrlTest.hpp>

#if MEASURE_TIME
#include <chrono>
#endif

Mercury_interface::Mercury_interface():
    interface(),
    torque_command_(mercury::num_act_joint),
    test_command_(mercury::num_act_joint),
    sensed_torque_(mercury::num_act_joint),
    torque_limit_max_(mercury::num_act_joint),
    torque_limit_min_(mercury::num_act_joint),
    motor_current_(mercury::num_act_joint)
{
    robot_sys_ = new Mercury_Model();
    sensed_torque_.setZero();
    torque_command_.setZero();
    motor_current_.setZero();
    test_command_.setZero();

    sp_ = Mercury_StateProvider::getStateProvider();
    state_estimator_ = new Mercury_StateEstimator(robot_sys_);  
    DataManager::GetDataManager()->RegisterData(&running_time_, DOUBLE, "running_time");
    DataManager::GetDataManager()->RegisterData(&sensed_torque_, DYN_VEC, "torque", mercury::num_act_joint);
    DataManager::GetDataManager()->RegisterData(&torque_command_, DYN_VEC, "command", mercury::num_act_joint);
    DataManager::GetDataManager()->RegisterData(&motor_current_, DYN_VEC, "motor_current", mercury::num_act_joint);

    _ParameterSetting();
    printf("[Mercury_interface] Contruct\n");
}

Mercury_interface::~Mercury_interface(){
    delete test_;
}

void Mercury_interface::GetCommand( void* _data,
        std::vector<double> & command){
    
    command.resize(mercury::num_act_joint*2, 0.);
    Mercury_SensorData* data = ((Mercury_SensorData*)_data);
    
    if(!_Initialization(data)){
#if MEASURE_TIME
        std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
#endif
        state_estimator_->Update(data);
        // Calcualate Torque
        test_->getTorqueInput(test_command_);

#if MEASURE_TIME
        std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> time_span1 = std::chrono::duration_cast< std::chrono::duration<double> >(t2 - t1);
        if(count_%1000 == 1){
            std::cout << "[Mercury_interface] All process took me " << time_span1.count()*1000.0 << "ms."<<std::endl;
        }
#endif
    }

    /// Begin of Torque Limit
    static bool isTurnoff(false);
    for (int i(0); i<mercury::num_act_joint; ++i){
        if(test_command_[i] > torque_limit_max_[i]){
            //if (count_ % 100 == 0) {
            //printf("%i th torque is too large: %f\n", i, torque_command_[i]);
            //}
            torque_command_[i] = torque_limit_max_[i];
            isTurnoff = true;
            //exit(0);
        }else if(test_command_[i]< torque_limit_min_[i]){
            //if (count_ % 100 == 0) {
            //printf("%i th torque is too small: %f\n", i, torque_command_[i]);
            //}
            torque_command_[i] = torque_limit_min_[i];
            isTurnoff = true;
            //exit(0);
        } else{
            torque_command_[i] = test_command_[i];
        }
        command[i] = torque_command_[i];
        sensed_torque_[i] = data->jtorque[i];
        motor_current_[i] = data->motor_current[i];
    }
    if (isTurnoff) {
        //torque_command_.setZero();
        for(int i(0); i<mercury::num_act_joint; ++i){
            command[i] = 0.;
        }
        isTurnoff = false;
    }
    /// End of Torque Limit Check


    // IF torque feedforward control is computed
    if( test_command_.rows() == 2* mercury::num_act_joint ){
        /// Begin of Torque Limit
        int k(0);
        for (int i(mercury::num_act_joint); i<mercury::num_act_joint*2; ++i){
            k = i - mercury::num_act_joint;
            if(test_command_[i] > torque_limit_max_[i]){
                torque_command_[i] = torque_limit_max_[i];
                isTurnoff = true;
            }else if(test_command_[i]< torque_limit_min_[i]){
                torque_command_[i] = torque_limit_min_[i];
                isTurnoff = true;
            } else{
                torque_command_[i] = test_command_[i];
            }
            command[i] = torque_command_[i];
            sensed_torque_[i] = data->jtorque[i];
            motor_current_[i] = data->motor_current[i];
        }
        if (isTurnoff) {
            //torque_command_.setZero();
            for(int i(mercury::num_act_joint); i<2*mercury::num_act_joint; ++i){
                command[i] = 0.;
            }
            isTurnoff = false;
        }
        /// End of Torque Limit Check
    }

    running_time_ = (double)(count_) * mercury::servo_rate;
    ++count_;
    // When there is sensed time
    sp_->curr_time_ = running_time_;

    //if(count_%500== 1){
    //dynacore::pretty_print(data->imu_ang_vel, "imu ang vel", 3);
    //dynacore::pretty_print(data->imu_acc, "imu acc", 3);
    //dynacore::Quaternion ori = sp_->body_ori_;
    //dynacore::pretty_print(ori, std::cout, "estimated_quat");

    //double yaw, pitch, roll;
    //dynacore::convert(ori, yaw, pitch, roll);
    //printf("rpy: %f, %f, %f\n", roll, pitch, yaw);
    //printf("\n");
    //}

     //if(count_%100== 1){
       //dynacore::pretty_print(global_ori_, std::cout, "sim global quat");
       //dynacore::pretty_print(sp_->body_ori_, std::cout, "estimated_quat");
       //printf("\n");
     //}
}
void Mercury_interface::GetReactionForce(std::vector<dynacore::Vect3> & reaction_force ){
    reaction_force.resize(2);
    for(int i(0); i<2; ++i){
        for(int j(0); j<3; ++j){
            reaction_force[i][j] = sp_->reaction_forces_[j];
        }
    }
}

bool Mercury_interface::_Initialization(Mercury_SensorData* data){
    if(count_ < 10){
    // TEST
    //if(count_ < 50000000){
        torque_command_.setZero();
        state_estimator_->Initialization(data);
        test_->TestInitialization();

        return true;
    }
    DataManager::GetDataManager()->start();
    return false;
}

void Mercury_interface::_ParameterSetting(){
    ParamHandler handler(MercuryConfigPath"INTERFACE_setup.yaml");

    std::string tmp_string;
    bool b_tmp;
    // TEST SETUP
    handler.getString("test_name", tmp_string);
    if(tmp_string == "joint_ctrl_test"){
        test_ = new JointCtrlTest(robot_sys_);
    }else if(tmp_string == "walking_test"){
        test_ = new WalkingTest(robot_sys_);
    }else if(tmp_string == "body_ctrl_test"){
        test_ = new BodyCtrlTest(robot_sys_);
    }else if(tmp_string == "foot_ctrl_test"){
        test_ = new FootCtrlTest(robot_sys_);
    }else {
        printf("[Interfacce] There is no test matching with the name\n");
        exit(0);
    }
    // State Estimator Setup
    handler.getBoolean("is_floating_base", b_tmp);
    state_estimator_->setFloatingBase(b_tmp);

    // Torque limit
    handler.getVector("torque_max", torque_limit_max_);
    handler.getVector("torque_min", torque_limit_min_);

    printf("[Mercury_interface] Parameter setup is done\n");
}
