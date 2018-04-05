#include "Valkyrie_interface.hpp"
#include <Valkyrie_Controller/TestSet/JointCtrlTest.hpp>
#include <Valkyrie_Controller/TestSet/MultiTaskTest.hpp>
#include <Valkyrie/Valkyrie_Model.hpp>
#include "Valkyrie_StateProvider.hpp"

Valkyrie_interface::Valkyrie_interface():interface(), gamma_(valkyrie::num_act_joint){
    robot_sys_ = new Valkyrie_Model();
    gamma_.setZero();
    sp_ = Valkyrie_StateProvider::getStateProvider();

    // TEST Selection
    //test_ = new JointCtrlTest(robot_sys_);
     test_ = new MultiTaskTest(robot_sys_);

    printf("[Valkyrie Interface] Contruct\n");
}

Valkyrie_interface::~Valkyrie_interface(){
}

void Valkyrie_interface::GetCommand(void* sensor_data, std::vector<double> & command){
    Valkyrie_SensorData* data = ((Valkyrie_SensorData*) sensor_data);
    for(int i(0); i<valkyrie::num_qdot; ++i) {
        sp_->q_[i] = data->q[i];
        sp_->qdot_[i] = data->qdot[i];
    }
    sp_->q_[valkyrie::num_qdot] = data->q[valkyrie::num_qdot];
    robot_sys_->UpdateSystem(sp_->q_, sp_->qdot_);
   if(!_Initialization(sensor_data)){
        test_->getTorqueInput(gamma_);
    }

   for (int i(0); i<valkyrie::num_act_joint; ++i) command[i] = gamma_[i];

    ++count_;
    running_time_ = count_ * valkyrie::servo_rate;
}

bool Valkyrie_interface::_Initialization(void * sensor_data){

    if(count_ < 1){
        gamma_.setZero();
        return true;
    }
    return false;
}

