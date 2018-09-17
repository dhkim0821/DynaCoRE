#include "DracoBip_StateProvider.hpp"
#include <Utils/DataManager.hpp>
#include "DracoBip_DynaCtrl_Definition.h"

DracoBip_StateProvider* DracoBip_StateProvider::getStateProvider(){
    static DracoBip_StateProvider state_provider_;
    return &state_provider_;
}

DracoBip_StateProvider::DracoBip_StateProvider():
                                stance_foot_(dracobip_link::lAnkle),
                                Q_(dracobip::num_q),
                                Qdot_(dracobip::num_qdot),
                                rotor_inertia_(dracobip::num_act_joint),
                                b_rfoot_contact_(0),
                                b_lfoot_contact_(0)
{
    rotor_inertia_.setZero();
    Q_.setZero();
    Qdot_.setZero();
    global_pos_local_.setZero();

    des_location_.setZero();

    DataManager* data_manager = DataManager::GetDataManager();

    data_manager->RegisterData(&curr_time_, DOUBLE, "time");
    data_manager->RegisterData(&Q_, DYN_VEC, "config", dracobip::num_q);
    data_manager->RegisterData(&Qdot_, DYN_VEC, "qdot", dracobip::num_qdot);
    data_manager->RegisterData(&global_pos_local_, VECT3, "global_pos_local", 3);

    data_manager->RegisterData(&b_rfoot_contact_, INT, "rfoot_contact", 1);
    data_manager->RegisterData(&b_lfoot_contact_, INT, "lfoot_contact", 1);
}

