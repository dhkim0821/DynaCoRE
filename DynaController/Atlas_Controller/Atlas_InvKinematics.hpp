#ifndef ATLAS_INVERSE_KINEMATICS
#define ATLAS_INVERSE_KINEMATICS

#include <rbdl/rbdl.h>
#include <Utils/wrap_eigen.hpp>

class RobotSystem;

class Atlas_InvKinematics{
    public:
        Atlas_InvKinematics();
        Atlas_InvKinematics(int swing_foot_id, int stance_foot_id);
        ~Atlas_InvKinematics();
        
       // Get Swing leg configuration from the current posture 
        void getLegConfigAtVerticalPosture(
                const dynacore::Vect3 & target_pos,
                const dynacore::Vector & guess_Q, 
                dynacore::Vector & config_sol);

        // For Double Contact Stand up posture
        void getDoubleSupportLegConfig(
                const dynacore::Vector & current_Q,
                const dynacore::Quaternion & des_quat,
                const double & des_height, dynacore::Vector & config_sol);

        // For Single Support and Swing
        void getSingleSupportStanceLegConfiguration(
                const dynacore::Vector & current_Q,
                const dynacore::Quaternion & des_quat,
                const double & des_height, dynacore::Vector & config_sol);

    protected:
        int max_iter_;
        int body_id_;
        int stance_foot_id_;
        int swing_foot_id_;
        int swing_leg_first_jidx_;

        int pelvis_id_;

        RobotSystem* model_;
};

#endif
