#ifndef MERCURY_INVERSE_KINEMATICS
#define MERCURY_INVERSE_KINEMATICS

#include <rbdl/rbdl.h>
#include <Utils/wrap_eigen.hpp>

class Mercury_InvKinematics{
    public:
        Mercury_InvKinematics();
        ~Mercury_InvKinematics();
        
        void getFootPosAtVerticalPosture(
                int link_id, const dynacore::Vect3 & leg_config,
                const dynacore::Vector & guess_Q, dynacore::Vect3 & foot_pos);

   
        void getLegConfigAtVerticalPosture(
                int link_id, const dynacore::Vect3 & target_pos,
                const dynacore::Vector & guess_Q, dynacore::Vector & config_sol);

    protected:
        int max_iter_;

        RigidBodyDynamics::Model* model_;
};

#endif
