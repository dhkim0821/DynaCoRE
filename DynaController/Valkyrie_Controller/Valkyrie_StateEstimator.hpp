#ifndef STATE_ESTIMATOR_VALKYRIE
#define STATE_ESTIMATOR_VALKYRIE

#include <Configuration.h>
#include <Utils/wrap_eigen.hpp>


class Valkyrie_StateProvider;
class RobotSystem;
class OriEstimator;
class Valkyrie_SensorData;

class Valkyrie_StateEstimator{
    public:
        Valkyrie_StateEstimator(RobotSystem* robot);
        ~Valkyrie_StateEstimator();

        void Initialization(Valkyrie_SensorData* );
        void Update(Valkyrie_SensorData* );

    protected:
        double initial_height_;
        int fixed_foot_;
        dynacore::Vect3 foot_pos_;
        Valkyrie_StateProvider* sp_;
        RobotSystem* robot_sys_;

        dynacore::Vector curr_config_;
        dynacore::Vector curr_qdot_;

        OriEstimator* ori_est_;
};

#endif
