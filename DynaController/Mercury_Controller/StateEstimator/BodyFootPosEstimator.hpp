#ifndef BODY_FOOT_POSITON_ESTIMATOR
#define BODY_FOOT_POSITON_ESTIMATOR

#include <Utils/wrap_eigen.hpp>
#include <Filter/filters.hpp>

class MoCapManager;
class RobotSystem;

class BodyFootPosEstimator{
public:
  BodyFootPosEstimator(RobotSystem*);
  ~BodyFootPosEstimator();

  void Initialization();
    void Update();

  void getMoCapBodyOri(dynacore::Quaternion & quat);
  void getMoCapBodyVel(dynacore::Vect3 & body_vel);

protected:
  MoCapManager* mocap_manager_;
  
  std::vector<filter*> vel_filter_;
  dynacore::Vect3 body_led_vel_;
};

#endif
