#ifndef BODY_FOOT_POSITON_ESTIMATOR
#define BODY_FOOT_POSITON_ESTIMATOR

#include <Utils/wrap_eigen.hpp>

class MoCapManager;
class RobotSystem;

class BodyFootPosEstimator{
public:
  BodyFootPosEstimator(RobotSystem*);
  ~BodyFootPosEstimator();

  void Initialization();
  void getMoCapBodyOri(dynacore::Quaternion & quat);
  void getMoCapBodyVel(dynacore::Vect3 & body_vel);

protected:
  MoCapManager* mocap_manager_;
};

#endif
