#ifndef ANGULAR_VELOCITY_ACCUMULATION
#define ANGULAR_VELOCITY_ACCUMULATION

#include "OriEstimator.hpp"

class BasicAccumulation:public OriEstimator{
public:
  BasicAccumulation();
  virtual ~BasicAccumulation();

  virtual void EstimatorInitialization(const dynacore::Quaternion & ini_quat,
                                       const std::vector<double> & acc,
                                       const std::vector<double> & ang_vel);

  virtual void setSensorData(const std::vector<double> & acc,
                             const std::vector<double> & acc_inc,
                             const std::vector<double> & ang_vel);

  void CoMStateInitialization(const dynacore::Vect3 & com_pos, 
          const dynacore::Vect3 & com_vel);
  void getEstimatedCoMState(dynacore::Vector & com_state);
protected:
    dynacore::Vector com_state_;
    dynacore::Vect3 ini_acc_;
};

#endif
