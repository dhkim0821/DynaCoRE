#ifndef ANGULAR_VELOCITY_ACCUMULATION_VALKYRIE
#define ANGULAR_VELOCITY_ACCUMULATION_VALKYRIE

#include "OriEstimator.hpp"
#include <Filter/filters.hpp>

class BasicAccumulation:public OriEstimator{
public:
  BasicAccumulation();
  virtual ~BasicAccumulation();

  virtual void EstimatorInitialization(const dynacore::Quaternion & ini_quat,
                                       const std::vector<double> & acc,
                                       const std::vector<double> & ang_vel);

  virtual void setSensorData(const std::vector<double> & acc,
                             const std::vector<double> & ang_vel);
protected:
};

#endif
