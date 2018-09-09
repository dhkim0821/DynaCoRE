#ifndef DRACO_BIPED_INTERFACE_H
#define DRACO_BIPED_INTERFACE_H

#include <Utils/wrap_eigen.hpp>
#include <Configuration.h>
#include <interface.hpp>
#include "DracoBip_DynaCtrl_Definition.h"
#include <Filter/filters.hpp>

class DracoBip_StateEstimator;
class DracoBip_StateProvider;

class DracoBip_interface: public interface{
public:
  DracoBip_interface();
  virtual ~DracoBip_interface();
  virtual void GetCommand(void * data, void * command);

private:
  int waiting_count_;
  
  void _ParameterSetting();
  bool _Initialization(DracoBip_SensorData* );

  DracoBip_Command* test_cmd_;

  dynacore::Vector torque_;
  dynacore::Vector jjvel_;
  dynacore::Vector jjpos_;
 
  dynacore::Vector torque_command_;
  dynacore::Vector jpos_command_;
  dynacore::Vector jvel_command_;
  
  DracoBip_StateEstimator* state_estimator_;
  DracoBip_StateProvider* sp_;
};

#endif
