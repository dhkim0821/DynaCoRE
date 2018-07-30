#ifndef VALKYRIE_INTERFACE_H
#define VALKYRIE_INTERFACE_H

#include <Utils/wrap_eigen.hpp>
#include <Configuration.h>
#include <interface.hpp>
#include "Valkyrie_DynaControl_Definition.h"
#include <Filter/filters.hpp>

class Valkyrie_StateEstimator;
class Valkyrie_StateProvider;

class Valkyrie_interface: public interface{
public:
  Valkyrie_interface();
  virtual ~Valkyrie_interface();

  virtual void GetCommand(void * data, void * command);

private:
  int waiting_count_;
  
  void _ParameterSetting();
  bool _Initialization(Valkyrie_SensorData* );

  Valkyrie_Command* test_cmd_;
  dynacore::Vector initial_upper_body_config_;

  dynacore::Vector jjvel_;
  dynacore::Vector jjpos_;
  
  Valkyrie_StateEstimator* state_estimator_;
  Valkyrie_StateProvider* sp_;
};

#endif
