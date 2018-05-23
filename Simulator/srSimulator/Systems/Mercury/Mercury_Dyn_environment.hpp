#ifndef		MERCURY_DYN_ENVIRONMENT
#define		MERCURY_DYN_ENVIRONMENT

#include "Mercury.hpp"
#include "srDyn/srSystem.h"
#include "srDyn/srCollision.h"
#include "Ground.h"
#include <vector>
#include "Configuration.h"
#include <DynaController/Mercury_Controller/Mercury_interface.hpp>
#include <DynaController/Mercury_Controller/Mercury_StateProvider.hpp>

class interface;

class Mercury_Dyn_environment
{
public:
  Mercury_Dyn_environment();
  ~Mercury_Dyn_environment();

  static void ContolFunction(void* _data);

  int count_;

  void Rendering_Fnc();
  void FixXY();
  void PassiveAnkleSpring();
  void PushRobotBody();
  
  interface* interface_;

  Mercury_StateProvider* sp_;

  Mercury_SensorData* data_;
  Mercury_Command* cmd_;
  Mercury*	m_Mercury;
  srSpace*	m_Space;
  Ground*	m_ground;

  void getIMU_Data(std::vector<double> & imu_acc,
                   std::vector<double> & imu_ang_vel);

  void getFootContact_Data(bool & left_foot_contact,
                           bool & right_foot_contact);  

private:

  std::vector<double> push_time_;
  std::vector<double> push_force_;
  std::vector<double> push_direction_;
  
  void _ParamterSetup();
  int num_substep_rendering_;
  double release_time_;

  std::vector<double> imu_ang_vel_bias_;
  std::vector<double> imu_ang_vel_var_;
};

#endif
