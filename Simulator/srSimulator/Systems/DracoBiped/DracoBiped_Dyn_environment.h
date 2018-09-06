#ifndef  DYN_ENVIRONMENT_DracoBiped
#define  DYN_ENVIRONMENT_DracoBiped

#include "LieGroup/LieGroup.h"
#include <vector>
#include <Utils/wrap_eigen.hpp>

//TEST JUNHYEOK
#include "DracoBiped.h"

//TEST
////////////////////////////////////////////////
#ifdef __APPLE__
#include <GLUT/glut.h>
#endif

//#ifdef linux
#ifdef __linux__
#include <GL/glut.h>
#endif
////////////////////////////////////////////////


class interface;
class srSpace;
class Ground;
class DracoBiped_Command;
class DracoBiped_SensorData;

class DracoBiped_Dyn_environment
{
public:
  DracoBiped_Dyn_environment();
  ~DracoBiped_Dyn_environment();

  static void ControlFunction(void* _data);
  void Rendering_Fnc();

  void SetCurrentState_All();
  void saveLandingLocation();
public:
   DracoBiped_SensorData* data_;
  DracoBiped_Command* cmd_;

 interface* interface_;
  DracoBiped* robot_;

  srSpace*	m_Space;
  Ground*	m_ground;

  double ori_mtx_[9];
  std::vector<double> ang_vel_  ;
protected:
  void _Get_Orientation(dynacore::Quaternion & rot);
  void _Copy_Array(double * , double *, int);
  void _CheckFootContact(bool & r_contact, bool & l_contact);
};

#endif
