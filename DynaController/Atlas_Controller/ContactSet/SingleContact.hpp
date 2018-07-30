#ifndef ATLAS_SINGLE_CONTACT
#define ATLAS_SINGLE_CONTACT

#include <WBDC/WBDC_ContactSpec.hpp>
class RobotSystem;
class Atlas_StateProvider;

class SingleContact: public WBDC_ContactSpec{
public:
  SingleContact(const RobotSystem* robot, int contact_pt);
  virtual ~SingleContact();

protected:
  virtual bool _UpdateJc();
  virtual bool _UpdateJcDotQdot();
  virtual bool _UpdateUf();
  virtual bool _UpdateInequalityVector();

  void _setU(double x, double y, double mu, dynacore::Matrix & U);
  const RobotSystem* robot_sys_;
  Atlas_StateProvider* sp_;

  int contact_pt_;
};

#endif
