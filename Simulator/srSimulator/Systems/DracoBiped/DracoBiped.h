#ifndef DracoBiped_H
#define DracoBiped_H

#include <srSysGenerator/SystemGenerator.h>

class DracoBiped: public SystemGenerator {
 public:
  DracoBiped();
  virtual ~DracoBiped();

 private:
  virtual void _SetCollision();
  virtual void _SetInitialConf();
  virtual void _SetJointLimit();

  std::vector<srCollision*> collision_;
};

#endif
