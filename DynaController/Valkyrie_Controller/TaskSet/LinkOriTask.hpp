#ifndef VALKYRIE_WBC_LINK_ORIENTATION_TASK
#define VALKYRIE_WBC_LINK_ORIENTATION_TASK

#include <WBC/Task.hpp>

class Valkyrie_StateProvider;
class RobotSystem;

class LinkOriTask: public Task{
public:
  LinkOriTask(RobotSystem*, int );
  virtual ~LinkOriTask();

  dynacore::Vector Kp_vec_;
  dynacore::Vector Kd_vec_;

protected:
  // Update op_cmd_
  virtual bool _UpdateCommand(void* pos_des,
                              const dynacore::Vector & vel_des,
                              const dynacore::Vector & acc_des);
  // Update Jt_
  virtual bool _UpdateTaskJacobian();
  // Update JtDotQdot_
  virtual bool _UpdateTaskJDotQdot();
    // no additional update
  virtual bool _AdditionalUpdate(){ return true;}

  const Valkyrie_StateProvider* sp_;
  const RobotSystem* robot_sys_;
  int bodyID_;
};

#endif
