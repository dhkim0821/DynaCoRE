#ifndef WBDC_COM_BODY_ORIENTATION_TASK
#define WBDC_COM_BODY_ORIENTATION_TASK

#include <WBDC_Relax/WBDC_Relax_Task.hpp>

class Mercury_StateProvider;
class RobotSystem;

class CoMBodyOriTask: public WBDC_Relax_Task{
public:
  CoMBodyOriTask(RobotSystem* ); // X, Y, Z, Rx, Ry, Rz
  virtual ~CoMBodyOriTask();

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

  Mercury_StateProvider* sp_;
  RobotSystem* robot_sys_;
};

#endif
