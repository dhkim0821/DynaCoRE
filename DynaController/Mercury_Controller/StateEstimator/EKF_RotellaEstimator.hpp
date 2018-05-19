#ifndef EKF_ROTELLA_ESTIMATOR
#define EKF_ROTELLA_ESTIMATOR

#include "EKF_PoseEstimator.hpp"

/* This estimator is based on:

Rotella, Nicholas, et al. 
"State estimation for a humanoid robot." 
Intelligent Robots and Systems (IROS 2014), 
2014 IEEE/RSJ International Conference on. IEEE, 2014.

and

Bloesch, Michael, et al. 
"State estimation for legged robots-consistent fusion of leg kinematics and IMU." 
Robotics 17 (2013): 17-24.

*/

class EKF_RotellaEstimator :public EKF_PoseEstimator{
public:
  EKF_RotellaEstimator();	
  ~EKF_RotellaEstimator();
  virtual void setSensorData(const std::vector<double> & acc,
                             const std::vector<double> & acc_inc,
                             const std::vector<double> & ang_vel,
                             dynacore::Vector joint_values);
  virtual void EstimatorInitialization(const dynacore::Quaternion & initial_global_orientation,
                                       const std::vector<double> & initial_imu_acc,
                                       const std::vector<double> & initial_imu_ang_vel);

protected:
  dynacore::Vector O_p_l; // global left foot position
  dynacore::Vector O_p_r; // global right foot position   

  dynacore::Vector B_bf;  // imu frame acceleration bias
  dynacore::Vector B_bw;  // imu frame angular velocity bias  
};


#endif
