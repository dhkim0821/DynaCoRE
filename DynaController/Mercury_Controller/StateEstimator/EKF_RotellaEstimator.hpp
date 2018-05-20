#ifndef EKF_ROTELLA_ESTIMATOR
#define EKF_ROTELLA_ESTIMATOR

#include "EKF_PoseEstimator.hpp"
#include <rbdl/rbdl.h>

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
                             const bool left_foot_contact,
                             const bool right_foot_contact,
                             dynacore::Vector joint_values);
  virtual void EstimatorInitialization(const dynacore::Quaternion & initial_global_orientation,
                                       const std::vector<double> & initial_imu_acc,
                                       const std::vector<double> & initial_imu_ang_vel);

  dynacore::Matrix getSkewSymmetricMatrix(dynacore::Vector vec_in);


  void handleFootContacts();
  void computeNewFootLocations(const int foot_link_id);
  void setStateVariablesToPrior();

  void showPrintOutStatements();

  void doFilterCalculations();
  void predictionStep();
  void updateStep();

protected:
  dynacore::Vector O_p_l; // global left foot position
  dynacore::Vector O_p_r; // global right foot position   

  dynacore::Vector B_bf;  // imu frame acceleration bias
  dynacore::Vector B_bw;  // imu frame angular velocity bias  

  dynacore::Vector f_imu;  // imu frame acceleration
  dynacore::Vector omega_imu;  // imu frame angular velocity  

  dynacore::Vector f_imu_input;  // imu frame acceleration
  dynacore::Vector omega_imu_input;  // imu frame angular velocity  


  bool lf_contact; // value of left foot contact
  bool rf_contact; // value of right foot contact

  bool prev_lf_contact; // previous value of the left foot contact
  bool prev_rf_contact; // previous value of the right foot contact

  RigidBodyDynamics::Model* robot_model;

  dynacore::Vector Q_config; // configuration of the robot_model

  int count;
  // EKF Variables-----------------------------------
  double local_gravity;
  dynacore::Vector gravity_vec;
  double dt;

  int dim_states;
  int dim_rvq_states;
  int dim_process_errors;
  int dim_error_states;  
  int dim_obs;  
  int dim_inputs;

  dynacore::Matrix C_rot; // rotation matrix from inerital frame to body frame.

  dynacore::Matrix F_c; // contiouous prediction Jacobian. aka: f_x (linearized error state dynamics)
  dynacore::Matrix L_c; // continuous process error Jacobian
  dynacore::Matrix Q_c; // continuous process noise
  double wf_intensity;  // imu process noise intensity
  double ww_intensity;  // angular velocity noise intensity  
  double wp_l_intensity;  // left foot location noise intensity  
  double wp_r_intensity;  // right foot location noise intensity

  double wp_intensity_default; // default foot location noise intensity
  double wp_intensity_unknown; // noise intensity when there is no foot contact

  double wbf_intensity; // imu bias intensity
  double wbw_intensity; // angular velocity bias intensity

  dynacore::Matrix F_k; // discretized error state prediction matrix
  dynacore::Matrix H_k; // discretized error state observation matrix

  dynacore::Matrix R_c; // Measurement noise covariance matrix
  dynacore::Matrix R_k; // Discretized measurement noise covariance matrix  
  double n_p;           // Measurement noise intensity

  dynacore::Matrix P_prior; // Prior covariance matrix
  dynacore::Matrix P_predicted; // Predicted covariance matrix  
  dynacore::Matrix P_posterior; // Posterior covariance matrix

  dynacore::Vector x_prior; // Prior EKF States
  dynacore::Vector x_predicted; // Predicted EKF States  
  dynacore::Vector x_posterior; // Posterior EKF States    

  dynacore::Vector delta_x_prior; // Prior error states
  dynacore::Vector delta_x_posterior; // Posterier error states
  dynacore::Vector delta_y; // prediction and measurement differences

};


#endif
