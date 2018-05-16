#ifndef ANGULAR_VELOCITY_ACCUMULATION
#define ANGULAR_VELOCITY_ACCUMULATION

#include "OriEstimator.hpp"

#include <Filter/filters.hpp>

class BasicAccumulation:public OriEstimator{
public:
  BasicAccumulation();
  virtual ~BasicAccumulation();

  virtual void EstimatorInitialization(const dynacore::Quaternion & ini_quat,
                                       const std::vector<double> & acc,
                                       const std::vector<double> & ang_vel);

  virtual void setSensorData(const std::vector<double> & acc,
                             const std::vector<double> & acc_inc,
                             const std::vector<double> & ang_vel);

  void CoMStateInitialization(const dynacore::Vect3 & com_pos, 
          const dynacore::Vect3 & com_vel);
  void getEstimatedCoMState(dynacore::Vector & com_state);

  void InitIMUOrientationEstimateFromGravity();
protected:
    dynacore::Vector com_state_;
    dynacore::Vect3 ini_acc_;


    int count;
    double calibration_time;
    bool reset_once;

    double bias_lp_frequency_cutoff;
    digital_lp_filter* x_bias_low_pass_filter;
    digital_lp_filter* y_bias_low_pass_filter;
    digital_lp_filter* z_bias_low_pass_filter;

    double x_acc_bias;
    double y_acc_bias;
    double z_acc_bias;


    digital_lp_filter* x_acc_low_pass_filter;
    digital_lp_filter* y_acc_low_pass_filter;
    digital_lp_filter* z_acc_low_pass_filter;        
    double lp_frequency_cutoff;

	dynacore::Vect3 g_B; // gravity direction in local frame
	dynacore::Quaternion g_B_local; // rotated gravity direction 
	double gravity_mag; 

	double theta_x;
  double theta_y;  
  dynacore::Quaternion Oq_B; // quaternion of the body frame w.r.t fixed frame
  dynacore::Quaternion Oq_B_init; // initial quaternion of the body frame w.r.t fixed frame
  dynacore::Matrix OR_B_init; // initial Rot matrix of body w.r.t fixed frame

  dynacore::Vect3 a_o; // body acceleration in fixed frame
  dynacore::Vect3 g_o; // gravity compensation in fixed frame  
  dynacore::Vect3 vec_f_o; // IMU acceleration in fixed frame

  dynacore::Vect3 v_o; // body velocity in fixed frame
  dynacore::Vect3 r_o; // position in fixed frame  


};

#endif
