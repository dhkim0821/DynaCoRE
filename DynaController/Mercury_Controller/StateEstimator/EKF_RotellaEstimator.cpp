#include "EKF_RotellaEstimator.hpp"
#include <Configuration.h>
#include <Utils/utilities.hpp>
#include <Mercury/Mercury_Definition.h>
#include <Utils/DataManager.hpp>

EKF_RotellaEstimator::EKF_RotellaEstimator():O_p_l(3),
	O_p_r(3),
	B_bf(3),
	B_bw(3)	
{
	// Initialize Global Positions
	O_r = dynacore::Vector::Zero(3);
	O_v = dynacore::Vector::Zero(3);

	// Initialize Orientations
	O_q_B.w() = 1;
	O_q_B.x() = 0.0; 	O_q_B.y() = 0.0; 	O_q_B.z() = 0.0;	

	// Initialize remaining state vectors
	O_p_l.setZero();
	O_p_r.setZero();
	B_bf.setZero();
	B_bw.setZero();

}

EKF_RotellaEstimator::~EKF_RotellaEstimator(){}

void EKF_RotellaEstimator::setSensorData(const std::vector<double> & acc,
                             const std::vector<double> & acc_inc,
                             const std::vector<double> & ang_vel,
                             dynacore::Vector joint_values){

}
void EKF_RotellaEstimator::EstimatorInitialization(const dynacore::Quaternion & initial_global_orientation,
                                       const std::vector<double> & initial_imu_acc,
                                       const std::vector<double> & initial_imu_ang_vel){

}