#include "EKF_RotellaEstimator.hpp"
#include <Configuration.h>
#include <Utils/utilities.hpp>
#include <Mercury/Mercury_Definition.h>
#include <Utils/DataManager.hpp>

#include <rbdl/urdfreader.h>

using namespace RigidBodyDynamics;

EKF_RotellaEstimator::EKF_RotellaEstimator():Q_config(mercury::num_q),
	O_p_l(3),
	O_p_r(3),
	B_bf(3),
	B_bw(3),
	lf_contact(false),
	rf_contact(false),
	f_imu(3),
	omega_imu(3)	
{
	// Load Robot Model
	robot_model = new Model();
	rbdl_check_api_version (RBDL_API_VERSION);
	if (!Addons::URDFReadFromFile (THIS_COM"/RobotSystems/Mercury/mercury.urdf", robot_model, false)) {
		std::cerr << "Error loading model ./mercury.urdf" << std::endl;
		abort();
	}
	// --------------------
	printf("[EKF Rotella Estimator] Successfully loaded URDF robot model.");

	// Initialize Global Positions
	O_r = dynacore::Vector::Zero(3);
	O_v = dynacore::Vector::Zero(3);

	// Initialize Orientations
	O_q_B.w() = 1;
	O_q_B.x() = 0.0; 	O_q_B.y() = 0.0; 	O_q_B.z() = 0.0;	
	Q_config[mercury::num_qdot] = 1.0; // Set orientation to identity.	

	// Initialize remaining state vectors
	O_p_l.setZero();
	O_p_r.setZero();
	B_bf.setZero();
	B_bw.setZero();

	// Initialize inputs
	f_imu.setZero();
	omega_imu.setZero();

	// Initialize EKF Variables
	dim_inputs = f_imu.size() + omega_imu.size();

}

EKF_RotellaEstimator::~EKF_RotellaEstimator(){
	delete robot_model;
}

void EKF_RotellaEstimator::EstimatorInitialization(const dynacore::Quaternion & initial_global_orientation,
                                       const std::vector<double> & initial_imu_acc,
                                       const std::vector<double> & initial_imu_ang_vel){

	printf("\n");

	O_q_B = initial_global_orientation;
	for(size_t i = 0; i < 3; i++){
		f_imu[i] = initial_imu_acc[i];
		omega_imu[i] = initial_imu_ang_vel[i];		
	}

	printf("[EKF Rotella Estimator]\n");
	dynacore::pretty_print(O_q_B, std::cout, "initial global orientation");
	dynacore::pretty_print(f_imu, std::cout, "initial f_imu");
	dynacore::pretty_print(omega_imu, std::cout, "initial angular velocity");		

}


void EKF_RotellaEstimator::setSensorData(const std::vector<double> & acc,
                             const std::vector<double> & acc_inc,
                             const std::vector<double> & ang_vel,
                             const bool left_foot_contact,
                             const bool right_foot_contact,
                             dynacore::Vector joint_values){

	for(size_t i = 0; i < 3; i++){
		f_imu[i] = acc[i];
		omega_imu[i] = ang_vel[i];		
	}	
	lf_contact = left_foot_contact;
	rf_contact = right_foot_contact;	
	Q_config.segment(mercury::num_virtual, mercury::num_act_joint) = joint_values;


	// printf("[EKF Rotella Estimator]\n");
	// dynacore::pretty_print(f_imu, std::cout, "body frame f_imu");
	// dynacore::pretty_print(omega_imu, std::cout, "body frame angular velocity");			
	// printf("Left Foot contact = %d \n", lf_contact);
	// printf("Right Foot contact = %d \n", rf_contact);	
	// dynacore::pretty_print(Q_config, std::cout, "Q_config");			

}