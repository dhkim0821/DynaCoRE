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
	prev_lf_contact(false),
	prev_rf_contact(false),
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

	count = 0;
	// Initialize EKF Variables
	local_gravity = 9.81; 
	gravity_vec	= dynacore::Vector::Zero(3);
	gravity_vec[2] = local_gravity;

	dt = mercury::servo_rate;
	dim_states = O_r.size() + O_v.size() + 4 + O_p_l.size() + O_p_r.size() + B_bf.size() + B_bw.size();
	dim_rvq_states = O_r.size() + O_v.size() + 4;


	dim_error_states = O_r.size() + O_v.size() + 3 + O_p_l.size() + O_p_r.size() + B_bf.size() + B_bw.size();
	dim_process_errors = f_imu.size() + omega_imu.size() + O_p_l.size() + O_p_r.size() + B_bf.size() + B_bw.size();
	dim_obs = O_p_l.size() + O_p_r.size();
	dim_inputs = f_imu.size() + omega_imu.size();



	C_rot = dynacore::Matrix::Identity(3,3);
	F_c = dynacore::Matrix::Zero(dim_error_states, dim_error_states);
	L_c = dynacore::Matrix::Zero(dim_error_states, dim_process_errors);	
	Q_c = dynacore::Matrix::Zero(dim_process_errors,dim_process_errors);

	F_k = dynacore::Matrix::Zero(dim_error_states, dim_error_states);
	H_k = dynacore::Matrix::Zero(dim_obs, dim_error_states);

	R_c = dynacore::Matrix::Zero(dim_obs, dim_obs);

	P_prior = dynacore::Matrix::Zero(dim_error_states, dim_error_states);
	P_predicted = dynacore::Matrix::Zero(dim_error_states, dim_error_states);	
	P_posterior = dynacore::Matrix::Zero(dim_error_states, dim_error_states);	

	x_prior = dynacore::Vector::Zero(dim_states);
	x_predicted = dynacore::Vector::Zero(dim_states);
	x_posterior = dynacore::Vector::Zero(dim_states);	
	x_prior[9] = 1.0; // Set Quaternion to identity.

	delta_x_prior = dynacore::Vector::Zero(dim_error_states);
	delta_x_posterior = dynacore::Vector::Zero(dim_error_states);	
	delta_y = dynacore::Vector(dim_obs);

	// Initialize Covariance parameters
	// Values are from reference paper. Need to be changed to known IMU parameters
	wf_intensity = 0.00078;   // m/(s^2)/sqrt(Hz) // imu process noise intensity
	ww_intensity = 0.000523;  // rad/s/sqrt(Hz) // angular velocity process noise intensity

	wp_intensity_default = 0.001;   // m/sqrt(Hz)	 // default foot location noise intensity
	wp_intensity_unknown = 1000.0;   // m/sqrt(Hz)	 // noise intensity when there is no foot contact
	wp_l_intensity = wp_intensity_default;   // m/sqrt(Hz)	 // left foot location noise intensity
	wp_r_intensity = wp_intensity_default;   // m/sqrt(Hz)   // right foot location noise intensity

	wbf_intensity = 0.0001;	  // m/(s^3)/sqrt(Hz)  // imu bias intensity
	wbw_intensity = 0.000618; // rad/(s^2)/sqrt(Hz)	 // ang vel bias intensity

	n_p = 0.01; // foot measurement noise intensity.


}

EKF_RotellaEstimator::~EKF_RotellaEstimator(){
	delete robot_model;
}

void EKF_RotellaEstimator::EstimatorInitialization(const dynacore::Quaternion & initial_global_orientation,
                                       const std::vector<double> & initial_imu_acc,
                                       const std::vector<double> & initial_imu_ang_vel){
	// Initialize Global Orientation
	O_q_B = initial_global_orientation;
	// Initialize IMU values
	for(size_t i = 0; i < 3; i++){
		f_imu[i] = initial_imu_acc[i];
		omega_imu[i] = initial_imu_ang_vel[i];		
	}

	// printf("[EKF Rotella Estimator]\n");
	// dynacore::pretty_print(O_q_B, std::cout, "initial global orientation");
	// dynacore::pretty_print(f_imu, std::cout, "initial f_imu");
	// dynacore::pretty_print(omega_imu, std::cout, "initial angular velocity");		

}


void EKF_RotellaEstimator::showPrintOutStatements(){
	// printf("[EKF Rotella Estimator]\n");
	dynacore::pretty_print(f_imu, std::cout, "body frame f_imu");
	dynacore::pretty_print(omega_imu, std::cout, "body frame angular velocity");			

	dynacore::pretty_print(f_imu_input, std::cout, "f_imu_input");
	dynacore::pretty_print(omega_imu_input, std::cout, "omega_imu_input");

	dynacore::pretty_print(x_predicted, std::cout, "x_predicted");


	// printf("Left Foot contact = %d \n", lf_contact);
	// printf("Right Foot contact = %d \n", rf_contact);	
	// dynacore::pretty_print(Q_config, std::cout, "Q_config");			
}

void EKF_RotellaEstimator::computeNewFootLocations(const int foot_link_id){
    // Find Foot body id
    unsigned int bodyid;
    switch(foot_link_id){
        case mercury_link::leftFoot:
            bodyid = robot_model->GetBodyId("lfoot");
            break;
        case mercury_link::rightFoot:
            bodyid = robot_model->GetBodyId("rfoot");
            break;
        default:
            printf("[EKF_RotellaEstimator] Not a valid foot link id\n");
            exit(0);
    }

    // Set parameters up
    dynacore::Vect3 Local_CoM = robot_model->mFixedBodies[
        bodyid - robot_model->fixed_body_discriminator].mCenterOfMass;

	Q_config.head(O_r.size()) = O_r;
	Q_config[O_r.size()] = O_q_B.x();
	Q_config[O_r.size()+1] = O_q_B.y();		
	Q_config[O_r.size()+2] = O_q_B.z();
	Q_config[mercury::num_qdot] = O_q_B.w();	

	// Update new foot locations
	if (foot_link_id == mercury_link::leftFoot){
	    O_p_l = CalcBodyToBaseCoordinates(*robot_model, Q_config, bodyid, Local_CoM, true);		
		x_prior.segment(dim_rvq_states, O_p_l.size()) = O_p_l;	    
	}
	if (foot_link_id == mercury_link::rightFoot){
	    O_p_r = CalcBodyToBaseCoordinates(*robot_model, Q_config, bodyid, Local_CoM, true);		
		x_prior.segment(dim_rvq_states + O_p_l.size(), O_p_r.size()) = O_p_r;
	}	


}


void EKF_RotellaEstimator::setSensorData(const std::vector<double> & acc,
                             const std::vector<double> & acc_inc,
                             const std::vector<double> & ang_vel,
                             const bool left_foot_contact,
                             const bool right_foot_contact,
                             dynacore::Vector joint_values){

	for(size_t i = 0; i < 3; i++){
		f_imu[i] = -acc[i];
		omega_imu[i] = ang_vel[i];		
	}	
	lf_contact = left_foot_contact;
	rf_contact = right_foot_contact;	
	Q_config.segment(mercury::num_virtual, mercury::num_act_joint) = joint_values;

	// Note: the first time the foot contact is activated, the estimator should start running.

	// Handle changes in foot contact
	handleFootContacts();	

	// Perform filter calculations given sensor data
	doFilterCalculations();

	// Print Statements
	// if (count % 100 == 0){
	// 	showPrintOutStatements();
	// }
	// count++;

	// Update foot contact booleans
	prev_lf_contact = lf_contact;
	prev_rf_contact = rf_contact;	

}

void EKF_RotellaEstimator::handleFootContacts(){
	// if the foot location is lost, set wp to a high number
	if (lf_contact){
		wp_l_intensity = wp_intensity_default;
	}else{
		wp_l_intensity = wp_intensity_unknown;
	}
	// if the foot location is lost, set wp to a high number
	if (rf_contact){
		wp_r_intensity = wp_intensity_default;
	}else{
		wp_r_intensity = wp_intensity_unknown;
	}	

	// Check if a new foot location will be used for estimation
	if ((prev_lf_contact == false) && (lf_contact == true)){
		computeNewFootLocations(mercury_link::leftFoot); // Update Left foot location
		//printf("\n New Left foot contact\n");
	}
	if ((prev_rf_contact == false) && (rf_contact == true)){
		computeNewFootLocations(mercury_link::rightFoot); // Update right foot location
		//printf("\n New Right foot contact\n");
	}

}

dynacore::Matrix getSkewSymmetricMatrix(dynacore::Vector vec_in){
	dynacore::Matrix ssm = dynacore::Matrix::Zero(3,3);
	ssm(0,1) = -vec_in[2]; ssm(0,2) = vec_in[1];
	ssm(1,0) = vec_in[2];  ssm(1,2) = -vec_in[0];	
	ssm(2,0) = -vec_in[1]; ssm(2,1) = vec_in[0];
	return ssm;
}

void EKF_RotellaEstimator::setStateVariablesToPrior(){
	// Get prior values:
	O_r = x_prior.head(O_r.size());
	O_v = x_prior.segment(O_r.size(), O_v.size());
	O_q_B.x() = x_prior[O_r.size() + O_v.size()];
	O_q_B.y() = x_prior[O_r.size() + O_v.size() + 1];	
	O_q_B.z() = x_prior[O_r.size() + O_v.size() + 2];		
	O_q_B.w() = x_prior[O_r.size() + O_v.size() + 3];

	C_rot = O_q_B.toRotationMatrix();

	O_p_l = x_prior.segment(dim_rvq_states, O_p_l.size());
	O_p_r = x_prior.segment(dim_rvq_states + O_p_l.size(), O_p_r.size());	
	B_bf = x_prior.segment(dim_rvq_states + O_p_l.size() + O_p_r.size(), B_bf.size());
	B_bw = x_prior.segment(dim_rvq_states + O_p_l.size() + O_p_r.size() + B_bw.size(), B_bw.size());		
}

void EKF_RotellaEstimator::predictionStep(){
	// Prepare state variables
	setStateVariablesToPrior();

	// Prepare filter input:
	f_imu_input = f_imu - B_bf;
	omega_imu_input = omega_imu - B_bw;

	// Prepare rotation values
	dynacore::Quaternion B_q_omega;
	dynacore::Vect3 vec3_omega_input; vec3_omega_input.setZero();
	for(size_t i = 0; i < 3; i++){ 
		vec3_omega_input[i] = omega_imu_input[i];
	}
	dynacore::convert(vec3_omega_input*dt, B_q_omega);

	// Perform Discrete State prediction step;
	// predict position and velocity
	x_predicted.segment(0, O_r.size()) = O_r + O_v*dt + 0.5*dt*dt*(C_rot.transpose()*f_imu_input + gravity_vec);
	x_predicted.segment(O_r.size(), O_v.size()) = O_v + dt*(C_rot.transpose()*f_imu_input + gravity_vec);
	// predict orientation
	dynacore::Quaternion q_predicted = 	dynacore::QuatMultiply(O_q_B, B_q_omega);
	x_predicted[O_r.size()+O_v.size()] = q_predicted.x();
	x_predicted[O_r.size()+O_v.size()+1] = q_predicted.y();
	x_predicted[O_r.size()+O_v.size()+2] = q_predicted.z();		
	x_predicted[O_r.size()+O_v.size()+3] = q_predicted.w();			
	
	// predict foot position and biases
	x_predicted.segment(dim_rvq_states, dim_states - dim_rvq_states) = x_prior.segment(dim_rvq_states, dim_states - dim_rvq_states); 
}

void EKF_RotellaEstimator::updateStep(){
	// Update Prior
	x_prior = x_predicted;
}


void EKF_RotellaEstimator::doFilterCalculations(){
	predictionStep();
	updateStep();
}