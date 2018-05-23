#include "BasicAccumulation.hpp"
#include <Configuration.h>
#include <Utils/utilities.hpp>
#include <Mercury/Mercury_Definition.h>
#include <Utils/DataManager.hpp>


BasicAccumulation::BasicAccumulation():OriEstimator(), com_state_(6){
  global_ori_.w() = 1.;
  global_ori_.x() = 0.;
  com_state_.setZero();

  count = 0;
  calibration_time = 2.5; // Seconds
  reset_once = false;

  // Bias Filter 
  bias_lp_frequency_cutoff = 2.0*3.1415*1.0; // 1Hz // (2*pi*frequency) rads/s 
  x_bias_low_pass_filter = new digital_lp_filter(bias_lp_frequency_cutoff, mercury::servo_rate);
  y_bias_low_pass_filter = new digital_lp_filter(bias_lp_frequency_cutoff, mercury::servo_rate);
  z_bias_low_pass_filter = new digital_lp_filter(bias_lp_frequency_cutoff, mercury::servo_rate);  
  x_acc_bias = 0.0;
  y_acc_bias = 0.0;  
  z_acc_bias = 0.0;

  //
  lp_frequency_cutoff = 2.0*3.1415*100; // 100Hz // (2*pi*frequency) rads/s
  x_acc_low_pass_filter = new digital_lp_filter(lp_frequency_cutoff, mercury::servo_rate);
  y_acc_low_pass_filter = new digital_lp_filter(lp_frequency_cutoff, mercury::servo_rate);
  z_acc_low_pass_filter = new digital_lp_filter(lp_frequency_cutoff, mercury::servo_rate);


  imu_acc.setZero(); // initialize IMU acceleration data
  
  gravity_mag = 9.81; // m/s^2;
  theta_x = 0.0;
  theta_y = 0.0;
  // Initialize body orientation to identity.
  g_B_local_vec.setZero();
  dynacore::Vect3 rpy_init; rpy_init.setZero();
  dynacore::convert(rpy_init, Oq_B); 
  OR_B_init = Oq_B.toRotationMatrix();

  g_o.setZero();// gravity compensation in fixed frame  
  a_o.setZero();// body acceleration in fixed frame 
  v_o; v_o.setZero();// body velocity in fixed frame
  r_o; r_o.setZero();// body position in fixed frame  

  DataManager::GetDataManager()->RegisterData(&a_o, VECT3, "est_body_acc", 3);
  DataManager::GetDataManager()->RegisterData(&v_o, VECT3, "est_body_vel", 3);
  DataManager::GetDataManager()->RegisterData(&r_o, VECT3, "est_body_pos", 3);


}
BasicAccumulation::~BasicAccumulation(){}

void BasicAccumulation::CoMStateInitialization(
        const dynacore::Vect3 & com_pos, 
        const dynacore::Vect3 & com_vel){

    // com_state_[0] = com_pos[0];
    // com_state_[1] = com_pos[1];
    // com_state_[2] = com_vel[0];
    // com_state_[3] = com_vel[1];
}

void BasicAccumulation::getEstimatedCoMState(dynacore::Vector & com_state){
    com_state = com_state_;
}


void BasicAccumulation::EstimatorInitialization(const dynacore::Quaternion & ini_quat,
                                                const std::vector<double> & acc,
                                                const std::vector<double> & ang_vel){
  global_ori_.w() = 1.;
  global_ori_.x() = 0.;
  global_ori_.y() = 0.;
  global_ori_.z() = 0.;

  for(int i(0); i<3; ++i){
      global_ang_vel_[i] = ang_vel[i];
    ini_acc_[i] = acc[i];
    }

  // Update bias estimate
  x_bias_low_pass_filter->input(acc[0]);
  y_bias_low_pass_filter->input(acc[1]);
  z_bias_low_pass_filter->input(acc[2]);

  x_acc_bias = x_bias_low_pass_filter->output();
  y_acc_bias = y_bias_low_pass_filter->output(); 
  z_acc_bias = z_bias_low_pass_filter->output();     

  //printf("Basic Accumulation \n");
  InitIMUOrientationEstimateFromGravity();
  //dynacore::pretty_print(Oq_B_init, std::cout, "Oq_B_init");

  v_o.setZero();
  r_o.setZero();


}

void BasicAccumulation::setSensorData(const std::vector<double> & acc,
                                      const std::vector<double> & acc_inc,
                                      const std::vector<double> & ang_vel){
  // Set IMU acceleration data
  for(size_t i = 0; i < 3; i++){
    imu_acc[i] = acc[i];
  }

  // Orientation
  dynacore::Quaternion delt_quat;
  dynacore::Vect3 delta_th;
  double theta(0.);
  for(int i(0); i<3; ++i){
    delta_th[i] = ang_vel[i] * mercury::servo_rate;
    theta += delta_th[i] * delta_th[i];
  }
  theta = sqrt(theta);  

  if(fabs(theta) > 1.e-20){
    delt_quat.w() = cos(theta/2.);
    delt_quat.x() = sin(theta/2.) * delta_th[0]/theta;
    delt_quat.y() = sin(theta/2.) * delta_th[1]/theta;
    delt_quat.z() = sin(theta/2.) * delta_th[2]/theta;
  } else {
    delt_quat.w() = 1.;
    delt_quat.x() = 0.;
    delt_quat.y() = 0.;
    delt_quat.z() = 0.;
  }

  global_ori_ = dynacore::QuatMultiply(global_ori_, delt_quat);
  static int count(0);
  ++count;
  if(count%500 == 501){
    dynacore::pretty_print(acc, "[estimator] acc");
    dynacore::pretty_print(ang_vel, "[estimator] ang vel");

    dynacore::pretty_print(delt_quat, std::cout, "delta quat");
    dynacore::pretty_print(global_ori_, std::cout, "global ori");
  }
  dynacore::Quaternion ang_quat;
  ang_quat.w() = 0.;
  ang_quat.x() = ang_vel[0];
  ang_quat.y() = ang_vel[1];
  ang_quat.z() = ang_vel[2];

  dynacore::Quaternion quat_dot = dynacore::QuatMultiply(global_ori_, ang_quat, false);
  quat_dot = dynacore::QuatMultiply(quat_dot, global_ori_.inverse(), false);

  global_ang_vel_[0] = quat_dot.x();
  global_ang_vel_[1] = quat_dot.y();
  global_ang_vel_[2] = quat_dot.z();


  dynacore::Quaternion global_acc;
  global_acc.w() = 0.;
  global_acc.x() = acc[0];
  global_acc.y() = acc[1];
  global_acc.z() = acc[2];


  dynacore::Quaternion quat_acc = dynacore::QuatMultiply(global_ori_, global_acc, false);
  quat_acc = dynacore::QuatMultiply(quat_acc, global_ori_.inverse(), false);

    // com_state_[4] = quat_acc.x() - ini_acc_[0]; 
    // com_state_[5] = quat_acc.y() - ini_acc_[1];

    // Reset filters and velocities once after bias calibration time
    // if (((count*mercury::servo_rate) > calibration_time) && (!reset_once)){
    //   // Reset the filters
    //   x_acc_low_pass_filter->clear();
    //   y_acc_low_pass_filter->clear();
    //   z_acc_low_pass_filter->clear();      
    //   reset_once = true;

    //   // Reset local velocities
    //   com_state_[2] = 0.0;
    //   com_state_[3] = 0.0;     

    //   // Reset local positions
    //   com_state_[0] = 0.0;
    //   com_state_[1] = 0.0;      

    //   // Estimate orientation       
    //   InitIMUOrientationEstimateFromGravity();
    //   v_o.setZero();
    //   r_o.setZero();

    // }else{
    //   // Update bias estimate
    //   x_bias_low_pass_filter->input(acc[0]);
    //   y_bias_low_pass_filter->input(acc[1]);
    //   z_bias_low_pass_filter->input(acc[2]);

    //   x_acc_bias = x_bias_low_pass_filter->output();
    //   y_acc_bias = y_bias_low_pass_filter->output(); 
    //   z_acc_bias = z_bias_low_pass_filter->output();           
    // }

    // Get Local Acceleration Estimate
    x_acc_low_pass_filter->input(acc[0] - x_acc_bias);
    y_acc_low_pass_filter->input(acc[1] - y_acc_bias);
    z_acc_low_pass_filter->input(acc[2] - z_acc_bias);    
    // x_acc_low_pass_filter->input(acc[0]);
    // y_acc_low_pass_filter->input(acc[1]);
    // z_acc_low_pass_filter->input(acc[2]);        


    // TEST
    //com_state_[4] = acc[0] - ini_acc_[0];
    com_state_[4] = x_acc_low_pass_filter->output();
    com_state_[5] = y_acc_low_pass_filter->output();

    com_state_[2] = com_state_[2] + com_state_[4]*mercury::servo_rate;
    com_state_[3] = com_state_[3] + com_state_[5]*mercury::servo_rate;

    com_state_[0] = com_state_[0] + com_state_[2]*mercury::servo_rate;
    com_state_[1] = com_state_[1] + com_state_[3]*mercury::servo_rate;


  // Convert body omega into a delta quaternion ------------------------------
  dynacore::Vect3 body_omega; body_omega.setZero();
  for(size_t i = 0; i < 3; i++){
    body_omega[i] = ang_vel[i];
  }
  dynacore::Quaternion delta_quat_body;
  dynacore::convert(body_omega*mercury::servo_rate, delta_quat_body);

  // Perform orientation update via integration
  Oq_B = dynacore::QuatMultiply(Oq_B, delta_quat_body); 

  // Prepare the quantity of the local IMU acceleration
  dynacore::Vect3 f_b; f_b.setZero(); // local IMU acceleration
  f_b.x() = -x_acc_low_pass_filter->output();
  f_b.y() = -y_acc_low_pass_filter->output();
  f_b.z() = -z_acc_low_pass_filter->output();

  // Convert the IMU Acceleration to be in the fixed frame
  dynacore::Vect3 f_o; f_o.setZero();// local IMU acceleration  
  dynacore::Matrix OR_B = Oq_B.normalized().toRotationMatrix();
  f_o = OR_B*f_b;


   // Get the body acceleration in the fixed frame:
  // Initialize vectors
  g_o[2] = gravity_mag;
  // Estimated body Acceleration in fixed frame
  a_o = f_o; //+ g_o;
  v_o = v_o + a_o*mercury::servo_rate;
  r_o = r_o + v_o*mercury::servo_rate; 

  if(count % 100 == 0){
    // printf("Basic Accumulation\n");
    // dynacore::pretty_print(g_B, std::cout, "gravity_dir");
    // printf("    gravity_mag = %0.4f \n", gravity_mag);
    // printf("    theta_x = %0.4f \n", theta_x);    
    // printf("    theta_y = %0.4f \n", theta_y);        
    // printf("    roll_value_comp = %0.4f \n", roll_value_comp);
    // printf("    pitch_value_comp = %0.4f \n", pitch_value_comp);
    // dynacore::pretty_print(g_B_local, std::cout, "rotated gravity_dir");
    // dynacore::pretty_print(g_B_local_vec, std::cout, "g_B_local_vec");    

    // printf("    norm(g_B_local) = %0.4f \n", g_B_local.norm());
    // dynacore::pretty_print(Oq_B_init, std::cout, "Initial body orientation w.r.t fixed frame: ");
    // dynacore::pretty_print(OR_B_init, std::cout, "OR_B_init: ");
    // dynacore::pretty_print(Oq_B, std::cout, "Body orientation w.r.t fixed frame: ");
    // printf("\n");


    // dynacore::pretty_print(body_omega, std::cout, "body_omega");
    // dynacore::pretty_print(delt_quat, std::cout, "delt_quat");
    // dynacore::pretty_print(delta_quat_body, std::cout, "delta_quat_body");

    // dynacore::pretty_print(imu_acc, std::cout, "Data IMU acc = ");

    // dynacore::pretty_print(f_b, std::cout, "IMU acc in body frame f_b = ");
    // dynacore::pretty_print(f_o, std::cout, "IMU acc in fixed frame f_o = ");    
    // dynacore::pretty_print(a_o, std::cout, "body acc in fixed frame a_o = ");    
    // dynacore::pretty_print(v_o, std::cout, "body vel in fixed frame v_o = ");    
    // dynacore::pretty_print(r_o, std::cout, "body pos in fixed frame r_o = ");    
    // printf("\n");    

  }    

    //count++;
}

void BasicAccumulation::InitIMUOrientationEstimateFromGravity(){
  // Finds The orientation of the body with respect to the fixed frame O ((^OR_B ). 
  // This assumes that the IMU can only sense gravity as the acceleration.
  //
  // The algorithm attempts to solve ^OR_B * f_b = f_o, 
  // where f_b is the acceleration of gravity in the body frame.
  // f_o is the acceleration of gravity in the fixed frame
  // In addition to ^OR_B acting as a change of frame formula, 
  // note that ^OR_B will also equal to the orientation of the body w.r.t to the fixed frame.

  // We will rotate f_b using an extrinsic rotation with global R_x (roll) and R_y (pitch) rotations
  // Thus, we will perform ^OR_y ^OR_x f_b = f_o.

  // Finally, note that ^OR_b = ^OR_y ^OR_x.
  // The resulting orientation will have the xhat component of ^OR_b (ie: ^OR_b.col(0)) to be always planar 
  // with the inertial x-z plane. 
  //
  // It is best to visualize the extrinsic rotation on paper for any given extrinsic roll then pitch operations

  g_B.setZero();
  g_B[0] = -x_acc_bias; 
  g_B[1] = -y_acc_bias;
  g_B[2] = -z_acc_bias; // We expect a negative number if gravity is pointing opposite of the IMU zhat direction

  // Test Vector
  // f_b = [-0.057744 -0.001452  -0.998330]  
  // g_B[0] = -0.057744;
  // g_B[1] = -0.001452;
  // g_B[2] = -0.998330;  // We expect a negative number if gravity is pointing opposite of the IMU zhat direction
  // g_B *= 9.7;

  gravity_mag = g_B.norm();
  g_B /= gravity_mag;

  // dynacore::Quaternion q_world_Ry;
  // dynacore::Quaternion q_world_Rx;      

  // Prepare to rotate gravity vector
  g_B_local.w() = 0;
  g_B_local.x() = g_B[0];  g_B_local.y() = g_B[1]; g_B_local.z() = g_B[2];
  g_B_local_vec[0] = g_B[0];   g_B_local_vec[1] = g_B[1];   g_B_local_vec[2] = g_B[2];

  //---------------------------------------------------------------------------
  // Use Rx to rotate the roll and align gravity vector  -
  // Compute Roll to rotate
  // theta_x = atan(g_B[1]/g_B[2]);
  // double roll_val = theta_x;      

  // The following method can handle any initial vector due to gravity
  theta_x = atan2(g_B_local_vec[2], g_B_local_vec[1]); // Returns angle \in [-pi, pi] between z and y projected vectors.
  double roll_val = (-M_PI/2.0 - theta_x);      // (-pi/2 - theta_x)
  roll_value_comp = roll_val;

  //dynacore::convert(0.0, 0.0, roll_val, q_world_Rx);
  // Create Roll Quaternion
  q_world_Rx.w() = cos(roll_val/2.);;
  q_world_Rx.x() = sin(roll_val/2.);
  q_world_Rx.y() = 0;
  q_world_Rx.z() = 0;

  //Rotate gravity vector to align the yhat directions
  dynacore::Matrix Rx = q_world_Rx.normalized().toRotationMatrix();
  g_B_local_vec = Rx*g_B_local_vec;
  // Note that quat multiply sometimes wraps around...
  g_B_local = dynacore::QuatMultiply( dynacore::QuatMultiply(q_world_Rx, g_B_local), q_world_Rx.inverse());


  // Use Ry to rotate pitch and align gravity vector  ---------------------------
  // Compute Pitch to rotate
  // theta_y = atan(g_B_local.x()/g_B_local.z());
  // double pitch_val = -theta_y;

  // The following method can handle any initial vector due to gravity
  theta_y = atan2(g_B_local_vec[2], g_B_local_vec[0]); // Returns angle \in [-pi, pi] between z and x projected vectors.
  double pitch_val = -((-M_PI/2.0) - theta_y);   // This is actually -(-pi/2 - theta_y)
  pitch_value_comp = pitch_val;

  //dynacore::convert(0.0, pitch_val, 0.0, q_world_Ry);
  // Create Pitch Quaternion
  q_world_Ry.w() = cos(pitch_val/2.);
  q_world_Ry.x() = 0;
  q_world_Ry.y() = sin(pitch_val/2.);
  q_world_Ry.z() = 0;  

  // Rotate gravity vector to align the xhat directions
  dynacore::Matrix Ry = q_world_Ry.normalized().toRotationMatrix();
  g_B_local_vec = Ry*g_B_local_vec;  
  // Note that quat multiply sometimes wraps around...   
  g_B_local = dynacore::QuatMultiply( dynacore::QuatMultiply(q_world_Ry, g_B_local), q_world_Ry.inverse());

  // Obtain initial body orientation w.r.t fixed frame.
  //Oq_B = q_y * q_x * q_b
  Oq_B_init = dynacore::QuatMultiply(q_world_Ry, q_world_Rx);
  // Set rotation matrix
  OR_B_init = Oq_B_init.normalized().toRotationMatrix();

  // // // Initialize global orientation
  Oq_B = Oq_B_init;

}
