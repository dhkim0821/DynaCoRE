#include "BasicAccumulation.hpp"
#include <Configuration.h>
#include <Utils/utilities.hpp>
#include <Mercury/Mercury_Definition.h>

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

  gravity_mag = 9.81; // m/s^2;
  theta_x = 0.0;
  theta_y = 0.0;
  theta_z = 0.0;
  // Initialize body orientation to identity.
  dynacore::Vect3 rpy_init; rpy_init.setZero();
  dynacore::convert(rpy_init, Oq_B); 
  dynacore::convert(rpy_init, Oq_B_init_yaw_rotated); 
  
  OR_B_init = Oq_B.toRotationMatrix();
  OR_B_init_yaw_rotated = Oq_B_init_yaw_rotated.toRotationMatrix();  

  OB_xhat_xy.setZero();

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
}

void BasicAccumulation::setSensorData(const std::vector<double> & acc,
                                      const std::vector<double> & acc_inc,
                                      const std::vector<double> & ang_vel){
  // Orientation
  dynacore::Quaternion delt_quat;
  dynacore::Vect3 delta_th;
  double theta(0.);
  for(int i(0); i<3; ++i){
    delta_th[i] = ang_vel[i] * mercury::servo_rate;
    theta += delta_th[i] * delta_th[i];
  }

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
/*
  dynacore::Quaternion quat_acc = dynacore::QuatMultiply(global_ori_, global_acc, false);
    quat_acc = dynacore::QuatMultiply(quat_acc, global_ori_.inverse(), false);

    // com_state_[4] = quat_acc.x() - ini_acc_[0]; 
    // com_state_[5] = quat_acc.y() - ini_acc_[1];

    // Reset filters and velocities once after bias calibration time
    if (((count*mercury::servo_rate) > calibration_time) && (!reset_once)){
      // Reset the filters
      x_acc_low_pass_filter->clear();
      y_acc_low_pass_filter->clear();
      z_acc_low_pass_filter->clear();      
      reset_once = true;

      // Reset local velocities
      com_state_[2] = 0.0;
      com_state_[3] = 0.0;     

      // Reset local positions
      com_state_[0] = 0.0;
      com_state_[1] = 0.0;      

      // Estimate orientation       
      InitIMUOrientationEstimate();

    }else{
      // Update bias estimate
      x_bias_low_pass_filter->input(acc[0]);
      y_bias_low_pass_filter->input(acc[1]);
      z_bias_low_pass_filter->input(acc[2]);

      x_acc_bias = x_bias_low_pass_filter->output();
      y_acc_bias = y_bias_low_pass_filter->output(); 
      z_acc_bias = z_bias_low_pass_filter->output();           
    }

    // Get Local Acceleration Estimate
    x_acc_low_pass_filter->input(acc[0] - x_acc_bias);
    y_acc_low_pass_filter->input(acc[1] - y_acc_bias);
    z_acc_low_pass_filter->input(acc[2] - z_acc_bias);    


    // TEST
    //com_state_[4] = acc[0] - ini_acc_[0];
    com_state_[4] = x_acc_low_pass_filter->output();
    com_state_[5] = y_acc_low_pass_filter->output();

    com_state_[2] = com_state_[2] + com_state_[4]*mercury::servo_rate;
    com_state_[3] = com_state_[3] + com_state_[5]*mercury::servo_rate;

    com_state_[0] = com_state_[0] + com_state_[2]*mercury::servo_rate;
    com_state_[1] = com_state_[1] + com_state_[3]*mercury::servo_rate;

  //if(count % 100 == 0){
    //dynacore::pretty_print(g_B, std::cout, "gravity_dir");
    //printf("    gravity_mag = %0.4f \n", gravity_mag);
    //printf("    theta_x = %0.4f \n", theta_x);    
    //printf("    theta_y = %0.4f \n", theta_y);        
    //dynacore::pretty_print(g_B_local, std::cout, "rotated gravity_dir");
    //printf("    norm(g_B_local) = %0.4f \n", g_B_local.norm());
    //dynacore::pretty_print(Oq_B_init, std::cout, "Initial body orientation w.r.t fixed frame: ");
    //dynacore::pretty_print(OR_B_init, std::cout, "OR_B_init: ");
     //dynacore::pretty_print(OB_xhat_xy, std::cout, "OB_xhat_xy");
     //printf("    theta_z = %0.4f \n", theta_z);    
     //dynacore::pretty_print(Oq_B_init_yaw_rotated, std::cout, "Yaw aligned Body orientation w.r.t fixed frame: ");
     //dynacore::pretty_print(OR_B_init_yaw_rotated, std::cout, "OR_B_init_yaw_rotated: ");
     //dynacore::pretty_print(Oq_B, std::cout, "Body orientation w.r.t fixed frame: ");
  //}    
*/
    //count++;
}

void BasicAccumulation::InitIMUOrientationEstimate(){
  // Finds The orientation of the body with respect to the fixed frame O. 
  // ^OR_B 
  // The algorithm attempts to solve ^OR_B * f_b = f_o, 
  // where f_b is the acceleration of gravity in the body frame.
  // f_o is the acceleration of gravity in the fixed frame
  // Note that ^OR_B will be equal to the orientation of the body w.r.t the fixed frame.

  // We will rotate the f_b using extrinsic rotation with global R_y (pitch) and R_x (roll) rotations
  // Thus, we will perform ^OR_x ^OR_y f_b = f_o.
  // and, ^OR_b = ^OR_x ^OR_y

  // A final yaw alignment is performed with ^OR_z * ^OR_b

  // This assumes that the +z vector of the IMU is in the opposite direction of gravity.


  g_B.setZero();
  g_B[0] = -x_acc_bias; 
  g_B[1] = -y_acc_bias;
  g_B[2] = -z_acc_bias; // We expect a negative number

  // Test Vector
  // f_b = [-0.057744 -0.001452  -0.998330]  
  // norm(f_b) = 9.7367
  // g_B[0] = -0.057744;
  // g_B[1] = -0.001452;
  // g_B[2] = -0.998330; // We expect a negative number

  gravity_mag = g_B.norm();
  g_B /= gravity_mag;

  dynacore::Quaternion q_world_Ry;
  dynacore::Quaternion q_world_Rx;      

  // Use Ry to rotate pitch and align gravity vector  ---------------------------
  // Prepare to rotate gravity vector
  g_B_local.w() = 0;
  g_B_local.x() = g_B[0];  g_B_local.y() = g_B[1]; g_B_local.z() = g_B[2];

  dynacore::Vect3 proj_B; proj_B.setZero();
  proj_B[1] = g_B[0];
  proj_B[1] = 0.0;
  proj_B[2] = g_B[2];

  proj_B /= proj_B.norm();

  // Local xhat direction
  dynacore::Vect3 xhat_B; xhat_B.setZero(); xhat_B[1] = 1.0;
  // Compute Pitch to rotate
  // theta_x = acos(xhat_B.dot(g_B));
  //theta_x = acos(xhat_B.dot(proj_B));
  theta_x = atan(g_B[0]/g_B[2]);
  //double pitch_val = (M_PI/2.0) + theta_x;
  double pitch_val = -theta_x;
  //dynacore::convert(0.0, pitch_val, 0.0, q_world_Ry);
  q_world_Ry.w() = cos(pitch_val/2.);
  q_world_Ry.x() = 0;
  q_world_Ry.y() = sin(pitch_val/2.);
  q_world_Ry.z() = 0;  

  // Rotate gravity vector to align the xhat directions
  g_B_local = dynacore::QuatMultiply( dynacore::QuatMultiply(q_world_Ry, g_B_local), q_world_Ry.inverse());
  //---------------------------------------------------------------------------

  // Use Rx to rotate the pitch and align gravity vector  ---------------------------
  // Local yhat direction
  dynacore::Vect3 yhat_A; yhat_A.setZero(); yhat_A[1] = 1.0;

  dynacore::Vect3 g_B_prime; g_B_prime.setZero();

  g_B_prime[0] = g_B_local.x();
  g_B_prime[1] = g_B_local.y();
  g_B_prime[2] = g_B_local.z();  
  
  // Compute Roll to rotate
  //theta_y = acos(yhat_A.dot(g_B_prime));
  theta_y = atan(g_B_prime[1]/g_B_prime[2]);

  double roll_val = theta_y;//-((M_PI/2.0) - theta_y);
  //dynacore::convert(0.0, 0.0, roll_val, q_world_Rx);
  q_world_Rx.w() = cos(roll_val/2.);;
  q_world_Rx.x() = sin(roll_val/2.);
  q_world_Rx.y() = 0;
  q_world_Rx.z() = 0;


  //Rotate gravity vector to align the yhat directions
  g_B_local = dynacore::QuatMultiply( dynacore::QuatMultiply(q_world_Rx, g_B_local), q_world_Rx.inverse());

  // Obtain initial body orientation w.r.t fixed frame.
  // Oq_B = q_x * q_y * q_b
  Oq_B_init = dynacore::QuatMultiply(q_world_Rx, q_world_Ry);//dynacore::QuatMultiply( q_world_Rx ,dynacore::QuatMultiply(q_world_Ry, Oq_B));
  // Set rotation matrix
  OR_B_init = Oq_B_init.normalized().toRotationMatrix();

  // //---------------------------------------------------------------------------
  // // Use Rz to rotate the yaw and align the x and y directions of the body orientation to the global frame.
  // // Extract the xhat direction of OR_B 
  // dynacore::Vect3 O_xhat_B; O_xhat_B.setZero(); 
  // O_xhat_B[0] = OR_B_init(0,0);
  // O_xhat_B[1] = OR_B_init(0,1);
  // O_xhat_B[2] = OR_B_init(0,2);  

  // // Construct only the x,y components of O_xhat_B
  // dynacore::Vect3 O_xhat_B_xy; O_xhat_B_xy.setZero(); 
  // O_xhat_B_xy[0] = OR_B_init(0,0);
  // O_xhat_B_xy[1] = OR_B_init(1,0);

  // OB_xhat_xy = O_xhat_B_xy;

  // O_xhat_B_xy /= O_xhat_B_xy.norm();


  // // Find the yaw to rotate
  // theta_z = acos(xhat_B.dot(O_xhat_B_xy));

  // // Use the y component to identify which direction to do the yaw rotation.
  // double xhat_ydir = O_xhat_B[1]; //
  // if (xhat_ydir > 0){
  //   theta_z = -theta_z;
  // }
  // double yaw_value = theta_z;

  // // // Rotate the quaternion of the body orientation to align the projected x and y directions
  // // dynacore::Quaternion q_world_Rz;
  // // dynacore::convert(yaw_value, 0.0, 0.0, q_world_Rz);

  // // // Get the body orientation with yaw aligned to the fixed frame 
  // // Oq_B_init_yaw_rotated = dynacore::QuatMultiply(q_world_Rz, Oq_B_init);
  // // OR_B_init_yaw_rotated = Oq_B_init_yaw_rotated.normalized().toRotationMatrix();

  // // // Initialize global orientation
  // Oq_B = Oq_B_init_yaw_rotated;

}
