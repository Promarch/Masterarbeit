#include <iostream>
#include <iomanip>
#include <fstream>
#include <ctime>
#include <array>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

// Sets default collision parameters, from https://frankaemika.github.io/libfranka/generate_joint_position_motion_8cpp-example.html
void setDefaultBehavior(franka::Robot &robot) {

    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
    robot.setJointImpedance({{3000, 3000, 3000, 3000, 3000, 3000, 3000}});
    robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
};

// Macro to convert variable name to string for the function
#define writeDataToFile(data) writeDataToFileImpl(data, #data)
std::string getCurrentDateTime() {
  std::time_t now = std::time(nullptr);
  char buf[80];
  std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", std::localtime(&now));
  return std::string(buf);
}
template <std::size_t N>
void writeDataToFileImpl(const std::vector<std::array<double, N>>& data, const std::string& var_name) {
  std::string filename = "data_output/" + var_name + "_" + getCurrentDateTime() + ".txt";
  std::ofstream data_file(filename);
  if (data_file.is_open()) {
    data_file << std::fixed << std::setprecision(4);
    for (const auto& sample : data) {
      for (size_t i = 0; i < sample.size(); ++i) {
        data_file << sample[i];
        if (i < sample.size() - 1) {
          data_file << ", ";
        }
      }
      data_file << "\n";
    }
    data_file.close();
  } else {
    std::cerr << "Unable to open file for writing" << std::endl;
  }
}

int main() {

  // Variables to store the torque or position of the robot during the movement
  std::vector<std::array<double, 7>> tau_data, tau_filter_data, tau_desired_data, joint_position_data; // Stores joint torque and position
  std::vector<std::array<double, 3>> position_data; // Stores the position in x,y,z
  std::vector<std::array<double, 6>> force_data; // Stores the force and torque on the EE
  std::vector<std::array<double, 5>> rotation_time_data; // stores the desired rotation with the current time

  try {
    // Set Up basic robot function
    franka::Robot robot("192.168.1.11");
    // Set new end-effector
    robot.setEE({1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.205, 1.0});
    // robot.setK({1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0});
    // robot.setEE({1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0});
    setDefaultBehavior(robot);
    franka::Model model = robot.loadModel();
    franka::RobotState initial_state = robot.readOnce();

      // Damping/Stifness
    const double translation_stiffness{150.0};
    const double rotation_stiffness{30.0};
    Eigen::MatrixXd K_p = Eigen::MatrixXd::Identity(6, 6);
    Eigen::MatrixXd K_d = Eigen::MatrixXd::Identity(6, 6);
    Eigen::MatrixXd K_i = Eigen::MatrixXd::Identity(6, 6); 
    Eigen::VectorXd Kn(7); Kn << 0, 0, 0, 0, 0, 50, 0;
    K_p.topLeftCorner(3,3) *= translation_stiffness; 
    K_p.bottomRightCorner(3,3) *= rotation_stiffness; 
    K_d.topLeftCorner(3,3) *= 1 * sqrt(translation_stiffness);
    K_d.bottomRightCorner(3,3) *= 1 * sqrt(rotation_stiffness);
    K_i.topLeftCorner(3,3) *= 2 * sqrt(translation_stiffness);
    K_i.bottomRightCorner(3,3) *= 2 * sqrt(rotation_stiffness); 

      // Time for the loop
    double time_global = 0.0;
    double time_cycle = 0.0;
    double time_max = 8;
    double period_acc = 2;
    double period_dec = 0.5; // time for decceleration, to stop abrubt braking
    double sampling_interval = 0.25;
    double next_sampling_time = 0;
    double dt = 0;
    
    // Initial orientation
    Eigen::Map<const Eigen::Matrix<double, 7,1>> q_init(initial_state.q.data());
    Eigen::Affine3d transform_init(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
    Eigen::Vector3d position_init(transform_init.translation());
    Eigen::Quaterniond rotation_init(transform_init.linear());
    Eigen::Matrix3d rot_matrix_init = rotation_init.toRotationMatrix();
    
      // Desired Rotation, created with quaternions
    // Flexion (Rotation around y in local CoSy) 
    double angle_flexion = -M_PI/9;
    Eigen::Vector3d axis_flexion(1,0,0);
    Eigen::AngleAxisd angle_axis_flexion(angle_flexion, axis_flexion);
    Eigen::Quaterniond quaternion_flexion(angle_axis_flexion);
    // Varus-Valgus (Rotation around z in local CoSy) 
    double angle_internal = 0;
    Eigen::Vector3d axis_internal(0,1,0);
    Eigen::AngleAxisd angle_axis_internal(angle_internal, axis_internal);
    Eigen::Quaterniond quaternion_internal(angle_axis_internal);
    // Combine the rotations
    Eigen::Quaterniond quaternion_combined = quaternion_internal * quaternion_flexion;
    Eigen::Quaterniond rot_quaternion_rel = rotation_init * quaternion_combined * rotation_init.inverse();
      // Desired relative position
    Eigen::Matrix<double, 3,1> pos_d;
    pos_d = position_init;
    Eigen::Quaterniond rot_d = rot_quaternion_rel * rotation_init; 
    std::cout << "quaternion_flexion: " << quaternion_flexion << "\nRotation_init: " << rotation_init << std::endl;

    return -1;

      // Variables to set new position
    // Decceleration
    bool decceleration = false; // if true, robot deccelerates, used to stop the motion before setting a new position
    double t_dec = 0;   // will be set the moment a decceleration is called
    // New position
    Eigen::Matrix<double, 3, 1> pos_temp; pos_temp.setZero();
    bool new_pos = false;   // state variable, if true new position will be set
    std::srand(std::time(nullptr)); // initialize random seed
    double max_flexion = -M_PI/4;    // Max possible flexion
    double max_internal = M_PI/18;  // Max possible internal rotation
    double min_internal = -M_PI/18; // Min possible internal rotation
    // Debug stuff
    double t_new_pos = 0; // Debug variable to see when a new position has been called
    Eigen::Matrix<double, 5, 1> rotation_time; 
    rotation_time << rot_d.coeffs(), time_global;
    std::array<double, 5> rotation_time_array{}; 
    Eigen::VectorXd::Map(&rotation_time_array[0], 5) = rotation_time;
    rotation_time_data.push_back(rotation_time_array);

    // Other variables
    double factor_rot{0};
    double factor_pos{0};


// --------------------------------------------------------------------------------------------
// ----                                                                                    ----
// ----                           Control loop                                             ----
// ----                                                                                    ----
// --------------------------------------------------------------------------------------------

    auto force_control_callback = [&] (const franka::RobotState& robot_state, franka::Duration period) -> franka::Torques {
      dt = period.toSec();
      time_global += dt;
      time_cycle += dt;

        // Get state variables
      std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
      Eigen::Map<const Eigen::Matrix<double, 6,7>> jacobian(jacobian_array.data());
      Eigen::Map<const Eigen::Matrix<double, 7,1>> dq(robot_state.dq.data()); 
      Eigen::Map<const Eigen::Matrix<double, 7,1>> q(robot_state.q.data());
      Eigen::Map<const Eigen::Matrix<double, 6,1>> F_ext(robot_state.K_F_ext_hat_K.data());
        // Get current position
      Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
      Eigen::Vector3d position(transform.translation());
      Eigen::Quaterniond rotation(transform.linear());
      Eigen::Matrix3d rot_matrix = rotation.toRotationMatrix();

      // -------------------------------------------------------------------
      // ----                   Set new position                        ----
      // -------------------------------------------------------------------

      if (new_pos==true){
          // Set new angles and compute new rotation
        // New flexion
        angle_flexion = (std::rand()/(RAND_MAX/max_flexion)); 
        Eigen::AngleAxisd angle_axis_flexion(angle_flexion, axis_flexion);
        Eigen::Quaterniond quaternion_flexion(angle_axis_flexion);
        // New internal rotation 
        if (abs(angle_flexion) < M_PI/18) {
          angle_internal = 0; // No internal rotation if the knee is extended
        }
        else {
          angle_internal = (std::rand()/(RAND_MAX/max_internal));
        }
        Eigen::AngleAxisd angle_axis_internal(angle_internal, axis_internal);
        Eigen::Quaterniond quaternion_internal(angle_axis_internal);
        // Combine the rotations
        quaternion_combined = quaternion_internal * quaternion_flexion;
        rot_quaternion_rel = rotation_init * quaternion_combined * rotation_init.inverse();
        // Calculate new rotation
        rot_d = rot_quaternion_rel * rotation_init; 
          // Set desired position temporarely to the current position. 
          // This is done to be able to continuously keep the positional control active instead of having to smooth it in/out
        pos_d = position;
        pos_temp = position;
          // Reset state variable
        new_pos = false; 
          // Print text and update data variable
        rotation_time << rot_d.coeffs(), time_global;
        Eigen::VectorXd::Map(&rotation_time_array[0], 5) = rotation_time;
        rotation_time_data.push_back(rotation_time_array);
        std::cout << "New flexion: " << angle_flexion << " and internal rot: " << angle_internal << " and quaternion \n" << rot_d;
        std::cout << "\nCurrent acceleration factor: " << factor_rot << std::endl;  
      }

      // -------------------------------------------------------------------
      // ----      Acceleration and decceleration factor                ----
      // -------------------------------------------------------------------

      // Calculate value depending on where in the acceleration/decceleration cycle the robot is in
      if ((time_global+period_dec)>time_max) {   // Deccelerate if the maximum time has been reached
        factor_rot = (1 + std::cos(M_PI * (time_global-(time_max-period_dec))/period_dec))/2;
        factor_pos = factor_rot;
      }
      else if (decceleration==true){  // Deccelerate if decceleration has been called because of a new position
        factor_rot = (1 + std::cos(M_PI * (time_cycle-t_dec)/period_dec))/2;
        factor_pos = factor_rot;
        if (factor_rot < 0.0001) {  // Give command for new pos if slow enough
          decceleration = false;
          time_cycle = 0;
          new_pos = true;
          std::cout << "New position to set" << std::endl;
        }
      }
      else if (time_cycle<period_acc) {    // start acceleration at the beginning of a new cycle
        factor_rot = (1 - std::cos(M_PI * time_cycle/period_acc))/2;
        factor_pos = 1;
        if (pos_d!=position_init){  // needed when a new position has been assigned
          pos_d = pos_temp + (time_cycle/period_acc) * (position_init - pos_temp);
        }
      }
      else { // If no other cases are valid, keep factor constant
        factor_rot = 1;
        factor_pos = 1;
      }
      // Only apply smooth acceleration to rotation so that position deviations are immediatly corrected
      Eigen::Matrix<double, 6,1> factor_filter; 
      factor_filter << factor_pos, factor_pos, factor_pos, factor_rot, factor_rot, factor_rot; 

      // -------------------------------------------------------------------
      // ----                 Calculate trajectory                      ----
      // -------------------------------------------------------------------

        // Compute trajectory between current and desired orientation
      // Positional error
      Eigen::Matrix<double, 6,1> error; error.setZero();
      error.head(3) << pos_d - position; 
      // Rotational error
      if (rot_d.coeffs().dot(rotation.coeffs()) < 0.0) {
        rotation.coeffs() << -rotation.coeffs();
      }
      // Difference quaternion
      Eigen::Quaterniond error_quaternion(rotation.inverse() * rot_d);
      error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
      // Transform to base frame
      error.tail(3) << transform.linear() * error.tail(3);

      // Calculate the necessary wrench, from Handbook of robotics, Chap. 7
      Eigen::Matrix<double, 6,1> h_c; h_c.setZero();
      Eigen::Matrix<double, 6,1> v_e = jacobian * dq;
      h_c = K_p * error - K_d * v_e ; 

      // // Add nullspace control to stay in fixed position
      // Eigen::MatrixXd pseudo_inv = jacobian.completeOrthogonalDecomposition().pseudoInverse();
      // Eigen::Matrix<double, 6,1> error_null = error; error_null.tail(3) << 0,0,0;
      // Eigen::MatrixXd h_null = (Eigen::MatrixXd::Identity(7,7) - pseudo_inv * jacobian) * Kn.cwiseProduct(q_init-q);

      // Calculate torque and map to an array
      Eigen::MatrixXd tau_d = jacobian.transpose() * h_c.cwiseProduct(factor_filter); //  + h_null 
      std::array<double, 7> tau_d_array{};
      Eigen::VectorXd::Map(&tau_d_array[0], 7) =  tau_d;

      // Calculate positional error for tests
      double distance = (pos_d-position).norm(); 

      // Stop if time is reached
      if (time_global >= time_max) {
        std::cout << std::fixed << std::setprecision(3);
        std::cout << "Time: " << time_global << std::endl << "Error: " << std::endl << error << std::endl << "Position error: " << distance << std::endl << std::endl; 
        std::cout << "\nFinished motion, shutting down example, time new pos: " << t_new_pos << std::endl;
        franka::Torques output = tau_d_array;
        return franka::MotionFinished(output);
      }

      // Print current wrench, time, and absolute positional error
      if (time_global >= next_sampling_time) {
        std::cout << std::fixed << std::setprecision(3);
        std::cout << "Time: " << time_global << ", rot_d: " << rot_d << "\nError: \n" << error << "\nPosition error: " << distance << "\n" << std::endl; 
        next_sampling_time += sampling_interval;
      }

      // -------------------------------------------------------------------
      // ----                 Command new position                      ----
      // -------------------------------------------------------------------

      if (decceleration==false) {
        // Set new position if: target reached/force threshold reached/no movement
        if (error.norm()<0.05) {   // Target reached
            // Set decceleration and current time to properly calculate the decceleration factor
          decceleration = true;
          t_dec = time_cycle;
            // Print which case was called and the current values that called it
          t_new_pos = time_global;
          std::cout << "Target reached, time:" << time_cycle << ", current error is: \n" << error << std::endl;
        }
        else if (F_ext.tail(3).norm()>4) {   // Torque threshold reached
            // Set decceleration and current time to properly calculate the decceleration factor
          decceleration = true;
          t_dec = time_cycle;
            // Print which case was called and the current values that called it
          t_new_pos = time_global; 
          std::cout << "Max torque reached, time:" << time_cycle << ", current forces are: \n" << F_ext << std::endl;
        }
        else if (dq.norm()<0.01 and factor_rot>0.9) {  // No movement, probably equilibrium position
            // Set decceleration and current time to properly calculate the decceleration factor
          decceleration = true;
          t_dec = time_cycle;
            // Print which case was called and the current values that called it
          t_new_pos = time_global; 
          std::cout << "Movement stopped, time:" << time_cycle << ", dq: \n" << dq << std::endl;
        }
      }
      // Calculate stupid stuff once again
      // std::array<double, 42> body_jacobian_array = model.bodyJacobian(franka::Frame::kEndEffector, robot_state);
      // Eigen::Map<const Eigen::Matrix<double, 6,7>> body_jacobian(body_jacobian_array.data());
      // Eigen::Matrix<double, 6, 1> F_tau_d = body_jacobian.transpose().completeOrthogonalDecomposition().pseudoInverse() * tau_d; 

      // Map the position and error to an array (I cant add Eigen vectors to arrays)
      std::array<double, 3> pos_array{}; 
      std::array<double, 6> error_array{}, F_tau_d_array{}; 
      Eigen::VectorXd::Map(&pos_array[0], 3) = position;
      Eigen::VectorXd::Map(&error_array[0], 6) = error;
      // Eigen::VectorXd::Map(&F_tau_d_array[0], 6) = F_tau_d;
      
      // Add the current data to the array
      tau_data.push_back(tau_d_array);
      tau_filter_data.push_back(robot_state.tau_ext_hat_filtered);
      position_data.push_back(pos_array);
      force_data.push_back(robot_state.K_F_ext_hat_K);
      joint_position_data.push_back(robot_state.q);

      // Send desired tau to the robot
      return tau_d_array;

    };

    // start control loop
    robot.control(force_control_callback);
    
    // Write Data to .txt file
    writeDataToFile(tau_data);
    writeDataToFile(tau_filter_data);
    writeDataToFile(position_data);
    writeDataToFile(force_data);
    writeDataToFile(joint_position_data);
    writeDataToFile(rotation_time_data);
  }
  // Catches Exceptions caused within the execution of a program (I think)
  catch (const franka::ControlException& e) {
    std::cout << "Control Exception: \n" << e.what() << std::endl;
    writeDataToFile(tau_data);
    writeDataToFile(tau_filter_data);
    writeDataToFile(position_data);
    writeDataToFile(force_data);
    writeDataToFile(joint_position_data);
    writeDataToFile(rotation_time_data);
    return -1;
  }
  // General exceptions
  catch (const franka::Exception& e) {
    std::cout << "Other Exception: \n" << e.what() << std::endl;
    writeDataToFile(tau_data);
    return -1;
  }
  
}
