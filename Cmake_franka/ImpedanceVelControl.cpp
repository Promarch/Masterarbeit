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
  std::string filename = "data_impedance_test/" + var_name + "_" + getCurrentDateTime() + ".txt";
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
  std::vector<std::array<double, 7>> tau_data, tau_filter_data, joint_position_data; // Stores joint torque and position
  std::vector<std::array<double, 5>> rotation_time_data; // stores the desired rotation with the current time
  std::vector<std::array<double, 16>> O_T_EE_data, F_T_EE_data; 
  std::vector<std::array<double, 6>> F_robot_data; // Stores wrench acting on EE
  std::vector<std::array<float , 6>> F_sensor_data, F_sensor_total_data; 
  // Similar to the stuff above, but instead of storing everything they are just need to convert the Eigen vectors into arrays
  std::array<double, 7> tau_d_array{};
  // Declare time global here so that i can display when the control failed in the except
  double time_global = 0.0;

  try {
    // Set Up basic robot function
    franka::Robot robot("192.168.1.11");
    // Set new end-effector
    robot.setEE({1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.150, 1.0});
    // robot.setK({1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0});
    // robot.setEE({1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0});
    setDefaultBehavior(robot);
    franka::Model model = robot.loadModel();
    franka::RobotState initial_state = robot.readOnce();

      // Damping/Stifness
    const double translation_stiffness{120.0};
    const double rotation_stiffness{6.0};
    Eigen::MatrixXd K_p = Eigen::MatrixXd::Identity(6, 6);
    Eigen::MatrixXd K_d = Eigen::MatrixXd::Identity(6, 6);
    Eigen::VectorXd Kn(7); Kn << 0, 0, 0, 0, 0, 200, 0;
    K_p.topLeftCorner(3,3) *= translation_stiffness; 
    K_p.bottomRightCorner(3,3) *= rotation_stiffness; 
    K_d.topLeftCorner(3,3) *= 1*sqrt(translation_stiffness);
    K_d.bottomRightCorner(3,3) *= 1*sqrt(rotation_stiffness);

      // Time for the loop
    
    double time_cycle = 0.0;
    double time_max = 6;
    double period_acc = 3;
    double period_dec = 0.8; // time for decceleration, to stop abrubt braking
    double sampling_interval = 0.25;
    double next_sampling_time = 0;
    double dt = 0;
    
    // Initial orientation
    Eigen::Map<const Eigen::Matrix<double, 7,1>> q_init(initial_state.q.data());
    Eigen::Affine3d transform_init(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
    Eigen::Vector3d position_init(transform_init.translation());
    // Eigen::Quaterniond rotation_init(transform_init.linear());
    Eigen::Quaterniond rotation_init(0.0, 0.7071068, 0.7071068, 0.0);
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
    // Translate the desired rotation into local coordinate system (rotation in EE-CoSy instead of base CoSy)
    Eigen::Quaterniond rot_quaternion_rel = rotation_init * quaternion_combined * rotation_init.inverse();
    // Add rotation to initial orientation
    Eigen::Quaterniond rot_d = rot_quaternion_rel * rotation_init; 
    // std::cout << "quaternion_flexion: " << quaternion_flexion << "\nRotation_init: " << rotation_init << "\nDesired quat: " << rot_d << std::endl;
      // Desired relative position
    Eigen::Matrix<double, 3,1> pos_d;
    pos_d = position_init;

      // Variables to set new position
    // Decceleration
    bool decceleration = false; // if true, robot deccelerates, used to stop the motion before setting a new position
    double t_dec = 0;   // will be set the moment a decceleration is called
    // New position
    Eigen::Matrix<double, 3, 1> pos_temp = pos_d;
    bool new_pos = false;   // state variable, if true new position will be set
    std::srand(std::time(nullptr)); // initialize random seed
    double max_flexion = -M_PI/4;    // Max possible flexion
    double max_internal = M_PI/12;   // Max possible internal-external rotation
    double range_internal = 2*max_internal; // Possible values (max_internal-min_internal)
    // Debug stuff
    double t_new_pos = 0; // Debug variable to see when a new position has been called
    Eigen::Matrix<double, 5, 1> rotation_time; 
    rotation_time << rot_d.coeffs(), time_global;
    std::array<double, 5> rotation_time_array{}; 
    Eigen::VectorXd::Map(&rotation_time_array[0], 5) = rotation_time;
    rotation_time_data.push_back(rotation_time_array);

      // Pre-allocation
    // Acceleration and decceleration factor
    double factor_rot{0};
    double factor_pos{0};
    Eigen::Matrix<double, 6,1> factor_filter; 
    // Commanded wrench 
    Eigen::Matrix<double, 6,1> h_c; 
    Eigen::Matrix<double, 6, 1> cart_vel; cart_vel.setZero(); // Commanded speed
    // Error between current and desired orientation
    Eigen::Matrix<double, 6,1> error; 

    std::cout << "Robot will start moving now \n"
              << "Press Enter to continue... \n";
    std::cin.ignore();

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

/*
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
          angle_internal = (std::rand()/(RAND_MAX/range_internal))-max_internal;
        }
        Eigen::AngleAxisd angle_axis_internal(angle_internal, axis_internal);
        Eigen::Quaterniond quaternion_internal(angle_axis_internal);
        // Combine the rotations
        quaternion_combined = quaternion_internal * quaternion_flexion;
        rot_quaternion_rel = rotation_init * quaternion_combined * rotation_init.inverse();
        // Calculate new rotation
        rot_d = rot_quaternion_rel * rotation_init; 
          // Reset state variable and cycle time
        new_pos = false; 
        time_cycle = 0;
          // Print text and update data variable
        rotation_time << rot_d.coeffs(), time_global;
        Eigen::VectorXd::Map(&rotation_time_array[0], 5) = rotation_time;
        rotation_time_data.push_back(rotation_time_array);
        std::cout << "New flexion: " << angle_flexion << " and internal rot: " << angle_internal << " and quaternion \n" << rot_d;
        std::cout << "\nCurrent acceleration factor: " << factor_rot << std::endl;  
      }
*/
      // -------------------------------------------------------------------
      // ----      Acceleration and decceleration factor                ----
      // -------------------------------------------------------------------

      // Calculate value depending on where in the acceleration/decceleration cycle the robot is in
      if ((time_global+period_dec)>time_max) {   // Deccelerate if the maximum time has been reached
        factor_rot = (1 + std::cos(M_PI * (time_global-(time_max-period_dec))/period_dec))/2 * factor_rot;  // * factor_rot added cause it is not garanteed that factor_rot==1 when close to t_maxy
        factor_pos = (1 + std::cos(M_PI * (time_global-(time_max-period_dec))/period_dec))/2 * factor_rot;
      }
      else if (decceleration==true){  // Deccelerate if decceleration has been called because of a new position
        factor_rot = (1 + std::cos(M_PI * (time_cycle-t_dec)/period_dec))/2 * factor_rot;
        factor_pos = 1;
        if (factor_rot < 0.001) {  // Give command for new pos if slow enough
          decceleration = false;
          new_pos = true;
          std::cout << "New position to set" << std::endl;
        }
      }
      else if (time_cycle<period_acc) {    // start acceleration at the beginning of a new cycle
        factor_rot = (1 - std::cos(M_PI * time_cycle/period_acc))/2;
        factor_pos = 1;
      }
      else { // If no other cases are valid, keep factor constant
        factor_rot = 1;
        factor_pos = 1;
      }
      // Only apply smooth acceleration to rotation so that position deviations are immediatly corrected
      factor_filter << factor_pos, factor_pos, factor_pos, factor_rot, factor_rot, factor_rot; 

      // -------------------------------------------------------------------
      // ----                 Calculate trajectory                      ----
      // -------------------------------------------------------------------

        // Compute trajectory between current and desired orientation
      // Positional error
      error.setZero();
      error.head(3) << pos_d - position; 
      // Rotational error
      if (rot_d.coeffs().dot(rotation.coeffs()) < 0.0) {
        rotation.coeffs() << -rotation.coeffs();
      }
      // Difference quaternion
      Eigen::Quaterniond error_quaternion(rotation.inverse() * rot_d);
      error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
      // Transform to base frame
      error.tail(3) << transform.rotation() * error.tail(3);

      // Calculate velocity
      cart_vel.head(3) << error.head(3);
      // Normalize rotation error and scale with max_speed
      cart_vel.tail(3) << error.tail(3)/error.tail(3).norm();

      // Calculate the necessary wrench, from Handbook of robotics, Chap. 7
      h_c.setZero();
      h_c = K_p * cart_vel - K_d * jacobian * dq; // - K_d * jacobian * dq 

      // Add nullspace control to stay in fixed position
      // Eigen::MatrixXd pseudo_inv = jacobian.completeOrthogonalDecomposition().pseudoInverse();
      // Eigen::MatrixXd h_null = (Eigen::MatrixXd::Identity(7,7) - pseudo_inv * jacobian) * Kn.cwiseProduct(q_init-q);

      // Calculate torque and map to an array
      Eigen::MatrixXd tau_d = jacobian.transpose() * h_c.cwiseProduct(factor_filter); //.cwiseProduct(factor_filter); //  + h_null 
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
        std::cout << "Time: " << time_global << ", acc factor: " << factor_rot << "\n";
        std::cout << "tau_d: " << tau_d.transpose() << "\nError: " << error.transpose() << "\n" << "\n"; // << ", rot_d: " << rot_d 
        next_sampling_time += sampling_interval;
      }

      // -------------------------------------------------------------------
      // ----                 Command new position                      ----
      // -------------------------------------------------------------------

      if (decceleration==false) {
        // Set new position if: target reached/force threshold reached/no movement
        if (error.norm()<0.005) {   // Target reached
            // Set decceleration and current time to properly calculate the decceleration factor
          decceleration = true;
          t_dec = time_cycle;
            // Print which case was called and the current values that called it
          t_new_pos = time_global;
          std::cout << "Target reached, time:" << time_global << ", current error is: \n" << error << std::endl;
        }
        else if (F_ext.tail(3).norm()>4) {   // Torque threshold reached
            // Set decceleration and current time to properly calculate the decceleration factor
          decceleration = true;
          t_dec = time_cycle;
            // Print which case was called and the current values that called it
          t_new_pos = time_global; 
          std::cout << "Max torque reached, time:" << time_global << ", current forces are: \n" << F_ext << std::endl;
        }
/*        else if (dq.norm()<0.01 and time_cycle>(period_acc+0.3)) {  // No movement, probably equilibrium position
            // Set decceleration and current time to properly calculate the decceleration factor
          decceleration = true;
          t_dec = time_cycle;
            // Print which case was called and the current values that called it
          t_new_pos = time_global; 
          std::cout << "Movement stopped, time:" << time_global << ", dq: \n" << dq << std::endl;
        }*/
      }

      
      // Add the current data to the array
      tau_data.push_back(tau_d_array);
      tau_filter_data.push_back(robot_state.tau_ext_hat_filtered);
      joint_position_data.push_back(robot_state.q);
      O_T_EE_data.push_back(robot_state.O_T_EE);
      F_T_EE_data.push_back(robot_state.F_T_EE);
      F_robot_data.push_back(robot_state.K_F_ext_hat_K);

      // Send desired tau to the robot
      return tau_d_array;

      // // Return 0 for debugging
      // std::array<double, 7> return_0 = {0, 0, 0, 0, 0, 0, 0}; 
      // return return_0; 

    };

    // start control loop
    robot.control(force_control_callback);
    
    // Write Data to .txt file
    writeDataToFile(tau_data);
    writeDataToFile(tau_filter_data);
    writeDataToFile(F_robot_data);
    writeDataToFile(joint_position_data);
    writeDataToFile(O_T_EE_data);
    writeDataToFile(F_T_EE_data);
    writeDataToFile(rotation_time_data);
  }
  // Catches Exceptions caused within the execution of a program (I think)
  catch (const franka::ControlException& e) {
    std::cout << "Time: " << time_global << ", Control Exception: \n" << e.what() << std::endl;
    writeDataToFile(tau_data);
    writeDataToFile(tau_filter_data);
    writeDataToFile(F_robot_data);
    writeDataToFile(joint_position_data);
    writeDataToFile(O_T_EE_data);
    writeDataToFile(F_T_EE_data);
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
