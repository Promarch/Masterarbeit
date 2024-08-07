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
    robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
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
  std::vector<std::array<double, 7>> tau_data, joint_position_data; // Vector to store the torque
  std::vector<std::array<double, 3>> position_data, rotation_data; // Vector to store the torque
  std::vector<std::array<double, 6>> force_data, tau_force_data; // Vector to store the force and torque on the EE
  
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
    const double translation_stiffness{300.0};
    const double rotation_stiffness{100.0};
    Eigen::MatrixXd K_p = Eigen::MatrixXd::Identity(6, 6);
    Eigen::MatrixXd K_d = Eigen::MatrixXd::Identity(6, 6);
    Eigen::MatrixXd K_i = Eigen::MatrixXd::Identity(6, 6); 
    Eigen::VectorXd Kn(7); Kn << 10, 10, 10, 10, 5, 5, 5;
    K_p.topLeftCorner(3,3) *= translation_stiffness; 
    K_p.bottomRightCorner(3,3) *= rotation_stiffness; 
    K_d.topLeftCorner(3,3) *= 1 * sqrt(translation_stiffness);
    K_d.bottomRightCorner(3,3) *= 1 * sqrt(rotation_stiffness);
    K_i.topLeftCorner(3,3) *= 2 * sqrt(translation_stiffness);
    K_i.bottomRightCorner(3,3) *= 2 * sqrt(rotation_stiffness); 

      // Time for the loop
    double time = 0.0;
    double time_max = 8;
    double time_acc = 5;
    double time_dec = 0.5; // time for decceleration, to stop abrubt braking
    double sampling_interval = 0.1;
    double next_sampling_time = 0;
    Eigen::Matrix<double, 6,1> int_error; int_error.setZero();
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
    Eigen::Vector3d axis_internal(1,0,0);
    Eigen::AngleAxisd angle_axis_internal(angle_internal, axis_internal);
    Eigen::Quaterniond quaternion_internal(angle_axis_internal);
    // Combine the rotations
    Eigen::Quaterniond quaternion_combined = quaternion_internal * quaternion_flexion;
    Eigen::Quaterniond rot_quaternion_rel = rotation_init * quaternion_combined * rotation_init.inverse();
      // Desired relative position
    Eigen::Matrix<double, 3,1> pos_d;
    pos_d << position_init;
    Eigen::Quaterniond rot_d = rot_quaternion_rel * rotation_init; 
    
    // Other variables
    double factor_rot{0};
    double factor_pos{0};


    auto force_control_callback = [&] (const franka::RobotState& robot_state, franka::Duration period) -> franka::Torques {
      dt = period.toSec();
      time += dt;

        // Factor for smooth acceleration
      // numerical value
      if (time<time_acc) {
        factor_rot = (1 - std::cos(M_PI * time/time_acc))/2;
        factor_pos = 1;
      } 
      else if (time<(time_max-time_dec)){
        factor_rot = 1;
        factor_pos = 1;
      }
      else {
        factor_rot = (1 + std::cos(M_PI * (time-(time_max-time_dec))/time_dec))/2;
        factor_pos = factor_rot;
      }
      // Only apply to rotation so that position deviations are immediatly corrected
      Eigen::Matrix<double, 6,1> factor_filter; 
      factor_filter << factor_pos, factor_pos, factor_pos, factor_rot, factor_rot, factor_rot;

        // Get state variables
      std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
      Eigen::Map<const Eigen::Matrix<double, 6,7>> jacobian(jacobian_array.data());
      Eigen::Map<const Eigen::Matrix<double, 7,1>> dq(robot_state.dq.data()); 
      Eigen::Map<const Eigen::Matrix<double, 7,1>> q(robot_state.q.data());
        // Get current position
      Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
      Eigen::Vector3d position(transform.translation());
      Eigen::Quaterniond rotation(transform.linear());
      Eigen::Matrix3d rot_matrix = rotation.toRotationMatrix();

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

      // Add nullspace control to stay in fixed position
      Eigen::MatrixXd pseudo_inv = jacobian.completeOrthogonalDecomposition().pseudoInverse();
      Eigen::MatrixXd h_null = (Eigen::MatrixXd::Identity(7,7) - pseudo_inv * jacobian) * Kn.cwiseProduct(q_init-q)*5;

      // Calculate torque and map to an array
      std::array<double, 7> tau_d{};
      Eigen::VectorXd::Map(&tau_d[0], 7) = jacobian.transpose() * h_c.cwiseProduct(factor_filter) + h_null; // 

      // Calculate positional error for tests
      double distance = (pos_d-position).norm(); 

      // Stop if time is reached
      if (time >= time_max) {
        std::cout << std::fixed << std::setprecision(3);
        std::cout << "Time: " << time << std::endl << "Error: " << std::endl << error << std::endl << "Position error: " << distance << std::endl << std::endl; 
        std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
        franka::Torques output = tau_d;
        return franka::MotionFinished(output);
      }

      // Print current wrench, time, and absolute positional error
      if (time >= next_sampling_time) {
        std::cout << std::fixed << std::setprecision(3);
        std::cout << "Time: " << time << std::endl << "Error: " << std::endl << error << std::endl << "Position error: " << distance << std::endl<< std::endl; 
        next_sampling_time += sampling_interval;
      }

      // Map the position and error to an array (I cant add Eigen vectors to arrays)
      std::array<double, 3> pos_array{}; 
      std::array<double, 6> error_array{}; 
      Eigen::VectorXd::Map(&pos_array[0], 3) = position;
      Eigen::VectorXd::Map(&error_array[0], 6) = error;
      
      // Add the current data to the array
      tau_data.push_back(tau_d);
      position_data.push_back(pos_array);
      force_data.push_back(robot_state.K_F_ext_hat_K);
      joint_position_data.push_back(robot_state.q);

      // Send desired tau to the robot
      return tau_d;

    };

    // start control loop
    robot.control(force_control_callback);
    // Write Data to .txt file
    writeDataToFile(tau_data);
    writeDataToFile(position_data);
    writeDataToFile(force_data);
    writeDataToFile(joint_position_data);
  }
  // Catches Exceptions caused within the execution of a program (I think)
  catch (const franka::ControlException& e) {
    std::cout << "Control Exception: \n" << e.what() << std::endl;
    writeDataToFile(tau_data);
    writeDataToFile(position_data);
    writeDataToFile(force_data);
    writeDataToFile(joint_position_data);
    return -1;
  }
  // General exceptions
  catch (const franka::Exception& e) {
    std::cout << "Other Exception: \n" << e.what() << std::endl;
    writeDataToFile(tau_data);
    return -1;
  }
  
  
}
