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
  std::vector<std::array<double, 7>> tau_data; // Vector to store the torque
  std::vector<std::array<double, 3>> position_data, rotation_data; // Vector to store the torque
  std::vector<std::array<double, 6>> force_data; // Vector to store the force and torque on the EE
  std::vector<std::array<double, 6>> error_data; // Vector 
  
  try {
    // Set Up basic robot function
    franka::Robot robot("192.168.1.11");
    // Set new end-effector
    robot.setEE({1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.225, 1.0});
    // robot.setEE({1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0});
    setDefaultBehavior(robot);
    franka::Model model = robot.loadModel();
    franka::RobotState initial_state = robot.readOnce();

      // Damping/Stifness
    const double translation_stiffness{1.0};
    const double rotation_stiffness{0.1};
    Eigen::MatrixXd K_p = Eigen::MatrixXd::Identity(6, 6);
    Eigen::MatrixXd K_d = Eigen::MatrixXd::Identity(6, 6);
    Eigen::MatrixXd K_i = Eigen::MatrixXd::Identity(6, 6); 
    K_p.topLeftCorner(3,3) *= translation_stiffness; 
    K_p.bottomRightCorner(3,3) *= rotation_stiffness; 
    K_d.topLeftCorner(3,3) *= 1 * sqrt(translation_stiffness);
    K_d.bottomRightCorner(3,3) *= 1 * sqrt(rotation_stiffness);
    K_i.topLeftCorner(3,3) *= 2 * sqrt(translation_stiffness);
    K_i.bottomRightCorner(3,3) *= 2 * sqrt(rotation_stiffness); 

      // Time variables for the loop
    double time = 0.0;
    double time_max = 5; // Maximum runtime
    double time_acc = 2;  // Time spent accelerating
    double time_dec = 0.5; // time for decceleration, to stop abrubt braking
    double sampling_interval = 0.1; // Interval at which the console outputs the current error
    double next_sampling_time = 0;  // Needed for the sampling interval
    Eigen::Matrix<double, 6,1> int_error; int_error.setZero();  // Integral error
    double dt = 0;  // Period
    
    // Initial orientation
    Eigen::Affine3d transform_init(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
    Eigen::Vector3d position_init(transform_init.translation());
    Eigen::Quaterniond rotation_init(transform_init.linear());
    Eigen::Matrix3d rot_matrix_init = rotation_init.toRotationMatrix();
    Eigen::Map<const Eigen::Matrix<double, 6, 1>> F_init(initial_state.K_F_ext_hat_K.data()); 
      // Compute current rotation in angles for plots
    double roll_init = std::atan2(rot_matrix_init(2, 1), rot_matrix_init(2, 2));
    double pitch_init = std::asin(-rot_matrix_init(2, 0));
    double yaw_init = std::atan2(rot_matrix_init(1, 0), rot_matrix_init(0, 0));
    
      // Desired force
    double internalTorque{0};
    double flexionTorque{0.0};
    double varusTorque{0};
    Eigen::Matrix<double, 6, 1> F_d{0, 0, 8, internalTorque, flexionTorque, varusTorque};
    F_d = F_d + F_init; // 
    
    // Other variables
    double factor_rot{0};
    double factor_pos{0};


    auto force_control_callback = [&] (const franka::RobotState& robot_state, franka::Duration period) -> franka::Torques {
      dt = period.toSec();
      time += dt;

        // Factor for smooth acceleration
        // the rotation have a smooth transition, the position starts immediatly at 1 since they are supposed to counteract movement instead of causing it
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
      // Combine the factors
      Eigen::Matrix<double, 6,1> factor_filter; 
      factor_filter << factor_rot, factor_rot, factor_rot, factor_rot, factor_rot, factor_rot;

        // Get model variables
      std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
      Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
        // Get current state
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> dtau(robot_state.dtau_J.data()); 
      Eigen::Matrix<double, 6, 1> dF = jacobian * dtau; 
      Eigen::Map<const Eigen::Matrix<double, 6, 1>> F(robot_state.K_F_ext_hat_K.data());
      
        // Get current orientation for plots
      Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
      Eigen::Vector3d position(transform.translation());
      Eigen::Quaterniond rotation(transform.linear());
      Eigen::Matrix3d rot_matrix = rotation.toRotationMatrix();
        // Compute current rotation in angles
      double roll = std::atan2(rot_matrix(2, 1), rot_matrix(2, 2));
      double pitch = std::asin(-rot_matrix(2, 0));
      double yaw = std::atan2(rot_matrix(1, 0), rot_matrix(0, 0));

        // Compute trajectory between current and desired orientation
      // Error
      Eigen::Matrix<double, 6,1> error; error.setZero();
      error << F-F_d; 

      // integrate error using forward euler
      int_error += dt*error;

      // Calculate the necessary wrench, from Haddadin und Schindlbeck (nicht direkt aber da hab ich die Formel her)
      Eigen::Matrix<double, 6,1> h_c; h_c.setZero();

      h_c = K_p * error; //  No K_i part, so this is left out: + K_i * int_error 
      // h_c << 0, 0, 0, 0, 0, 0;
      // Calculate torque and map to an array
      std::array<double, 7> tau_d{};
      Eigen::VectorXd::Map(&tau_d[0], 7) = jacobian.transpose() * h_c.cwiseProduct(factor_filter); // 

      // Calculate positional error for tests
      double distance = (position_init-position).norm(); 

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

      // Map the position and error to an array
      std::array<double, 3> pos_array{}; 
      std::array<double, 6> error_array{}; 
      Eigen::VectorXd::Map(&pos_array[0], 3) = position;
      Eigen::VectorXd::Map(&error_array[0], 6) = error;
      // Add the current data to the array
      tau_data.push_back(tau_d);
      position_data.push_back(pos_array);
      rotation_data.push_back({(roll_init-roll)/M_PI*180, (pitch-pitch_init)/M_PI*180, (yaw-yaw_init)/M_PI*180});
      force_data.push_back(robot_state.K_F_ext_hat_K);
      error_data.push_back(error_array);

      // tau_d = {0,0,0,0,0,0,0};
      // Send desired tau to the robot
      return tau_d;

    };

    // start control loop
    robot.control(force_control_callback);
    // Write Data to .txt file
    writeDataToFile(tau_data);
    writeDataToFile(position_data);
    writeDataToFile(rotation_data);
    writeDataToFile(force_data);
    writeDataToFile(error_data);
  }
  // Catches Exceptions caused within the execution of a program (I think)
  catch (const franka::ControlException& e) {
    std::cout << e.what() << std::endl;
    writeDataToFile(tau_data);
    writeDataToFile(position_data);
    writeDataToFile(rotation_data);
    writeDataToFile(force_data);
    writeDataToFile(error_data);

    return -1;
  }
  // General exceptions
  catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }
  
}
