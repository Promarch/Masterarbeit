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
  std::string filename = "data_grav/" + var_name + "_" + getCurrentDateTime() + ".txt";
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
  std::vector<std::array<double, 7>> tau_grav_data, tau_filter_data; 
  std::vector<std::array<double, 6>> F_tau_grav_data, F_tau_filter_data, F_ext_data, F_grav_data; 

  try {
    // Set Up basic robot function
    franka::Robot robot("192.168.1.11");
    // Set new end-effector
    // robot.setEE({1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.240, 1.0});
    robot.setEE({1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0});
    setDefaultBehavior(robot);
    franka::Model model = robot.loadModel();
    franka::RobotState initial_state = robot.readOnce();

    // Variables to control loop time
    double time{0};
    double time_max{5};
    double sampling_interval = 0.25; // Interval at which the console outputs the current error
    double next_sampling_time = 0;  // Needed for the sampling interval


    auto force_control_callback = [&] (const franka::RobotState& robot_state, franka::Duration period) -> franka::Torques {
      
      time += period.toSec();

        // Get state variables
      // Jacobian
      std::array<double, 42> jacobian_array = model.bodyJacobian(franka::Frame::kEndEffector, robot_state);
      Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
      // Tau with gravity
      std::array<double, 7> tau_j_array = robot_state.tau_J; 
      Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J(tau_j_array.data()); 
      // Gravity on each joint
      std::array<double, 7> gravity_array = model.gravity(robot_state);
      Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());
      // Tau filtered
      std::array<double, 7> tau_filter_array = robot_state.tau_ext_hat_filtered;
      Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_filter(tau_filter_array.data());
      
        // Compute stuff
      Eigen::Matrix<double, 7, 1> tau_grav = tau_J-gravity;
      Eigen::Matrix<double, 6, 1> F_tau_filter = jacobian.transpose().completeOrthogonalDecomposition().pseudoInverse() * tau_filter; 
      Eigen::Matrix<double, 6, 1> F_tau_grav = jacobian * tau_grav; 
      Eigen::Matrix<double, 6, 1> F_grav = jacobian * gravity; 

        // Map the Eigen vectors to an array
      std::array<double, 7> tau_grav_array{}; 
      std::array<double, 6> F_tau_filter_array{}, F_tau_grav_array{}, F_grav_array{}; 
      Eigen::VectorXd::Map(&tau_grav_array[0], 7) = tau_grav;
      Eigen::VectorXd::Map(&F_tau_filter_array[0], 6) = F_tau_filter;
      Eigen::VectorXd::Map(&F_tau_grav_array[0], 6) = F_tau_grav;
      Eigen::VectorXd::Map(&F_grav_array[0], 6) = F_grav;

        // Add current measurements to the vector      
      tau_grav_data.push_back(tau_grav_array);
      tau_filter_data.push_back(tau_filter_array);
      F_tau_grav_data.push_back(F_tau_grav_array);
      F_tau_filter_data.push_back(F_tau_filter_array);
      F_ext_data.push_back(robot_state.K_F_ext_hat_K);
      F_grav_data.push_back(F_grav_array);

      // Print current wrench, time, and absolute positional error
      if (time >= next_sampling_time) {
        std::cout << std::fixed << std::setprecision(3);
        std::cout << "Time: " << time << "\n"; 
        next_sampling_time += sampling_interval;
      }

        // Send torque = 0 to ensure no movement
      std::array<double, 7> tau_d = {0,0,0,0,0,0,0};
      if (time >= time_max) {
        franka::Torques output = tau_d;
        return franka::MotionFinished(output);
      }
      return tau_d;
    };

    // start control loop
    robot.control(force_control_callback);
      // Write Data to .txt file
    writeDataToFile(tau_grav_data);
    writeDataToFile(tau_filter_data);
    writeDataToFile(F_tau_grav_data);
    writeDataToFile(F_tau_filter_data);
    writeDataToFile(F_ext_data);
    writeDataToFile(F_grav_data);

  }
  // Catches Exceptions caused within the execution of a program (I think)
  catch (const franka::ControlException& e) {
    std::cout << e.what() << std::endl;
    writeDataToFile(tau_grav_data);
    writeDataToFile(tau_filter_data);
    writeDataToFile(F_tau_grav_data);
    writeDataToFile(F_tau_filter_data);
    writeDataToFile(F_ext_data);
    writeDataToFile(F_grav_data);

    return -1;
  }
  // General exceptions
  catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }
  
}
