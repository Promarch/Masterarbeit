#include <iostream>
#include <iomanip>
#include <fstream>
#include <algorithm>
#include <cmath>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <franka/exception.h>
#include <franka/duration.h>
#include <franka/robot.h>
#include <franka/model.h>
#include <franka/robot_state.h>

void setDefaultBehavior(franka::Robot& robot) {
  // Set collision behavior.
  robot.setCollisionBehavior(
      {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
      {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
      {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
      {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
  robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
  robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
}

#define writeDataToFile(data) writeDataToFileImpl(data, #data)
template <std::size_t N>
void writeDataToFileImpl(const std::vector<std::array<double, N>>& data, const std::string& var_name) {
  std::string filename = var_name+".txt";
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



int main(int argc, char** argv) {

  std::vector<std::array<double, 7>> dq_d_values;
  std::vector<std::array<double, 6>> force_K_data, force_EE_data;

  try {
    franka::Robot robot("192.168.1.11");
    setDefaultBehavior(robot);
    robot.setEE({1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.245, 1.0});
    robot.setK({1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -0.245, 1.0});

    franka::Model model = robot.loadModel();
    
      // Variables
    // Positions
    std::array<double, 7> initial_position;
    std::array<double, 7> deltaPos;
    // Time 
    double time = 0.0;
    double sampling_interval = 0.1;
    double next_sampling_time = sampling_interval;
    double time_max = 2.5;
    
    auto JointPositionCallback = [&] (const franka::RobotState& robot_state, franka::Duration period) -> franka::JointPositions {

      time += period.toSec();
      if (time == 0.0) {
        initial_position = robot_state.q_d;
      }

      // Check if it's time to sample the force/torque values
      if (time >= next_sampling_time) {
        next_sampling_time += sampling_interval;
        std::cout << "Time: " << time << "\n"; 
      }

      double delta_angle = M_PI / 12.0 * (1 - std::cos(M_PI * time/2.5))/2;
      franka::JointPositions output = {{initial_position[0], initial_position[1],
                                        initial_position[2], initial_position[3],
                                        initial_position[4], initial_position[5],
                                        initial_position[6]}};


        // Calculations to get wrench at different locations
      // Get state variables
      std::array<double, 42> jacobian_array = model.bodyJacobian(franka::Frame::kEndEffector, robot_state);
      Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> tau_filter(robot_state.tau_ext_hat_filtered.data()); 
      // Calculate force at EE
      Eigen::Matrix<double, 6, 1> F_tau_filter = jacobian.transpose().completeOrthogonalDecomposition().pseudoInverse() * tau_filter; 

        // Convert to array and add to other values
      // Convert Eigen to array
      std::array<double, 6> F_tau_filter_array{}; 
      Eigen::VectorXd::Map(&F_tau_filter_array[0], 6) = F_tau_filter;


      force_K_data.push_back(robot_state.K_F_ext_hat_K);
      force_EE_data.push_back(F_tau_filter_array);

      if (time >= time_max) {
        std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
        return franka::MotionFinished(output);
      }
      
      return output;
    };

    robot.control(JointPositionCallback);
    writeDataToFile(force_K_data);
    writeDataToFile(force_EE_data);
  } 
  catch (const franka::Exception& e) {

    std::cout << e.what() << std::endl << "this message was displayed\n";
    return -1;
  }
    // Write the collected force/torque data to a text file

  return 0;
}