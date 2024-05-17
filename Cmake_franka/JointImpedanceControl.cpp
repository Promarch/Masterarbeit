#include <iostream>
#include <iomanip>
#include <fstream>
#include <algorithm>
#include <cmath>
#include <vector>

#include <Eigen/Core>
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

void setDefaultBehavior(franka::Robot& robot) {
  // Set collision behavior.
  robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                              {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                              {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                              {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});
  robot.setJointImpedance({{3000, 3000, 3000, 1000, 3000, 3000, 3000}});
  robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
}

int main(int argc, char** argv) {

  std::vector<std::array<double, 7>> dq;
  std::vector<std::array<double, 7>> jointTorque_desired;
  std::vector<std::array<double, 7>> jointTorque_filtered;

  try {
    // connect to robot
    franka::Robot robot("192.168.1.11");
    setDefaultBehavior(robot);

      // Variables
    // Positions
    std::array<double, 7> initial_position;
    std::array<double, 7> torque = {0,0,0,0,0,0,0}; 
    
    // Time 
    double time = 0.0;
    double sampling_interval = 0.05;
    double next_sampling_time = sampling_interval;
    // Debug stuff
    std::array<double, 7> help = robot.readOnce().tau_ext_hat_filtered; 


    auto force_control_callback = [&](const franka::RobotState& robot_state,
                                      franka::Duration period) -> franka::Torques {
      time += period.toSec();
      if (time == 0.0) {
        initial_position = robot_state.q_d;
      }

      // Take a sample of the measured torques
      if (time >= next_sampling_time) {
        jointTorque_desired.push_back(robot_state.tau_J_d);
        dq.push_back(robot_state.dq);
        jointTorque_filtered.push_back(robot_state.tau_ext_hat_filtered);
        next_sampling_time += 0.05;
      }

      double factor = (1 - std::cos(M_PI * time/1.5))/2;
      for (size_t i =0; i<7; i++) {
        torque[i] = 1.01*(-robot_state.tau_ext_hat_filtered[i]);
//        std::cout << std::endl << "pos i = " << i << std::endl;
      }
      torque[3] = factor * 3;



      franka::Torques output = torque; 

      if (time >= 3.0) {
        std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
        return franka::MotionFinished(output);
      }
      
      return torque; 

    };

    // start real-time control loop
    robot.control(force_control_callback);

  } catch (const std::exception& ex) {
    // print exception
    std::cout << ex.what() << std::endl;
  }

  // Write the collected force/torque data to a text file
  std::ofstream data_file("jointTorque_desired.txt");
  if (data_file.is_open()) {
    data_file << std::fixed << std::setprecision(2);
    for (const auto& sample : jointTorque_desired) {
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

  std::ofstream measured_data_file("dq.txt");
  if (measured_data_file.is_open()) {
    measured_data_file << std::fixed << std::setprecision(2);
    for (const auto& sample : dq) {
      for (size_t i = 0; i < sample.size(); ++i) {
        measured_data_file << sample[i];
        if (i < sample.size() - 1) {
          measured_data_file << ", ";
        }
      }
      measured_data_file << "\n";
    }
    measured_data_file.close();
  } else {
    std::cerr << "Unable to open file for writing" << std::endl;
  }

  std::ofstream filtered_data_file("jointTorque_filtered.txt");
  if (filtered_data_file.is_open()) {
    filtered_data_file << std::fixed << std::setprecision(2);
    for (const auto& sample : jointTorque_filtered) {
      for (size_t i = 0; i < sample.size(); ++i) {
        filtered_data_file << sample[i];
        if (i < sample.size() - 1) {
          filtered_data_file << ", ";
        }
      }
      filtered_data_file << "\n";
    }
    filtered_data_file.close();
  } else {
    std::cerr << "Unable to open file for writing" << std::endl;
  }

  return 0;
}
