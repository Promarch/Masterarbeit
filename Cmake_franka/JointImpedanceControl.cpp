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

  // variables to store force/Joint Torque
  std::vector<std::array<double, 7>> tau_ext_print;
  std::vector<std::array<double, 7>> tau_d_print;

    // Compliance parameters
  const std::array<double, 7> k_gains_array{{600.0, 600.0, 600.0, 600.0, 600.0, 600.0, 600.0}};
  const std::array<double, 7> d_gains_array{{50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0}};
  // Map to Eigen
  Eigen::Map<const Eigen::ArrayXd> k_gains(k_gains_array.data(), k_gains_array.size());
  Eigen::Map<const Eigen::ArrayXd> d_gains(d_gains_array.data(), d_gains_array.size());

  try {
    // connect to robot
    franka::Robot robot("192.168.1.11");
    setDefaultBehavior(robot);

    // Load the kinematics and dynamics model
    franka::Model model = robot.loadModel();

    // Set starting variables
    double time{0.0};
    double timer{0.0};
    double sampling_interval = 0.1;
    double next_sampling_time = sampling_interval;
    franka::RobotState initial_state = robot.readOnce();
    Eigen::Map<Eigen::VectorXd> initial_position(initial_state.q.data(), initial_state.q.size());


    // Define callback function to set the desired joint position
    auto joint_pose_callback = [&] (const franka::RobotState& robot_state, franka::Duration period) -> franka::JointPositions {
      
      time += period.toSec();
      double delta_angle = M_PI / 18.0 * (1 - std::cos(M_PI * time/2.5))/2;
      franka::JointPositions output = {{initial_position[0], initial_position[1],
                                        initial_position[2], initial_position[3]+delta_angle,
                                        initial_position[4], initial_position[5],
                                        initial_position[6]}};

      if (time >= 5.0) {
        std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
        return franka::MotionFinished(output);
      }
      return output;

    };

    // Define callback function for the joint torque loop
    std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
      impedance_control_callback = [&model, &timer, &sampling_interval, &next_sampling_time, &tau_ext_print, &tau_d_print, k_gains, d_gains]
          (const franka::RobotState& robot_state, franka::Duration period) -> franka::Torques {

        // Check if it's time to sample the force/torque values
        timer += period.toSec();
        if (timer >= next_sampling_time) {
          tau_ext_print.push_back(robot_state.tau_ext_hat_filtered);
          tau_d_print.push_back(robot_state.tau_J_d);
          next_sampling_time += 0.1;
        }
        
        // Get state variables
        std::array<double, 7> coriolis_array = model.coriolis(robot_state);
        Eigen::Map<const Eigen::ArrayXd> coriolis(coriolis_array.data(), coriolis_array.size());

        // Convert to eigen
        Eigen::Map<const Eigen::ArrayXd> q(robot_state.q.data(), robot_state.q.size()); 
        Eigen::Map<const Eigen::ArrayXd> q_d(robot_state.q_d.data(), robot_state.q_d.size());
        Eigen::Map<const Eigen::ArrayXd> dq(robot_state.dq.data(), robot_state.dq.size());

        // Compute Torque
        Eigen::VectorXd tau_d_calculated(7);
        tau_d_calculated << k_gains * (q_d - q) - d_gains * dq + coriolis;

        // Convert to array
        std::array<double, 7> tau_d_array{};
        Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d_calculated;

        double factor = (1 - std::cos(M_PI * timer/2.5 )) / 2.0; 
        tau_d_array[3] = factor * 6;

        return tau_d_array;

      };

    // start real-time control loop
    robot.control(impedance_control_callback, joint_pose_callback);

  } catch (const std::exception& ex) {
    // print exception
    std::cout << ex.what() << std::endl;
  }


  // Write the collected force/torque data to a text file
  std::ofstream data_file("tau_ext_print.txt");
  if (data_file.is_open()) {
    data_file << std::fixed << std::setprecision(2);
    for (const auto& sample : tau_ext_print) {
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

  std::ofstream measured_data_file("tau_d_print.txt");
  if (measured_data_file.is_open()) {
    measured_data_file << std::fixed << std::setprecision(2);
    for (const auto& sample : tau_d_print) {
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

  return 0;
}
