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
  std::vector<std::array<double, 7>> dq;
  std::vector<std::array<double, 7>> jointTorque_desired;
  std::vector<std::array<double, 7>> jointTorque_filtered;

    // Compliance parameters
  const std::array<double, 7> k_gains_array{600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0};
  const std::array<double, 7> d_gains_array{50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0};
  // Map to Eigen
  Eigen::Map<const Eigen::VectorXd> k_gains(k_gains_array.data(), k_gains_array.size());
  Eigen::Map<const Eigen::VectorXd> d_gains(d_gains_array.data(), d_gains_array.size());

  try {
    // connect to robot
    franka::Robot robot("192.168.1.11");
    setDefaultBehavior(robot);

    // Load the kinematics and dynamics model
    franka::Model model = robot.loadModel();

    // Set starting variables
    double time{0.0};
    franka::RobotState initial_state = robot.readOnce();
    Eigen::Map<Eigen::VectorXd> initial_position(initial_state.q.data(), initial_state.q.size());


    // Define callback function to set the desired joint position
    auto joint_pose_callback = [&] (const franka::RobotState& robot_state, franka::Duration period) -> franka::JointPositions {
      
      
      time += period.toSec();
      double delta_angle = M_PI / 10.0 * (1 - std::cos(M_PI * time/2.5))/2;
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
      impedance_control_callback = [&model, k_gains, d_gains](const franka::RobotState& robor_state, franka::Duration /*period*/) -> franka::Torques {

        // Get state variables
        std::array<double, 7> coriolis_array = model.coriolis(robor_state);
        Eigen::Map<const Eigen::VectorXd> coriolis(coriolis_array.data(), coriolis_array.size());

      };

    // start real-time control loop
    robot.control(joint_pose_callback);

  } catch (const std::exception& ex) {
    // print exception
    std::cout << ex.what() << std::endl;
  }

/*
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
*/
  return 0;
}
