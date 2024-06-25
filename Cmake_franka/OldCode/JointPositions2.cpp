#include <iostream>
#include <iomanip>
#include <fstream>
#include <algorithm>
#include <cmath>
#include <vector>

#include <franka/exception.h>
#include <franka/duration.h>
#include <franka/robot.h>
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

int main(int argc, char** argv) {

  std::vector<std::array<double, 7>> dq_d_values;
  std::vector<std::array<double, 6>> force_torque_data;

  try {
    franka::Robot robot("192.168.1.11");
    setDefaultBehavior(robot);

      // Variables
    // Positions
    std::array<double, 7> initial_position;
    std::array<double, 7> deltaPos;
    // Time 
    double time = 0.0;
    double sampling_interval = 0.1;
    double next_sampling_time = sampling_interval;
    
    auto JointPositionCallback = [&initial_position, &time, &next_sampling_time, &force_torque_data, &dq_d_values, &deltaPos]
                                (const franka::RobotState& robot_state, franka::Duration period) -> franka::JointPositions {
      time += period.toSec();
      if (time == 0.0) {
        initial_position = robot_state.q_d;
      }

      // Check if it's time to sample the force/torque values
      if (time >= next_sampling_time) {
        force_torque_data.push_back(robot_state.O_F_ext_hat_K);
        next_sampling_time += 0.1;
      }

      double delta_angle = M_PI / 12.0 * (1 - std::cos(M_PI * time/2.5))/2;
      franka::JointPositions output = {{initial_position[0], initial_position[1]-delta_angle,
                                        initial_position[2], initial_position[3],
                                        initial_position[4], initial_position[5],
                                        initial_position[6]}};

      if (time >= 5.0) {
        std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
        return franka::MotionFinished(output);
      }
      
      return output;
    };

    robot.control(JointPositionCallback);

  } 
  catch (const franka::Exception& e) {

    std::cout << e.what() << std::endl << "this message was displayed\n";
    return -1;
  }
    // Write the collected force/torque data to a text file
  std::ofstream data_file("force_torque_data.txt");
  if (data_file.is_open()) {
    data_file << std::fixed << std::setprecision(2);
    for (const auto& sample : force_torque_data) {
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
  return 0;
}