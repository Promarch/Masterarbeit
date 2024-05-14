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
  robot.setCollisionBehavior(
      {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
      {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
      {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
      {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});
  robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
  robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
}

int main(int argc, char** argv) {

  try {
    franka::Robot robot("192.168.1.11");
    setDefaultBehavior(robot);

    // Set collision behavior.
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
/*
    double time = 0.0;
    double time_max = 5.0;
    std::array<double, 7> start_position = robot.readOnce().q_d;
    std::array<double, 7> pos1 = {0.23, -0.75, 0.72, -2.63, 0, 1.86, 1.21};
    std::array<double, 7> deltaPos;
    std::array<double, 7> initial_position;

    // Substract the initial position from the second position
    std::transform(pos1.begin(), pos1.end(), start_position.begin(), deltaPos.begin(), std::minus<double>() );
*/


    std::array<double, 7> initial_position;
    double time = 0.0;
    double sampling_interval = 0.1;
    double next_sampling_time = sampling_interval;

    // Vector to store the sampled force/torque values
    std::vector<std::array<double, 6>> force_torque_data;
    
    robot.control([&initial_position, &time, &next_sampling_time, &force_torque_data](const franka::RobotState& robot_state,
                                             franka::Duration period) -> franka::JointPositions {
      time += period.toSec();
      if (time == 0.0) {
        initial_position = robot_state.q_d;
      }

    // Check if it's time to sample the force/torque values
      if (time >= next_sampling_time) {
        force_torque_data.push_back(robot_state.O_F_ext_hat_K);
        next_sampling_time += 0.1;
      }

      double delta_angle = M_PI / 8.0 * (1 - std::cos(M_PI / 2.5 * time));
      franka::JointPositions output = {{initial_position[0]+delta_angle, initial_position[1],
                                        initial_position[2], initial_position[3] + delta_angle,
                                        initial_position[4] + delta_angle, initial_position[5],
                                        initial_position[6] + delta_angle}};
      if (time >= 5.0) {
        std::cout << std::endl << "Finished motion, shutting down example" << std::endl;

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

          return franka::MotionFinished(output);
        }
      return output;
    });
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}