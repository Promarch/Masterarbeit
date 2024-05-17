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

// Helper function to linearly interpolate between two positions
std::array<double, 7> interpolate(const std::array<double, 7>& start, const std::array<double, 7>& end, double t) {
  std::array<double, 7> result;
  for (size_t i = 0; i < start.size(); ++i) {
    result[i] = start[i] + t * (end[i] - start[i]);
  }
  return result;
}

int main(int argc, char** argv) {

  std::vector<std::array<double, 7>> dq_d_values;

  try {
    franka::Robot robot("192.168.1.11");
    setDefaultBehavior(robot);

      // Variables
    // Positions
    std::array<double, 7> initial_position;
    std::array<double, 7> start_position = robot.readOnce().q_d;
    std::array<double, 7> pos1 = {0.23, -0.75, 0.72, -2.63, 0, 1.86, 1.21};
    std::array<double, 7> pos2 = {-0.32, -0.23, -0.33, -2.68, 0.09, 2.27, 0.43};
    std::array<double, 7> deltaPos;
    // Substract the initial position from the second position
    std::transform(pos1.begin(), pos1.end(), start_position.begin(), deltaPos.begin(), std::minus<double>() );
    // Time 
    double time = 0.0;
    double sampling_interval = 0.1;
    double next_sampling_time = sampling_interval;
    // Sampled force/torque values
    std::vector<std::array<double, 6>> force_torque_data;
    
    
    robot.control([&initial_position, &pos1, &pos2, &time, &next_sampling_time, &force_torque_data, &dq_d_values, &deltaPos]
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

      std::array<double, 7> target_position;
      double factor = (1 - std::cos(M_PI * time/2.5 )) / 2.0;
      for (size_t i = 0; i<7; i++) {
        target_position[i] = initial_position[i] + deltaPos[i]*factor;
      }

      franka::JointPositions output = target_position;
      dq_d_values.push_back(robot_state.dq_d);

/*
      double delta_angle = M_PI / 8.0 * (1 - std::cos(M_PI / 2.5 * time));

      std::array<double, 7> target_position = initial_position;
      target_position[0] = initial_position[0]+delta_angle;
      dq_d_values.push_back(robot_state.dq_d);
      franka::JointPositions output = target_position; 

      // Interpolate the positions
      double t = time/2.0;
      target_position = interpolate(initial_position, pos1, t);

      franka::JointPositions output = target_position;
      dq_d_values.push_back(robot_state.dq_d);
*/

      if (time >= 5.0) {
        std::cout << std::endl << "Finished motion, shutting down example" << std::endl;

        return franka::MotionFinished(output);
        }
      return output;
    });
  } 
  catch (const franka::Exception& e) {

    std::cout << e.what() << std::endl << "this message was displayed\n";
    return -1;
  }
      // Write the collected force/torque data to a text file
    std::ofstream data_file("dq_d.txt");
    if (data_file.is_open()) {
      data_file << std::fixed << std::setprecision(2);
      for (const auto& sample : dq_d_values) {
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