// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <array>
#include <iomanip>
#include <fstream>
#include <iostream>
#include <Eigen/Core>
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
    robot.setJointImpedance({{3000, 3000, 3000, 3000, 3000, 3000, 3000}});
    robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
};

int main() {

  // parameters
  double desired_mass{0.0};
  constexpr double target_mass{1.0};    // NOLINT(readability-identifier-naming)
  constexpr double k_p{1.0};            // NOLINT(readability-identifier-naming)
  constexpr double k_i{2.0};            // NOLINT(readability-identifier-naming)
  constexpr double filter_gain{0.0005};  // NOLINT(readability-identifier-naming)
  try {
    // Set Up basic robot function
    franka::Robot robot("192.168.1.11");
    // Set new end-effector
    setDefaultBehavior(robot);
    franka::Model model = robot.loadModel();

    // set collision behavior
    robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});
    franka::RobotState initial_state = robot.readOnce();
    Eigen::VectorXd initial_tau_ext(7), tau_error_integral(7);
    // Bias torque sensor
    std::array<double, 7> gravity_array = model.gravity(initial_state);
    Eigen::Map<Eigen::Matrix<double, 7, 1>> initial_tau_measured(initial_state.tau_J.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> initial_gravity(gravity_array.data());
    initial_tau_ext = initial_tau_measured - initial_gravity;
    // init integrator
    tau_error_integral.setZero();
    
    // Debug stuff
    double sampling_interval = 0.1; // Time for first sampling interval
    double next_sampling_time = 0;  // Interval at which the debug loop is called
    double time_max = 8;

    // define callback for the torque control loop
    Eigen::Vector3d initial_position;
    double time = 0.0;
    auto get_position = [](const franka::RobotState& robot_state) {
      return Eigen::Vector3d(robot_state.O_T_EE[12], robot_state.O_T_EE[13],
                             robot_state.O_T_EE[14]);
    };
    auto force_control_callback = [&](const franka::RobotState& robot_state,
                                      franka::Duration period) -> franka::Torques {
      time += period.toSec();
      if (time == 0.0) {
        initial_position = get_position(robot_state);
      }
      if (time > 0 && (get_position(robot_state) - initial_position).norm() > 0.01) {
        throw std::runtime_error("Aborting; too far away from starting pose!");
      }
      // get state variables
      std::array<double, 42> jacobian_array =
          model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
      Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> tau_measured(robot_state.tau_J.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());
      Eigen::Map<const Eigen::Matrix<double, 6, 1>> F_ext(robot_state.K_F_ext_hat_K.data());
      Eigen::VectorXd tau_d(7), desired_force_torque(6), tau_cmd(7), tau_ext(7);
      desired_force_torque.setZero();
      desired_force_torque(2) = desired_mass * -9.81;
      tau_ext << tau_measured - gravity - initial_tau_ext;
      tau_d << jacobian.transpose() * desired_force_torque;
      tau_error_integral += period.toSec() * (tau_d - tau_ext);
      // FF + PI control
      tau_cmd << tau_d + k_p * (tau_d - tau_ext) + k_i * tau_error_integral;
      // Smoothly update the mass to reach the desired target value
      desired_mass = filter_gain * target_mass + (1 - filter_gain) * desired_mass;
      std::array<double, 7> tau_d_array{};
      Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_cmd;

      // For debug: Print current wrench, time, and absolute positional error every "sampling_interval"
      if (time >= next_sampling_time) {
        std::cout << std::fixed << std::setprecision(3);
        std::cout << "Desired Force:" << desired_force_torque(2) << ", measured Force: \n" << F_ext << "\n";
        next_sampling_time += sampling_interval;
      }

      if (time >= time_max) {
        std::cout << "\n\nFinished motion, shutting down example" << std::endl;
        franka::Torques tau_finished = tau_d_array; 
        return franka::MotionFinished(tau_finished);
      }

      return tau_d_array;
    };
    std::cout << "WARNING: Make sure sure that no endeffector is mounted and that the robot's last "
                 "joint is "
                 "in contact with a horizontal rigid surface before starting. Keep in mind that "
                 "collision thresholds are set to high values."
              << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    // start real-time control loop
    robot.control(force_control_callback);
  } catch (const std::exception& ex) {
    // print exception
    std::cout << ex.what() << std::endl;
  }
  return 0;
}