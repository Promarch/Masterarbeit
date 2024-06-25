#include <iostream>
#include <iomanip>
#include <fstream>
#include <algorithm>
#include <cmath>
#include <vector>

#include <Eigen/Dense>
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

  // Declare variables to store the joint positions and forces
  std::vector<std::array<double, 7>> dq;
  std::vector<std::array<double, 7>> jointTorque_desired;
  std::vector<std::array<double, 7>> jointTorque_filtered;

    // Compliance parameters
  // Set Stiffness
  const double translational_stiffness{150.0};
  const double rotational_stiffness{10.0};
  Eigen::MatrixXd stiffness(6, 6);
  stiffness.setZero();
  stiffness.topLeftCorner(3, 3) << translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
  stiffness.bottomRightCorner(3, 3) << rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
  // Set Damping
  Eigen::MatrixXd damping(6, 6);
  damping.setZero();
  damping.topLeftCorner(3, 3) << 2.0 * sqrt(translational_stiffness) * Eigen::MatrixXd::Identity(3, 3);
  damping.bottomRightCorner(3, 3) << 2.0 * sqrt(rotational_stiffness) * Eigen::MatrixXd::Identity(3, 3);

  try {
    // connect to robot and set default values
    franka::Robot robot("192.168.1.11");
    setDefaultBehavior(robot);
    // Load the kinematics (jacobian) and dynamics (coriolis) model
    franka::Model model = robot.loadModel();

    // Read the initial position
    franka::RobotState initial_state = robot.readOnce();
    // Set equilibrium point as the initial position and convert variables to eigen
    Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
    Eigen::Vector3d position_d(initial_transform.translation());
    Eigen::Quaterniond orientation_d(initial_transform.linear());

    // Torque control loop
    std::function<franka::Torques(const franka::RobotState&, franka::Duration)> 
      impedance_control = [&](const franka::RobotState& robot_state, franka::Duration /*duration*/) -> franka::Torques {
      
      // get state variables
      std::array<double, 7> coriolis_array = model.coriolis(robot_state);
      std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, robot_state);

      // Convert to eigen
      Eigen::Map<const Eigen::Matrix<double, 7,1>> coriolis(coriolis_array.data());
      Eigen::Map<const Eigen::Matrix<double, 6,7>> jacobian(jacobian_array.data());
      Eigen::Map<const Eigen::Matrix<double, 7,1>> q(robot_state.q.data());
      Eigen::Map<const Eigen::Matrix<double, 7,1>> dq(robot_state.dq.data());
      Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
      Eigen::Vector3d position(transform.translation());
      Eigen::Quaterniond orientation(transform.linear());

        // Compute error to desired equilibrium pose
      // position error
      Eigen::Matrix<double, 6,1> error;
      error.head(3) << position - position_d;
      // Orientation error
      if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
        orientation.coeffs() << -orientation.coeffs();  // Negate the coefficient of the quatiernion for consistency with the desired position
      }
      Eigen::Quaterniond error_quaternion(orientation.inverse()*orientation_d);
      error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z(); 
      error.tail(3) << -transform.linear() * error.tail(3);   // transform to base frame

        // Compute the Joint Torques
      // Create Joint Variables
      Eigen::VectorXd tau_task(7), tau_d(7);
      // Calculate the spring-damper system
      tau_task << jacobian.transpose() * (-stiffness * error - damping * (jacobian * dq ));
      tau_d << tau_task + coriolis; 

      std::array<double, 7> tau_d_array{};

      
      };

  } 
  catch (const std::exception& ex) {
    // print exception
    std::cout << ex.what() << std::endl;
  }
  return 0;
}
