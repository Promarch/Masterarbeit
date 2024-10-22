#include <iostream>
#include <iomanip>
#include <fstream>

#include <array>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

void setDefaultBehavior(franka::Robot &robot) {

    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
    robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
    robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
};


int main() {
  try {
    // Set Up basic robot function
    franka::Robot robot("192.168.1.11");
    setDefaultBehavior(robot);
    franka::Model model = robot.loadModel();
    franka::RobotState initial_state = robot.readOnce();
    robot.setEE({1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0});

      // Initial position
    // Eigen::Affine3d O_T_EE(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
    // Eigen::Affine3d F_T_EE(Eigen::Matrix4d::Map(initial_state.F_T_EE.data()));
    // Eigen::Affine3d initial_transform(O_T_EE * F_T_EE.inverse());
    Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
    Eigen::Vector3d pos_d(initial_transform.translation());
    Eigen::Quaterniond orientation_d(initial_transform.linear());
    Eigen::Matrix3d rot_matrix(initial_transform.rotation());

      // Other variables
    //Jacobian
    std::array<double, 42> body_jacobian_array = model.bodyJacobian(franka::Frame::kEndEffector, initial_state);
    std::array<double, 42> zero_jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, initial_state);
    Eigen::Map<const Eigen::Matrix<double, 6,7>> bodyJacobian(body_jacobian_array.data());
    Eigen::Map<const Eigen::Matrix<double, 6,7>> zeroJacobian(zero_jacobian_array.data());
    // Tau and gravity
    Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity(model.gravity(initial_state).data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> tau(initial_state.tau_J.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_ext(initial_state.tau_ext_hat_filtered.data());

      // Calculate what happens when you transform the CoSy of the jacobian
    Eigen::MatrixXd adjoint = Eigen::MatrixXd::Identity(6, 6);adjoint.setZero();
    adjoint.topLeftCorner(3,3) = rot_matrix; 
    adjoint.bottomRightCorner(3,3) = rot_matrix; 
    Eigen::Matrix3d pos_skew; pos_skew << 0, -pos_d(2), pos_d(1), pos_d(2), 0, -pos_d(0), -pos_d(1), pos_d(0), 0;
    adjoint.bottomLeftCorner(3,3) = pos_skew * rot_matrix;
    Eigen::MatrixXd bodyJacobian_from_zero = adjoint * zeroJacobian;

      // Print stuff
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Adjoint: \n" << adjoint; 
    std::cout << "\nZero Jacobian: \n" << zeroJacobian; 
    std::cout << "\nBody Jacobian: \n" << bodyJacobian;
    std::cout << "\nbodyJacobian_from_zero: \n" << bodyJacobian_from_zero << std::endl;

    return 0;
  }

  catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }
}