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

      // Initial orientation
    Eigen::Map<const Eigen::Matrix<double, 7,1>> q_init(initial_state.q.data());
    Eigen::Affine3d transform_init(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
    Eigen::Vector3d position_init(transform_init.translation());
    // Eigen::Quaterniond rotation_init(transform_init.linear());
    Eigen::Quaterniond rotation_init(0.0, 0.7071068, 0.7071068, 0.0);
    Eigen::Matrix3d rot_matrix(transform_init.rotation());
    
      // Desired Rotation, created with quaternions
    // Flexion (Rotation around y in local CoSy) 
    double angle_flexion = -2*M_PI/12;
    Eigen::Vector3d axis_flexion(1,0,0);
    Eigen::AngleAxisd angle_axis_flexion(angle_flexion, axis_flexion);
    Eigen::Quaterniond quaternion_flexion(angle_axis_flexion);
    // Varus-Valgus (Rotation around z in local CoSy) 
    double angle_varus = 2*M_PI/9*0;
    Eigen::Vector3d axis_varus(0,1,0);
    Eigen::AngleAxisd angle_axis_varus(angle_varus, axis_varus);
    Eigen::Quaterniond quaternion_varus(angle_axis_varus);
    // Combine the rotations
    Eigen::Quaterniond quaternion_combined = quaternion_varus * quaternion_flexion;
    // Translate the desired rotation into local coordinate system (rotation in EE-CoSy instead of base CoSy)
    Eigen::Quaterniond rot_quaternion_rel = rotation_init * quaternion_combined * rotation_init.inverse();
    // Add rotation to initial orientation
    Eigen::Quaterniond rot_d = rot_quaternion_rel * rotation_init; 
    Eigen::Quaterniond error_quat = rotation_init.inverse() * rot_d;
    Eigen::Vector3d error(error_quat.x(), error_quat.y(), error_quat.z());

    std::cout << "Desired Rotation in local CoSy: " << quaternion_combined;
    std::cout<<"\nDesired Rotation in base  CoSy: " << rot_quaternion_rel;
    std::cout<<"\nDesired Rotatio added to local: " << rot_d;
    std::cout<<"\nOther Calculation             : " << rotation_init * quaternion_flexion * quaternion_varus;
    std::cout<<"\nDifference quaternion         : " << error_quat;
    std::cout<<"\nDifference quat in base Cosy  : " << ((-rotation_init.toRotationMatrix() * error)*180/M_PI).transpose() << std::endl;
      // Other variables
    //Jacobian
    std::array<double, 42> zero_jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, initial_state);
    std::array<double, 42> body_jacobian_array = model.bodyJacobian(franka::Frame::kEndEffector, initial_state);
    Eigen::Map<const Eigen::Matrix<double, 6,7>> zeroJacobian(zero_jacobian_array.data());
    Eigen::Map<const Eigen::Matrix<double, 6,7>> bodyJacobian(body_jacobian_array.data());
    // Tau and gravity
    Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity(model.gravity(initial_state).data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> tau(initial_state.tau_J.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_ext(initial_state.tau_ext_hat_filtered.data());
    // Joint position
    Eigen::Map<Eigen::Matrix<double, 7, 1>> q(initial_state.q.data());
    // Mass Matrix
    Eigen::Map<const Eigen::Matrix<double, 7,7>> mass(model.mass(initial_state).data());

      // Calculate what happens when you transform the CoSy of the jacobian
    Eigen::MatrixXd adjoint = Eigen::MatrixXd::Identity(6, 6);adjoint.setZero();
    adjoint.topLeftCorner(3,3) = rot_matrix; 
    adjoint.bottomRightCorner(3,3) = rot_matrix; 
    Eigen::Matrix3d pos_skew; pos_skew << 0, -position_init(2), position_init(1), position_init(2), 0, -position_init(0), -position_init(1), position_init(0), 0;
    adjoint.bottomLeftCorner(3,3) = pos_skew * rot_matrix;
    Eigen::MatrixXd bodyJacobian_from_zero = adjoint * zeroJacobian;

      // Print stuff
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "q: " << q.transpose();
    std::cout << "\nO_T_EE: \n" << transform_init.matrix() << std::endl; 

    // std::cout << "\nMass Matrix: \n" << mass;
    // std::cout << "\nAdjoint: \n" << adjoint; 
    // std::cout << "\nZero Jacobian: \n" << zeroJacobian; 
    // std::cout << "\nBody Jacobian: \n" << bodyJacobian;
    // std::cout << "\nbodyJacobian_from_zero: \n" << bodyJacobian_from_zero << std::endl;

    return 0;
  }

  catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }
}