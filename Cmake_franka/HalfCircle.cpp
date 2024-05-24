#include <iostream>
#include <iomanip>
#include <fstream>
#include <algorithm>
#include <cmath>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

void setDefaultBehavior(franka::Robot& robot) {
  // Set collision behavior.
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
    robot.setJointImpedance({{3000, 3000, 3000, 1000, 3000, 3000, 3000}});
    robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
}

int main () {

    // Variables to store force/torque data
    std::array<double, 7> force_print; 
    std::array<double, 7> torque_print; 
    
        // Compliance parameters
    const double translational_stiffness{150.0};
    const double rotational_stiffness{10.0};
    Eigen::MatrixXd stiffness(6, 6), damping(6, 6);
    // Stiffness Matrix
    stiffness.setZero();
    stiffness.topLeftCorner(3, 3) << translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
    stiffness.bottomRightCorner(3, 3) << rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
    // Damping matrix
    damping.setZero();
    damping.topLeftCorner(3, 3) << 2.0 * sqrt(translational_stiffness) * Eigen::MatrixXd::Identity(3, 3);
    damping.bottomRightCorner(3, 3) << 2.0* sqrt(rotational_stiffness) * Eigen::MatrixXd::Identity(3, 3);

    // Geometric variables
    const double radius{10.0};
    double time = 0.0;

    try {
            //Set Up
        // connect to the robot
        franka::Robot robot("192.168.1.11");
        setDefaultBehavior(robot);
        // Load Model 
        franka::Model model = robot.loadModel();

        // Get starting position
        franka::RobotState initial_state = robot.readOnce();
        Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
        Eigen::Vector3d init_pos(initial_transform.translation());
        Eigen::Quaterniond init_rot(initial_transform.linear());

        // Callback function
        std::function<franka::Torques(const franka::RobotState&, franka::Duration)> 
            impedance_control_callback = [&](const franka::RobotState& robot_state, franka::Duration period) -> franka::Torques {

            time += period.toSec();
            // Get state variables
            std::array<double, 7> coriolis_array = model.coriolis(robot_state);
            std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, robot_state);

            // Convert to eigen
            Eigen::Map<const Eigen::Matrix<double,7,1>> coriolis(coriolis_array.data());
            Eigen::Map<const Eigen::Matrix<double,6,7>> jacobian(jacobian_array.data());
            Eigen::Map<const Eigen::Matrix<double,7,1>> q(robot_state.q.data());
            Eigen::Map<const Eigen::Matrix<double,7,1>> dq(robot_state.dq.data());
            Eigen::Affine3d transform_matrix(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
            Eigen::Vector3d position(transform_matrix.translation());
            Eigen::Quaterniond orientation(transform_matrix.linear());

                // Computation of the desired pose
            Eigen::Matrix<double, 6,1> des_pos;
            des_pos.setZero();
            des_pos[0] = radius * cos(M_PI_2 + time * M_PI/5);
            des_pos[1] = radius * sin(M_PI_2 + time * M_PI/5) - radius;
        };

    }
    catch (const franka::Exception& ex) {
        std::cout << ex.what() << std::endl;
    }

    return 0;
}