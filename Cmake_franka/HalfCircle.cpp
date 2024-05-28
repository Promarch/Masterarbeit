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

    // Randbedingungen und -variablen
    const double radius{1.0};
    double time = 0.0;
    double time_max = 5;

    try {
            //Set Up
        // connect to the robot
        franka::Robot robot("192.168.1.11");
        setDefaultBehavior(robot);
        // Load Model for Jacobian
        franka::Model model = robot.loadModel();

        // Callback function
        std::function<franka::JointVelocities(const franka::RobotState&, franka::Duration)> 
            JointVelocities_control_callback = [&](const franka::RobotState& robot_state, franka::Duration period) -> franka::JointVelocities {

            time += period.toSec();
            // Get state variables
            std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
            Eigen::Map<const Eigen::Matrix<double,6,7>> jacobian(jacobian_array.data());

            // Calculate y and z speeds
            double dy = radius * -M_PI/time_max * (cos(time * M_PI/time_max));
            double dz = radius * -M_PI/time_max * (sin(time * M_PI/time_max));

            // Calculate factor to smooth function
            double factor = (1 - cos(M_PI * time/(time_max/2) )) / 2.0;

            // Define workspace array
            Eigen::VectorXd wd(6);
            wd.setZero();
            wd(1) = dy;
            wd(2) = dz;
            
            // Compute desired Joint velocities
            Eigen::VectorXd JointVelocities(7);
            JointVelocities << jacobian.transpose() * wd * factor;

            std::array<double, 7> JointVelocities_array{};
            Eigen::VectorXd::Map(&JointVelocities_array[0], 7) = JointVelocities;
            franka::JointVelocities output = JointVelocities_array;
            

            if (time >= time_max) {
                std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
                return franka::MotionFinished(output);
            }

            return output;
        };

        robot.control(JointVelocities_control_callback);

    }
    catch (const franka::Exception& ex) {
        std::cout << ex.what() << std::endl;
    }

    return 0;
}

    /*
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
    */