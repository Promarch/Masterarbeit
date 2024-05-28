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

    // Set gains for the joint impedance control.
    // Stiffness
    const std::array<double, 7> k_gains = {{600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0}};
    // Damping
    const std::array<double, 7> d_gains = {{50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0}};

    // Randbedingungen und -variablen
    const double radius{0.1};
    double time = 0.0;
    double time_max = 5;

    try {
            //Set Up
        // connect to the robot
        franka::Robot robot("192.168.1.11");
        setDefaultBehavior(robot);
        // Load Model for Jacobian
        franka::Model model = robot.loadModel();

        // Starting variables
        franka::RobotState initial_state = robot.readOnce();
        std::array<double, 16> initial_pose = initial_state.O_T_EE; 
        // Convert to Eigen
        Eigen::Map<Eigen::Matrix4d> initial_transformation(initial_state.O_T_EE.data());

        auto cartesian_pose_callback = [&] (const franka::RobotState& robot_state, franka::Duration period) -> franka::CartesianPose {

            time += period.toSec();

            // set y and z positions
            double y = radius * -sin(time * M_PI/time_max);
            double z = radius * cos(time * M_PI/time_max) - radius;
            // Calculate factor to smooth function
            double factor = (1 - cos(M_PI * time/(time_max/2) )) / 2.0; // Sinusosidal function that goes from 0 to 0 during time_max

            // Normal way
            franka::CartesianPose pose_desired = initial_pose; 
            pose_desired.O_T_EE[13] += y * factor;
//            pose_desired.O_T_EE[14] += z * factor;

            // Create new Translation matrix to try doing the same shit with eigen
            Eigen::Matrix4d pose_out, transformation(Eigen::Matrix4d::Identity());
            transformation(1,3) = transformation(1,3) + y * factor; 
            transformation(2,3) = transformation(2,3) + z * factor; 
            pose_out = initial_transformation * transformation; 
            // Convert to array
            std::array<double, 16> pose_out_array; 
            Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::ColMajor>>(pose_out_array.data()) = pose_out;

            if (time >= time_max) {
                std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
                return franka::MotionFinished(pose_out_array);
            }

            return pose_out_array;
        };

        // Callback function for force
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
            double factor = (1 - cos(M_PI * time/(time_max/2) )) / 2.0; // Sinusosidal function that goes from 0 to 0 during time_max

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
            
            };

        robot.control(cartesian_pose_callback);

    }
    catch (const franka::Exception& ex) {
        std::cout << ex.what() << std::endl;
    }

    return 0;
}