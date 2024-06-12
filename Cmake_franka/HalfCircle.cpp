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
    robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
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
    double time_max = 10;

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
            double y = abs(radius * -sin(time * 2*M_PI/time_max));
            double z = radius * cos(time * 2*M_PI/time_max) - radius; //
            // Calculate factor to smooth function
            double period_1 = (1 - std::cos(M_PI * time/(time_max/2) )) / 2.0;      // Sinusosidal function with 1 period during time_max
            double period_2 = (1 - cos(M_PI * time/(time_max/2/2) )) / 2.0;    // Sinusosidal function with 2 periods during time_max

                // Create new Translation matrix to try doing the same shit with eigen
            Eigen::Matrix4d pose_out, translation(Eigen::Matrix4d::Identity()), rotation(Eigen::Matrix4d::Identity());
            // Translation to control the movement (half circle)
            translation(1,3) = (translation(1,3) + y) * period_2; 
            translation(2,3) = (translation(2,3) + z) * period_1;
            // Rotation to control the gripper 
            double theta = -M_PI_2 * period_1; // periodical function that peaks at pi/2 at time_max/2
            Eigen::Vector3d axis(1,0,0); // rotation axis (here I want a rotation around the x-axis)
            Eigen::AngleAxisd rotation_vector(theta, axis); // rotation vector, needed in the next step
            rotation.block<3,3>(0,0) = rotation_vector.toRotationMatrix();  // put the rotation into the translation matrix
            // Compute the desired position
            pose_out = translation * initial_transformation * rotation; // 

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
/*
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
*/
        robot.control(cartesian_pose_callback);

    }
    catch (const franka::Exception& ex) {
        std::cout << ex.what() << std::endl;
    }

    return 0;
}