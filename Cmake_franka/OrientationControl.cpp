#include <array>
#include <iostream>

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

int main(){
    try {
        // Set Up basic robot function
        franka::Robot robot("192.168.1.11");
        setDefaultBehavior(robot);
        franka::Model model = robot.loadModel(); 

        // Set proportional factors
        const double stiffness{150.0};
        const double damping{10.0};
        Eigen::MatrixXd K_p = Eigen::MatrixXd::Identity(6, 6);
        K_p.topLeftCorner(3,3) *= stiffness; 
        K_p.bottomRightCorner(3,3) *= damping; 

            // Variables 
        // Variables for the loop
        double time = 0.0;
        double time_max = 4.0;
            // Variables for the trajectory
        // Initial state
        franka::RobotState initial_state = robot.readOnce();
        Eigen::Affine3d transform_init(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
        Eigen::Quaterniond orientation_init(transform_init.linear());
        // rotation of 45°
        double angle = M_PI_4;
        Eigen::Quaterniond angle_quat(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitY()));
        // add the rotation to the initial orientation
        Eigen::VectorXd w_d(6);
        w_d.setZero();
        Eigen::Quaterniond orientation_d = angle_quat * orientation_init;
        w_d.tail(3) << orientation_d.x(), orientation_d.y(), orientation_d.z();

        // Control Loop
        auto force_control_callback = [&] (const franka::RobotState& robot_state, franka::Duration period) -> franka::Torques {

            time += period.toSec();

            // Get state variables
            std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
            Eigen::Map<const Eigen::Matrix<double, 6,7>> jacobian(jacobian_array.data());
            Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
            Eigen::Quaterniond orientation(transform.linear());

            // Compute error
            Eigen::VectorXd w(6), tau_d(7), error(6);
            if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
                orientation.coeffs() << -orientation.coeffs();
            };
            
            Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
            error.setZero(); 
            error.tail(3) << 

            
            w.setZero();
            w.tail(3) << orientation.x(), orientation.y(), orientation.z(); 

            // Calculate torque
            tau_d = jacobian.transpose() * (K_p * (w-w_d)*(time/time_max));

            std::array<double, 7> tau_d_array; 
            Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d; 
            franka::Torques output = tau_d_array; 

            if (time >= time_max) {
                std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
                return franka::MotionFinished(output);
            }

            return output;

        };

        robot.control(force_control_callback);

    }

    catch (const franka::Exception& e) {

        std::cout << e.what() << std::endl << "this message was displayed\n";
        return -1;
    }

};
