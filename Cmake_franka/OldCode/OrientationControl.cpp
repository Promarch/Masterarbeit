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

// Macro to convert variable name to string
#define writeDataToFile(data) writeDataToFileImpl(data, #data ".txt")

void writeDataToFileImpl(const std::vector<std::array<double, 7>>& data, const std::string& filename) {
  std::ofstream data_file(filename);
  if (data_file.is_open()) {
    data_file << std::fixed << std::setprecision(2);
    for (const auto& sample : data) {
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
}

int main(){

    std::vector<std::array<double, 7>> F_d_data;
    std::vector<std::array<double, 6>> stiff_data;
    std::vector<std::array<double, 6>> damp_data; 

    try {
        // Set Up basic robot function
        franka::Robot robot("192.168.1.11");
        setDefaultBehavior(robot);
        franka::Model model = robot.loadModel(); 

        // Set proportional factors
        const double translation_stiffness{150.0};
        const double rotation_stiffness{10.0};
        Eigen::MatrixXd K_p = Eigen::MatrixXd::Identity(6, 6);
        Eigen::MatrixXd K_d = Eigen::MatrixXd::Identity(6, 6); 
        K_p.topLeftCorner(3,3) *= translation_stiffness; 
        K_p.bottomRightCorner(3,3) *= rotation_stiffness; 
        K_d.topLeftCorner(3,3) *= 2 * sqrt(translation_stiffness);
        K_d.bottomRightCorner(3,3) *= 2 * sqrt(rotation_stiffness); 

             
            // Variables for the loop
        double time = 0.0;
        double time_max = 4.0;
        double sampling_interval = 0.05;
        double next_sampling_time = sampling_interval;
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
            Eigen::Map<const Eigen::Matrix<double, 7,1>> dq(robot_state.dq.data()); 
            Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
            Eigen::Quaterniond orientation(transform.linear());

            // Compute error
            Eigen::VectorXd w(6), tau_d(7), error(6);
            
            if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
                orientation.coeffs() << -orientation.coeffs();
            };
            /*
            Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
            error.setZero(); 
            */

            w.setZero();
            w.tail(3) << orientation.x(), orientation.y(), orientation.z(); 

            // Calculate torque
            tau_d = jacobian.transpose() * (K_p * (w-w_d) * (time/time_max)) ; // - K_d * (jacobian * dq) 

            std::array<double, 7> tau_d_array; 
            Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d; 
            franka::Torques output = tau_d_array; 

            
            std::array<double, 7> F_d, damp, stiff;
            Eigen::VectorXd::Map(&F_d[0], 7) = jacobian.transpose() * (K_p * (w-w_d)); 
            //Eigen::VectorXd::Map(&damp[0], 6) = K_d * (jacobian * dq); 
            //Eigen::VectorXd::Map(&stiff[0], 6) = K_p * (w-w_d)*(time/time_max); 
            
            if (time >= next_sampling_time) {
                F_d_data.push_back(robot_state.q_d);
                //damp_data.push_back(damp);
                //stiff_data.push_back(stiff);
                next_sampling_time += 0.1;
            }
            

            if (time >= time_max) {
                std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
                return franka::MotionFinished(output);
            }

            return output;

        };

        robot.control(force_control_callback);
        writeDataToFile(F_d_data);

    }

    catch (const franka::Exception& e) {

        std::cout << e.what() << std::endl << "this message was displayed\n";
        writeDataToFile(F_d_data);
        return -1;
    }

};
