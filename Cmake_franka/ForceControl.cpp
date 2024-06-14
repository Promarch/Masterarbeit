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

Eigen::Matrix<double, 6, 1> computeTransformation(const franka::RobotState& robot_state) {
    // Extract the transformation matrix
    Eigen::Matrix4d transform_init(robot_state.O_T_EE.data());

    // Extract the position (translation) vector
    Eigen::Vector3d position_init = transform_init.block<3, 1>(0, 3);

    // Extract the rotation matrix
    Eigen::Matrix3d rotation_init = transform_init.topLeftCorner(3, 3);

    // Compute the Euler angles
    double z_rot_init = atan2(rotation_init(1, 0), rotation_init(0, 0));
    double y_rot_init = atan2(-rotation_init(2, 0), sqrt(pow(rotation_init(2, 1), 2) + pow(rotation_init(2, 2), 2)));
    double x_rot_init = atan2(rotation_init(2, 1), rotation_init(2, 2));

    // Create the combined 6x1 vector
    Eigen::Matrix<double, 6, 1> x;
    x << position_init(0), position_init(1), position_init(2), x_rot_init, y_rot_init, z_rot_init;

    return x;
}

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

int main() {
  try {
    // Set Up basic robot function
    franka::Robot robot("192.168.1.11");
    setDefaultBehavior(robot);
    franka::Model model = robot.loadModel();
    franka::RobotState initial_state = robot.readOnce();

      // Variables for the loop
    double time = 0.0;
    double time_max = 2.5;
    double time_acc = 0.7;
    double sampling_interval = 0.1;
    double next_sampling_time = sampling_interval;

      // Trajectory variable
    Eigen::Matrix<double, 6,1> delta = {0.0, 0.10, 0.0, 0.0, 0.0, 0.0};
    Eigen::Matrix<double, 6,1> x_d, x_d_test; x_d_test.setZero(); 
    x_d = computeTransformation(initial_state) + delta; // 
    x_d_test(1) = x_d(1); 
    double factor = 0;

      // Damping/Stifness
    const double translation_stiffness{100.0};
    const double rotation_stiffness{10.0};
    Eigen::MatrixXd K_p = Eigen::MatrixXd::Identity(6, 6);
    Eigen::MatrixXd K_d = Eigen::MatrixXd::Identity(6, 6); 
    K_p.topLeftCorner(3,3) *= translation_stiffness; 
    K_p.bottomRightCorner(3,3) *= rotation_stiffness; 
    K_d.topLeftCorner(3,3) *= 2 * sqrt(translation_stiffness);
    K_d.bottomRightCorner(3,3) *= 2 * sqrt(rotation_stiffness); 
    
    auto force_control_callback = [&] (const franka::RobotState& robot_state, franka::Duration period) -> franka::Torques {

      time += period.toSec();

      // Get current position and rotation
      std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
      Eigen::Map<const Eigen::Matrix<double, 6,7>> jacobian(jacobian_array.data());
      Eigen::Map<const Eigen::Matrix<double, 7,1>> dq(robot_state.dq.data()); 
      Eigen::Matrix<double, 6,1> v_e = jacobian * dq; 
      Eigen::Matrix<double, 6,1> x_e, x_e_test; x_e_test.setZero();
      x_e = computeTransformation(robot_state);
      x_e_test(1) = x_e(1); 
      // acceleration time
      if (time<time_acc) {
        factor = (1 - std::cos(M_PI * time/time_acc))/2;
      } 
      else {
        factor = 1;
      }
      
      Eigen::Matrix<double, 6,1> h_c; h_c.setZero();
      x_e.setZero(); 

      h_c = K_p * (x_d_test-x_e_test) - K_d * v_e; 

      std::array<double, 7> tau_d{};
      Eigen::VectorXd::Map(&tau_d[0], 7) = jacobian.transpose() * h_c * factor;

      if (time >= time_max) {
        std::cout << "Time: " << time << std::endl << "H_c: " << h_c << std::endl; 
        std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
        franka::Torques output = tau_d;
        return franka::MotionFinished(output);
      }

      if (time >= next_sampling_time) {
          std::cout << "Time: " << time << std::endl << "H_c: " << h_c << std::endl; 
          next_sampling_time += sampling_interval;
      }

      return tau_d;

    };

    robot.control(force_control_callback);
  }

  catch (const franka::Exception& e) {

    std::cout << e.what() << std::endl << "this message was displayed\n";
    // writeDataToFile();
    return -1;
  }
}