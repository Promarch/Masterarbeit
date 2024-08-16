#include <iostream>
#include <iomanip>
#include <fstream>
#include <ctime>
#include <array>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

// Sets default collision parameters, from https://frankaemika.github.io/libfranka/generate_joint_position_motion_8cpp-example.html
void setDefaultBehavior(franka::Robot &robot) {

    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
    robot.setJointImpedance({{3000, 3000, 3000, 3000, 3000, 3000, 3000}});
    robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
};

// Macro to convert variable name to string for the function
#define writeDataToFile(data) writeDataToFileImpl(data, #data)
std::string getCurrentDateTime() {
  std::time_t now = std::time(nullptr);
  char buf[80];
  std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", std::localtime(&now));
  return std::string(buf);
}
template <std::size_t N>
void writeDataToFileImpl(const std::vector<std::array<double, N>>& data, const std::string& var_name) {
  std::string filename = "data_output/" + var_name + "_" + getCurrentDateTime() + ".txt";
  std::ofstream data_file(filename);
  if (data_file.is_open()) {
    data_file << std::fixed << std::setprecision(4);
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

  // Variables to store the torque or position of the robot during the movement
  std::vector<std::array<double, 7>> tau_data, tau_filter_data, tau_desired_data, joint_position_data; // Vector to store the torque
  std::vector<std::array<double, 3>> position_data; // Vector to store the torque
  std::vector<std::array<double, 6>> force_data, force_tau_d_data; // Vector to store the force and torque on the EE
  
  try {
    // Set Up basic robot function
    franka::Robot robot("192.168.1.11");
    // Set new end-effector
    robot.setEE({1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.205, 1.0});
    // robot.setK({1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0});
    // robot.setEE({1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0});
    setDefaultBehavior(robot);
    franka::Model model = robot.loadModel();
    franka::RobotState initial_state = robot.readOnce();

      // Damping/Stifness
    const double translation_stiffness{500.0};
    const double rotation_stiffness{100.0};
    Eigen::MatrixXd K_p = Eigen::MatrixXd::Identity(6, 6);
    Eigen::MatrixXd K_d = Eigen::MatrixXd::Identity(6, 6);
    Eigen::MatrixXd K_i = Eigen::MatrixXd::Identity(6, 6); 
    Eigen::VectorXd Kn(7); Kn << 0, 0, 0, 0, 0, 300, 0;
    K_p.topLeftCorner(3,3) *= translation_stiffness; 
    K_p.bottomRightCorner(3,3) *= rotation_stiffness; 
    K_d.topLeftCorner(3,3) *= 1 * sqrt(translation_stiffness);
    K_d.bottomRightCorner(3,3) *= 1 * sqrt(rotation_stiffness);
    K_i.topLeftCorner(3,3) *= 2 * sqrt(translation_stiffness);
    K_i.bottomRightCorner(3,3) *= 2 * sqrt(rotation_stiffness); 

      // Time for the loop
    double time = 0.0;
    double time_max = 8;
    double time_acc = 5;
    double time_dec = 0.5; // time for decceleration, to stop abrubt braking
    double sampling_interval = 0.1;
    double next_sampling_time = 0;
    Eigen::Matrix<double, 6,1> int_error; int_error.setZero();
    double dt = 0;
    
    // Initial orientation
    Eigen::Map<const Eigen::Matrix<double, 7,1>> q_init(initial_state.q.data());
    Eigen::Affine3d transform_init(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
    Eigen::Vector3d position_init(transform_init.translation());
    Eigen::Quaterniond rotation_init(transform_init.linear());
    
      // Create desired rotation with rotation matrix
    // Flexion (rotation around EEs x-axis)
    double angle_flexion = -M_PI/9;
    Eigen::AngleAxisd vector_flexion(angle_flexion, Eigen::Vector3d::UnitX());
    Eigen::Matrix3d rotation_flexion = vector_flexion.toRotationMatrix();
    Eigen::Affine3d transform_flexion = Eigen::Affine3d(rotation_flexion);
    Eigen::Quaterniond quaternion_flexion(rotation_flexion);
    // Internal-external (rotation around EEs x-axis)
    double angle_internal = -M_PI/18;
    Eigen::AngleAxisd vector_internal(angle_internal, Eigen::Vector3d::UnitY());
    Eigen::Matrix3d rotation_internal = vector_internal.toRotationMatrix();
    Eigen::Affine3d transform_internal = Eigen::Affine3d(rotation_internal);
    Eigen::Quaterniond quaternion_internal(rotation_internal);
    // Combine rotations (first flexion then rotation)
    Eigen::Affine3d transform_d = transform_init * transform_flexion * transform_internal; 
    // Combine rotation quaternion for error plotting
    Eigen::Quaterniond quaternion_combined = quaternion_internal * quaternion_flexion;
    Eigen::Quaterniond rot_d = rotation_init * quaternion_combined;

    Eigen::Matrix<double, 3,1> pos_d;
    pos_d << position_init;

    // Stiffness/Damping for cartesian
    Eigen::VectorXd Kp(7), Kd(7);
    Kp << 600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0;
    Kd << 50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0;

    // std::cout << "Initial transformation matrix: \n" << rot_matrix_init << "\nCartesian desired transMatrix: \n" << cart_rotationFlexion << std::endl;
    // return -1;

    // Acceleration variables
    double factor_rot{0};
    double factor_pos{0};
    double factor_acc{0};

    // Pre-allocate stuff
    std::array<double, 16> cart_pose_array{}; 
    std::array<double, 7> tau_d_array{}; 
    Eigen::Matrix<double, 3, 1> error; 
    Eigen::Matrix<double, 7,1> tau_d;


// --------------------------------------------------------------------------------------------
// ----                           Position callback                                        ----
// --------------------------------------------------------------------------------------------

    // Define callback function to send Cartesian pose goals to get inverse kinematics solved.
    auto cartesian_pose_callback = [&](const franka::RobotState& robot_state, franka::Duration period) -> franka::CartesianPose {
      dt = period.toSec();
      time += dt;

        // Get current position
      Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
      Eigen::Vector3d position(transform.translation());
      Eigen::Quaterniond rotation(transform.linear());

        // Factor for smooth acceleration
      if (time<time_acc) {
        factor_acc = (1 - std::cos(M_PI * time/time_acc))/2;
      } 
      else {
        factor_acc = 1;
      }

        // Calculate desired pose
      Eigen::Affine3d transform_temp;
      transform_temp = transform_init.matrix() + factor_acc * (transform_d.matrix() - transform_init.matrix());
      // Set desired pose
      Eigen::VectorXd::Map(&cart_pose_array[0], 16) = Eigen::Map<Eigen::VectorXd>(transform_temp.matrix().data(), 16);

      // Rotational error
      if (rot_d.coeffs().dot(rotation.coeffs()) < 0.0) {
        rotation.coeffs() << -rotation.coeffs();
      }
      Eigen::Quaterniond error_quaternion(rotation.inverse() * rot_d);
      error << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
      // Transform to base frame
      error << transform.linear() * error;

      // Calculate positional error for tests
      double distance = (pos_d-position).norm(); 

      // Print current wrench, time, and absolute positional error every sampling_interval
      if (time >= next_sampling_time) {
        std::cout << std::fixed << std::setprecision(3);
        std::cout << "Time: " << time << std::endl << "Error: " << std::endl << error << std::endl << "Position error: " << distance << std::endl<< std::endl; 
        next_sampling_time += sampling_interval;
      }

      // // Stop motion when time is reached
      if (time >= time_max) {
        std::cout << std::fixed << std::setprecision(3);
        std::cout << "Time: " << time << std::endl << "Error: " << std::endl << error << std::endl << "Position error: " << distance << std::endl << std::endl; 
        std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
        return franka::MotionFinished(cart_pose_array);
      }
      // Send desired pose.
      return cart_pose_array;
    };

// --------------------------------------------------------------------------------------------
// ----                           Control callback                                         ----
// --------------------------------------------------------------------------------------------

    auto force_control_callback = [&] (const franka::RobotState& robot_state, franka::Duration /*period*/) -> franka::Torques {

        // Get state variables
      std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
      Eigen::Map<const Eigen::Matrix<double, 6,7>> jacobian(jacobian_array.data());
      Eigen::Map<const Eigen::Matrix<double, 7,1>> dq(robot_state.dq.data()); 
      Eigen::Map<const Eigen::Matrix<double, 7,1>> q(robot_state.q.data());
      Eigen::Map<const Eigen::Matrix<double, 7,1>> q_d(robot_state.q_d.data());

        // Calculate torque with aid of cartesian pose
      tau_d = Kp.cwiseProduct(q_d-q) - Kd.cwiseProduct(dq);
      Eigen::VectorXd::Map(&tau_d_array[0], 7) =  tau_d;


      // Map the position and error to an array (I cant add Eigen vectors to arrays)
      std::array<double, 3> pos_array{}; 
      std::array<double, 6> F_tau_d_array{}; 
      // Eigen::VectorXd::Map(&F_tau_d_array[0], 6) = F_tau_d;
      
      // Add the current data to the array
      tau_data.push_back(tau_d_array);
      tau_filter_data.push_back(robot_state.tau_ext_hat_filtered);
      // force_tau_d_data.push_back(F_tau_d_array);
      force_data.push_back(robot_state.K_F_ext_hat_K);
      joint_position_data.push_back(robot_state.q);

      // Send desired tau to the robot
      return tau_d_array;

    };

    // start control loop
    robot.control(force_control_callback, cartesian_pose_callback);
    // Write Data to .txt file
    writeDataToFile(tau_data);
    writeDataToFile(tau_filter_data);
    writeDataToFile(force_tau_d_data);
    writeDataToFile(position_data);
    writeDataToFile(force_data);
    writeDataToFile(joint_position_data);
  }
  // Catches Exceptions caused within the execution of a program (I think)
  catch (const franka::ControlException& e) {
    std::cout << "Control Exception: \n" << e.what() << std::endl;
    writeDataToFile(tau_data);
    writeDataToFile(tau_filter_data);
    writeDataToFile(force_tau_d_data);
    writeDataToFile(position_data);
    writeDataToFile(force_data);
    writeDataToFile(joint_position_data);
    return -1;
  }
  // General exceptions
  catch (const franka::Exception& e) {
    std::cout << "Other Exception: \n" << e.what() << std::endl;
    writeDataToFile(tau_data);
    return -1;
  }
  
}