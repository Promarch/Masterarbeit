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

// Sets default collision parameters, from https://frankaemika.github.io/libfranka/generate_joint_position_motion_8cpp-example.html
void setDefaultBehavior(franka::Robot &robot) {

    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
    robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
    robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
};

// Takes the current robot state and outputs a 6x1 vector: x,y,z,phi_x, phi_y, phi_z
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

// Function that takes an orientation (position+rotation) and translates it in the EE coordinates
Eigen::Matrix<double, 6, 1> OrientationToEE(const franka::RobotState& robot_state, Eigen::Matrix<double, 6,1> vector) {

  // Get the rotation matrix from the translation matrix
  Eigen::Matrix4d transformation(robot_state.O_T_EE.data());
  Eigen::Matrix3d rotation = transformation.topLeftCorner(3,3); 

  // Apply the rotation
  vector.head(3) = rotation * vector.head(3);
  vector.tail(3) = rotation * vector.tail(3); 

  return vector; 

}

// Macro to convert variable name to string for the function
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

// Get x_e, but with quaternions
Eigen::Matrix<double, 6 ,1> computeTransformationQuat(const franka::RobotState& robot_state) {

  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.linear());

  Eigen::Matrix<double, 6 ,1> x_e; 
  x_e.head(3) = position; 
  x_e.tail(3) << orientation.x(), orientation.y(), orientation.z();

  return x_e;
}

int main() {
  try {
    // Set Up basic robot function
    franka::Robot robot("192.168.1.11");
    setDefaultBehavior(robot);
    franka::Model model = robot.loadModel();
    franka::RobotState initial_state = robot.readOnce();

      // Damping/Stifness
    const double translation_stiffness{50.0};
    const double rotation_stiffness{20.0};
    Eigen::MatrixXd K_p = Eigen::MatrixXd::Identity(6, 6);
    Eigen::MatrixXd K_d = Eigen::MatrixXd::Identity(6, 6); 
    K_p.topLeftCorner(3,3) *= translation_stiffness; 
    K_p.bottomRightCorner(3,3) *= rotation_stiffness; 
    K_d.topLeftCorner(3,3) *= 2 * sqrt(translation_stiffness);
    K_d.bottomRightCorner(3,3) *= 2 * sqrt(rotation_stiffness); 

      // Variables for the loop
    double time = 0.0;
    double time_max = 2;
    double time_acc = 1.5;
    double time_dec = 0.5; // time for decceleration, to stop abrubt braking
    double sampling_interval = 0.1;
    double next_sampling_time = 0;

    // Initial position
    Eigen::Affine3d transform_init(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
    Eigen::Vector3d position_init(transform_init.translation());
    Eigen::Quaterniond rotation_init(transform_init.linear());
    
      // Desired relative position
    Eigen::Matrix<double, 3,1> delta_pos, pos_d;
    delta_pos = {0.0, -0.2, 0.0};
    delta_pos = rotation_init * delta_pos; 
      // Create quaternion to rotate around desired angle
    double rot_angle = M_PI/6;
    Eigen::Vector3d axis(1,0,0);
    Eigen::AngleAxisd angle_axis(rot_angle, axis);
    Eigen::Quaterniond rot_quaternion(angle_axis);
    Eigen::Quaterniond rot_quaternion_rel = rotation_init * rot_quaternion * rotation_init.inverse();
    // Compute the desired position and rotation
    pos_d << position_init + delta_pos;
    Eigen::Quaterniond rot_d = rot_quaternion_rel * rotation_init; 
    
    // Other variables
    double factor = 0;


    auto force_control_callback = [&] (const franka::RobotState& robot_state, franka::Duration period) -> franka::Torques {

      time += period.toSec();

      // acceleration time
      if (time<time_acc) {
        factor = (1 - std::cos(M_PI * time/time_acc))/2;
      } 
      else if (time<(time_max-time_dec)){
        factor = 1;
      }
      else {
        factor = (1 + std::cos(M_PI * (time-(time_max-time_dec))/time_dec))/2;
      }

        // Get state variables
      std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
      Eigen::Map<const Eigen::Matrix<double, 6,7>> jacobian(jacobian_array.data());
      Eigen::Map<const Eigen::Matrix<double, 7,1>> dq(robot_state.dq.data()); 
        // Get current position
      Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
      Eigen::Vector3d position(transform.translation());
      Eigen::Quaterniond rotation(transform.linear());

        // Compute trajectory between current and desired orientation
      // Positional error
      Eigen::Matrix<double, 6,1> error; 
      error.head(3) << pos_d - position; 
      // Rotational error
      if (rot_d.coeffs().dot(rotation.coeffs()) < 0.0) {
        rotation.coeffs() << -rotation.coeffs();
      }
      // Difference quaternion
      Eigen::Quaterniond error_quaternion(rotation.inverse() * rot_d);
      error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
      // Transform to base frame
      error.tail(3) << transform.linear() * error.tail(3);

      // Calculate the necessary wrench, from Handbook of robotics, Chap. 7
      Eigen::Matrix<double, 6,1> h_c; h_c.setZero();
      Eigen::Matrix<double, 6,1> v_e = jacobian * dq; 
      h_c = K_p * error - K_d * v_e; 

      // Calculate torque and map to an array
      std::array<double, 7> tau_d{};
      Eigen::VectorXd::Map(&tau_d[0], 7) = jacobian.transpose() * h_c * factor;

      // Calculate positional error for tests
      double distance = (pos_d-position).norm(); 

      // Stop if time is reached
      if (time >= time_max) {
        std::cout << std::fixed << std::setprecision(3);
        std::cout << "Time: " << time << std::endl << "Rotation error: " << std::endl << error.tail(3) << std::endl << "Position error: " << distance << std::endl << std::endl; 
        std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
        franka::Torques output = tau_d;
        return franka::MotionFinished(output);
      }

      // Print current wrench, time, and absolute positional error
      if (time >= next_sampling_time) {
        std::cout << std::fixed << std::setprecision(3);
        std::cout << "Time: " << time << std::endl << "Rotation error: " << std::endl << error.tail(3) << std::endl << "Position error: " << distance << std::endl<< std::endl; 
        next_sampling_time += sampling_interval;
      }

      return tau_d;

    };

    robot.control(force_control_callback);
  }

  catch (const franka::Exception& e) {

    std::cout << e.what() << std::endl;
    // writeDataToFile();
    return -1;
  }
}