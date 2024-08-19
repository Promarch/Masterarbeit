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
  std::string filename = "data_output_cartesian/" + var_name + "_" + getCurrentDateTime() + ".txt";
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
  std::vector<std::array<double, 7>> tau_data, tau_filter_data, tau_desired_data, joint_position_data; // Stores joint torque 
  std::vector<std::array<double, 6>> force_data, force_tau_d_data; // Stores wrench acting on EE
  std::vector<std::array<double, 5>> rotation_time_data; // stores the desired rotation with the current time

  
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

      // Stiffness Damping on joint level
    Eigen::VectorXd Kp(7), Kd(7);
    Kp << 200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 200.0;
          // 600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0;
    Kd << 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0;
          // 50.0, 50.0, 50.0, 50.0, 10.0, 25.0, 15.0;

      // Time for the loop
    double time_global = 0.0; // Time the robot has been running for
    double time_cycle = 0.0;  // Time since new position
    double time_max = 17;     // Maximum runtime
    double period_acc = 4;    // Time between old and new commanded position
    double period_reacceleration = 1; // Time to reaccelerate the joints if starting config is not the initial position
    double period_dec = 0.5; // time for decceleration, to stop abrubt braking
    double sampling_interval = 0.1; // Time for first sampling interval
    double next_sampling_time = 0;  // Interval at which the debug loop is called
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
    Eigen::Affine3d transform_start = transform_init;
    pos_d << position_init;

    // Acceleration variables
    double factor_torque{0};
    double factor_cart{0};

    // Pre-allocate stuff
    std::array<double, 16> cart_pose_array{}; 
    std::array<double, 7> tau_d_array{}; 
    Eigen::Matrix<double, 6, 1> error; 
    Eigen::Matrix<double, 7,1> tau_d;

      // Variables to set new position
    // Decceleration
    bool decceleration = false; // if true, robot deccelerates, used to stop the motion before setting a new position
    bool acceleration = false; // set true when external wrench is too high, will cause the robot to slowly accelerate again
    double t_dec = 0;   // will be set the moment a decceleration is called
    // New position
    Eigen::Affine3d transform_temp;

    Eigen::Matrix<double, 3, 1> pos_temp = pos_d;
    bool new_pos = false;   // state variable, if true new position will be set
    std::srand(std::time(nullptr)); // initialize random seed
    double max_flexion = -M_PI*2/9;    // Max possible flexion
    double max_internal = M_PI/12;   // Max possible internal-external rotation
    double range_internal = 2*max_internal; // Possible values (max_internal-min_internal)
    // Debug stuff
    Eigen::Matrix<double, 5, 1> rotation_time; 
    rotation_time << rot_d.coeffs(), time_global;
    std::array<double, 5> rotation_time_array{}; 
    Eigen::VectorXd::Map(&rotation_time_array[0], 5) = rotation_time;
    rotation_time_data.push_back(rotation_time_array);

// --------------------------------------------------------------------------------------------
// ----                                                                                    ----
// ----                           Position callback                                        ----
// ----                                                                                    ----
// --------------------------------------------------------------------------------------------

    // Define callback function to send Cartesian pose goals to get inverse kinematics solved.
    auto cartesian_pose_callback = [&](const franka::RobotState& robot_state, franka::Duration period) -> franka::CartesianPose {
      dt = period.toSec();
      time_global += dt;
      time_cycle += dt;

        // Get current position
      Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
      Eigen::Vector3d position(transform.translation());
      Eigen::Quaterniond rotation(transform.linear());

      // -------------------------------------------------------------------
      // ----               Calculate new position                      ----
      // -------------------------------------------------------------------

      if (new_pos==true) {
        // std::cout << "Old transform_temp: \n" << transform_temp.matrix() << std::endl;
          // New starting position is current position
        transform_start = transform_temp; 
          // Set new angles and compute new rotation
        // New Flexion
        angle_flexion = (std::rand()/(RAND_MAX/max_flexion)); 
        vector_flexion = Eigen::AngleAxisd(angle_flexion, Eigen::Vector3d::UnitX());
        transform_flexion = Eigen::Affine3d(vector_flexion.toRotationMatrix());
        // New Internal-external rotation
        if (abs(angle_flexion) < M_PI/18) {
          angle_internal = 0; // No internal rotation if the knee is extended
        }
        else {
          angle_internal = (std::rand()/(RAND_MAX/range_internal))-max_internal;
        }
        vector_internal = Eigen::AngleAxisd (angle_internal, Eigen::Vector3d::UnitY());
        transform_internal = Eigen::Affine3d(vector_internal.toRotationMatrix());
        // Combine rotations (first flexion then rotation)
        transform_d = transform_init * transform_flexion * transform_internal;
        // Set new position to false since new position was just set
        new_pos = false; 
        time_cycle = 0;
        // Debug: Combine rotation quaternion for error plotting
        quaternion_flexion = Eigen::Quaterniond(vector_flexion.toRotationMatrix());
        quaternion_internal = Eigen::Quaterniond(vector_internal.toRotationMatrix());
        quaternion_combined = quaternion_internal * quaternion_flexion;
        rot_d = rotation_init * quaternion_combined;
        // Debug: Print new angles and add to array
        rotation_time << rot_d.coeffs(), time_global;
        Eigen::VectorXd::Map(&rotation_time_array[0], 5) = rotation_time;
        rotation_time_data.push_back(rotation_time_array);
        std::cout << "\nNew flexion: " << angle_flexion*180/M_PI << ", New internal: " << angle_internal*180/M_PI << ", Time: " << time_global << std::endl;

      }

      // -------------------------------------------------------------------
      // ----      Acceleration and decceleration factor                ----
      // -------------------------------------------------------------------

      if (time_cycle<period_acc) {
        factor_cart = (1 - std::cos(M_PI * time_cycle/period_acc))/2;
      }
      else {
        factor_cart = 1;
      }

      // -------------------------------------------------------------------
      // ----                Calculate desired pose                     ----
      // -------------------------------------------------------------------

      transform_temp = transform_start.matrix() + factor_cart * (transform_d.matrix() - transform_start.matrix());
      Eigen::VectorXd::Map(&cart_pose_array[0], 16) = Eigen::Map<Eigen::VectorXd>(transform_temp.matrix().data(), 16);


        // Calculate error: 6x1 vector, first three elements are error in x/y/z, last three elements are rotation error in x/y/z
      error.head(3) << pos_d - position; 
      if (rot_d.coeffs().dot(rotation.coeffs()) < 0.0) {
        rotation.coeffs() << -rotation.coeffs();
      }
      Eigen::Quaterniond error_quaternion(rotation.inverse() * rot_d);
      error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
      // Transform to base frame
      error.tail(3) << transform.linear() * error.tail(3);


      // For debug: Print current wrench, time, and absolute positional error every "sampling_interval"
      if (time_global >= next_sampling_time) {
        std::cout << std::fixed << std::setprecision(3);
        std::cout << "Time: " << time_global  << ", Factor torque: " << factor_torque << "\nError: \n" << error << "\nPosition error: " << error.head(3).norm() << "\n\n"; 
        next_sampling_time += sampling_interval;
      }

      // Stop motion when time is reached
      if (time_global >= time_max) {
        std::cout << std::fixed << std::setprecision(3);
        std::cout << "Time: " << time_global << "\nError: \n" << error << "\nPosition error: " << error.head(3).norm(); 
        std::cout << "\n\nFinished motion, shutting down example" << std::endl;
        return franka::MotionFinished(cart_pose_array);
      }
      // Send desired pose.
      return cart_pose_array;
    };

// --------------------------------------------------------------------------------------------
// ----                                                                                    ----
// ----                           Control callback                                         ----
// ----                                                                                    ----
// --------------------------------------------------------------------------------------------

    auto force_control_callback = [&] (const franka::RobotState& robot_state, franka::Duration /*period*/) -> franka::Torques {

        // Get state variables
      Eigen::Map<const Eigen::Matrix<double, 7,1>> dq(robot_state.dq.data()); 
      Eigen::Map<const Eigen::Matrix<double, 7,1>> q(robot_state.q.data());
      Eigen::Map<const Eigen::Matrix<double, 7,1>> q_d(robot_state.q_d.data());
      Eigen::Map<const Eigen::Matrix<double, 6,1>> F_ext(robot_state.K_F_ext_hat_K.data());

        // Deccelerate if forces to high or desired position reached or no joint movement present
      if (decceleration==false) {
        if (F_ext.tail(3).norm()>4) {   // Forces too high
          decceleration = true;
          acceleration = true; 
          t_dec = time_cycle;
          std::cout << "Torques too high; Time: " << time_global << ", wrench: \n" << F_ext << "\n";
        }
        else if (F_ext.head(3).norm()>12) {   // Torque too high
          decceleration = true;
          acceleration = true; 
          t_dec = time_cycle;
          std::cout << "Forces too high; Time: " << time_global << ", wrench: \n" << F_ext << "\n";
        }
        else if (error.tail(3).norm()<0.04 and (time_cycle>period_acc or time_cycle<0.01) and (time_global+period_acc)<time_max) { // Position reached, acceleration finished, not too close to end of time_max
          new_pos = true;
          std::cout << "Desired rotation reached; Time: " << time_global << ", error: \n" << error << std::endl;
        }
        else if (dq.norm()<0.005 and time_cycle>period_acc and (time_global+period_acc)<time_max) { // Standstill reached, acceleration finished, not too close to end of time_max
          new_pos = true;
          std::cout << "No joint movement present; Time: " << time_global << ", error: \n" << error << std::endl;
        }
      }

        // Decceleration or acceleration factor 
      if ((time_global+period_dec)>time_max) {
        factor_torque = (1 + std::cos(M_PI * (time_global-(time_max-period_dec))/period_dec))/2;
      }
      else if (decceleration==true){
        factor_torque = (1 + std::cos(M_PI * (time_cycle-t_dec)/period_dec))/2 * factor_torque;
        if (factor_torque<0.001) {
          factor_torque = 0.0;
          if (time_cycle>period_acc and (time_global+period_acc)<time_max) { // if acceleration is finished and there is enough time to get to a new configuration
            decceleration=false;
            new_pos = true;
            std::cout << "new position to set, time: " << time_global << std::endl;
          }
        }
      }
      else if (acceleration == true){ // if decceleration was called because high wrench -> smoothly reaccelerate the joints
        factor_torque = (1 - std::cos(M_PI * time_cycle/period_reacceleration))/2;
        if (factor_torque>0.995){
          acceleration = false;
          std::cout << "\nFinished accelerating, time: " << time_global << "\n";
        }
      }
      else {
        factor_torque = 1;
      }
        // Calculate torque with aid of cartesian pose
      tau_d = Kp.cwiseProduct(q_d-q) - Kd.cwiseProduct(dq);
      Eigen::VectorXd::Map(&tau_d_array[0], 7) =  factor_torque * tau_d;

      
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
    writeDataToFile(force_data);
    writeDataToFile(joint_position_data);
    writeDataToFile(rotation_time_data);
  }
  // Catches Exceptions caused within the execution of a program (I think)
  catch (const franka::ControlException& e) {
    std::cout << "Control Exception: \n" << e.what() << std::endl;
    writeDataToFile(tau_data);
    writeDataToFile(tau_filter_data);
    writeDataToFile(force_data);
    writeDataToFile(joint_position_data);
    writeDataToFile(rotation_time_data);
    return -1;
  }
  // General exceptions
  catch (const franka::Exception& e) {
    std::cout << "Other Exception: \n" << e.what() << std::endl;
    writeDataToFile(tau_data);
    return -1;
  }
  
}