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

// Linux headers
#include <fcntl.h>      // Contains file controls like O_RDWR
#include <errno.h>      // Error integer and strerror() function
#include <termios.h>    // Contains POSIX terminal control definitions
#include <unistd.h>     // write(), read(), close()
#include <sys/ioctl.h>
#include <linux/serial.h>
#include "cmake/BotaForceTorqueSensorComm.h"

void setDefaultBehavior(franka::Robot& robot) {
  // Set collision behavior.
  robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                              {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                              {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                              {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});
  robot.setJointImpedance({{3000, 3000, 3000, 1000, 3000, 3000, 3000}});
  robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
}

  // Function to write a vector to a text file
// Macro to convert variable name to string for the function
#define writeDataToFile(data) writeDataToFileImpl(data, #data)
std::string getCurrentDateTime() {
  std::time_t now = std::time(nullptr);
  char buf[80];
  std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", std::localtime(&now));
  return std::string(buf);
}
template <typename T, std::size_t N>
void writeDataToFileImpl(const std::vector<std::array<T, N>>& data, const std::string& var_name) {
  std::string filename = "data_output_knee/" + var_name + "_" + getCurrentDateTime() + ".txt";
  std::ofstream data_file(filename);
  if (data_file.is_open()) {
    data_file << std::fixed << std::setprecision(5);
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

// Function and classes needed for the Botasys sensor, from: https://gitlab.com/botasys/bota_serial_driver
int serial_port;
class myBotaForceTorqueSensorComm : public BotaForceTorqueSensorComm
{
  public:
  int serialReadBytes(uint8_t* data, size_t len) override {
    return read(serial_port, data, len);
  }
  int serialAvailable() override {
    int bytes;
    ioctl(serial_port, FIONREAD, &bytes);
    return bytes;
  }
} sensor;
void set_up_serial(){

    // Open the serial port. Change device path as needed.
    printf("Open serial port.\n");
    serial_port = open("/dev/ttyUSB0", O_RDWR);
    printf("Opened port %i.\n",serial_port);

    if (serial_port < 0) {
      printf("Error %i from opening device: %s\n", errno, strerror(errno));
      if (errno == 13) {
        printf("Add the current user to the dialout group");
      }
      return;
    }

    // Create new termios struc, we call it 'tty' for convention
    struct termios tty;
    struct serial_struct ser_info;
    memset(&tty, 0, sizeof(tty));

    // Read in existing settings, and handle any error
    if(tcgetattr(serial_port, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    }

    tty.c_cflag &= ~PARENB; // Disable parity
    tty.c_cflag &= ~CSTOPB; // 1 stop bit
    tty.c_cflag |= CS8; // 8 bits per byte
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON; // Disable canonical mode
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    tty.c_cc[VTIME] = 10; // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    // Set in/out baud rate to be 460800
    cfsetispeed(&tty, B460800);
    cfsetospeed(&tty, B460800);

    // Save tty settings, also checking for error
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    }

    // Enable linux FTDI low latency mode
    ioctl(serial_port, TIOCGSERIAL, &ser_info);
    ser_info.flags |= ASYNC_LOW_LATENCY;
    ioctl(serial_port, TIOCSSERIAL, &ser_info);
    printf("Finished serial port set up\n");
}
void check_sensor(){

    // Variables for debug/ensure the program cancels correctly
    int counter=0;  // Counts how many times the sensor could be read
    auto t_start = std::chrono::high_resolution_clock::now();
    auto t_current = std::chrono::high_resolution_clock::now();
    auto t_limit = std::chrono::seconds(5);
    auto elapsedTime = std::chrono::duration_cast<std::chrono::seconds>(t_current - t_start);

    while (counter<20) {
      switch(sensor.readFrame())
      {
        case BotaForceTorqueSensorComm::VALID_FRAME:
          if (sensor.frame.data.status.val>0)
          {
            printf("No valid forces:\n");
            printf(" app_took_too_long: %i\n",sensor.frame.data.status.app_took_too_long);
            printf(" overrange: %i\n",sensor.frame.data.status.overrange);
            printf(" invalid_measurements: %i\n",sensor.frame.data.status.invalid_measurements);
            printf(" raw_measurements: %i\n",sensor.frame.data.status.raw_measurements);
          }
          else
          {
            // for (uint8_t i=0; i<6; i++)
            // {
            //   printf("%f",sensor.frame.data.forces[i]);
            //   printf("\t");
            // }
            counter++;
            // printf("Counter: %i\n", counter);
          }
          break;
        case BotaForceTorqueSensorComm::NOT_VALID_FRAME:
          printf("No valid frame: %i\n",sensor.get_crc_count());
          break;
        case BotaForceTorqueSensorComm::NOT_ALLIGNED_FRAME:
          printf("lost sync, trying to reconnect\n");
          break;
        case BotaForceTorqueSensorComm::NO_FRAME:
          break;
        default:
          // Break the loop if it takes above 5 seconds
          t_current = std::chrono::high_resolution_clock::now();
          elapsedTime = std::chrono::duration_cast<std::chrono::seconds>(t_current - t_start);
          if (elapsedTime>t_limit) {
              printf("Sensor set up failed\n");
              return;
          }
          break;
      }
    }// while app run
    printf("Sensor set up finished.\n");
}


int main(int argc, char** argv) {

  // Variables to store the torque or position of the robot during the movement
  std::vector<std::array<double, 7>> tau_data, tau_filter_data, tau_desired_data, joint_position_data; // Stores joint torque 
  std::vector<std::array<double, 6>> F_robot_data, force_tau_d_data; // Stores wrench acting on EE
  std::vector<std::array<float , 6>> F_sensor_data, F_sensor_total_data; 
  std::vector<std::array<double, 5>> rotation_time_data; // stores the desired rotation with the current time
  std::vector<std::array<double, 16>> O_T_EE_data, F_T_EE_data; 

    // Sensor set up
  // Serial port set up
  set_up_serial();
  // Set up sensor
  check_sensor();
  // Get starting values of the external sensor
  float F_sensor_start_aligned[6];
  std::array<float, 6> F_sensor_start_array; 
  std::memcpy(F_sensor_start_aligned, sensor.frame.data.forces, 6 * sizeof(float));
  std::copy(F_sensor_start_aligned, F_sensor_start_aligned + 6, F_sensor_start_array.begin());
  Eigen::Map<Eigen::Matrix<float, 1, 6>> F_sensor_start(F_sensor_start_aligned);
  std::cout << "Starting sensor force: " << F_sensor_start << "\n";

  try {
    // Set Up basic robot function
    franka::Robot robot("192.168.1.11");
    // Set new end-effector
    robot.setEE({1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.220, 1.0});
    setDefaultBehavior(robot);
    franka::Model model = robot.loadModel();
    franka::RobotState initial_state = robot.readOnce();

      // Stiffness Damping on joint level
    Eigen::VectorXd Kp(7), Kd(7);
    Kp << 200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 200.0;
    Kd << 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0;

      // Time variables
    double time_global = 0.0; // Time the robot has been running for
    double time_max = 30;      // Maximum runtime
    double dt = 0;            // Initialization of time variable of the loop
    double period_acc = 2;    // Time between old and new commanded position
    double period_dec = 0.5;  // time for decceleration, to stop abrubt braking
    double time_cycle = 0.0;  // Time since new position
    double period_reacceleration = 0.5; // Time to reaccelerate the joints if starting config is not the initial position
    double sampling_interval = 0.1;     // Time for first sampling interval
    double next_sampling_time = 0;      // Interval at which the debug loop is called

      // Set limitations to max speed
    double vel_max = 0.10; // [rad/s]
    double factor_vel = 0;
    double factor_tau = 0;

    // Initial orientation
    Eigen::Map<const Eigen::Matrix<double, 7,1>> q_init(initial_state.q.data());
    Eigen::Affine3d transform_init(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
    Eigen::Vector3d position_init(transform_init.translation());
    Eigen::Quaterniond rotation_init(transform_init.linear());

      // Desired Rotation, created with quaternions
    // Flexion (Rotation around y in local CoSy) 
    double angle_flexion = -M_PI/12;
    Eigen::Vector3d axis_flexion(1,0,0);
    Eigen::AngleAxisd angle_axis_flexion(angle_flexion, axis_flexion);
    Eigen::Quaterniond quaternion_flexion(angle_axis_flexion);
    // Varus-Valgus (Rotation around z in local CoSy) 
    double angle_varus = -M_PI/12*0;
    Eigen::Vector3d axis_varus(0,1,0);
    Eigen::AngleAxisd angle_axis_varus(angle_varus, axis_varus);
    Eigen::Quaterniond quaternion_varus(angle_axis_varus);
    // Combine the rotations
    Eigen::Quaterniond quaternion_combined = quaternion_varus * quaternion_flexion;
    // Translate the desired rotation into local coordinate system (rotation in EE-CoSy instead of base CoSy)
    Eigen::Quaterniond rot_quaternion_rel = rotation_init * quaternion_combined * rotation_init.inverse();
    // Add rotation to initial orientation
    Eigen::Quaterniond rot_d = rot_quaternion_rel * rotation_init; 

    // Remember which desired position where commanded when
    Eigen::Matrix<double, 5, 1> rotation_time; 
    rotation_time << rot_d.coeffs(), time_global;
    std::array<double, 5> rotation_time_array{}; 
    Eigen::VectorXd::Map(&rotation_time_array[0], 5) = rotation_time;
    rotation_time_data.push_back(rotation_time_array);

      // Variables to set new position
    // Decceleration
    bool decceleration = false; // if true, robot deccelerates, used to stop the motion before setting a new position
    bool acceleration = false; // set true when external wrench is too high, will cause the robot to slowly accelerate again after deceleration of the joint
    double t_dec = 0;   // will be set the moment a deceleration is called
    double t_acc = 0;   // will be set the moment an acceleration is called
    // New position
    bool new_pos = false;   // state variable, if true new position will be set
    bool pos_reached = false; // state variable, set true when the desired position has been reached
    std::srand(std::time(nullptr)); // initialize random seed
    double max_flexion = -M_PI/5;    // Max possible flexion
    double max_varus = M_PI/12;   // Max possible internal-external rotation
    double range_varus = 2*max_varus; // Possible values (max_varus-min_varus)

    // Random debug variables
    bool test = false; 

    // Pre-allocate stuff
    std::array<double, 6> cart_vel_array{}; 
    std::array<double, 7> tau_d_array{}; 
    Eigen::Matrix<double, 6, 1> cart_vel; cart_vel.setZero();
    Eigen::Matrix<double, 6, 1> error; 
    Eigen::Matrix<double, 7,1> tau_d;
    std::array<float, 6> F_sensor_array;
    std::array<float, 6> F_sensor_temp_array;
    Eigen::Quaterniond error_quaternion;

    std::cout << "Robot will start moving now \n"
              << "Press Enter to continue... \n";
    std::cin.ignore();   

// --------------------------------------------------------------------------------------------
// ----                                                                                    ----
// ----                           Velocity callback                                        ----
// ----                                                                                    ----
// --------------------------------------------------------------------------------------------

    auto cartesian_velocity_callback = [&](const franka::RobotState& robot_state, franka::Duration period) -> franka::CartesianVelocities {
      dt = period.toSec();
      time_global += dt;
      time_cycle += dt;

        // Get current position
      Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
      Eigen::Vector3d position(transform.translation());
      Eigen::Quaterniond rotation(transform.linear());

      // -------------------------------------------------------------------
      // ----                   Set new position                        ----
      // -------------------------------------------------------------------
      if (new_pos==true){
          // Set new angles and compute new rotation
        // New flexion
        angle_flexion = (std::rand()/(RAND_MAX/max_flexion)); 
        Eigen::AngleAxisd angle_axis_flexion(angle_flexion, axis_flexion);
        Eigen::Quaterniond quaternion_flexion(angle_axis_flexion);
        // New internal rotation 
        if (abs(angle_flexion) < M_PI/18) {
          angle_varus = 0; // No internal rotation if the knee is extended
        }
        else {
          angle_varus = (std::rand()/(RAND_MAX/range_varus))-max_varus;
        }
        Eigen::AngleAxisd angle_axis_varus(angle_varus, axis_varus);
        Eigen::Quaterniond quaternion_varus(angle_axis_varus);
        // Combine the rotations
        quaternion_combined = quaternion_varus * quaternion_flexion;
        rot_quaternion_rel = rotation_init * quaternion_combined * rotation_init.inverse();
        // Calculate new rotation
        rot_d = rot_quaternion_rel * rotation_init; 
          // Reset state variable and cycle time
        pos_reached = false; 
        new_pos = false; 
        time_cycle = 0;
          // Print text and update variable with desired rotations
        rotation_time << rot_d.coeffs(), time_global;
        Eigen::VectorXd::Map(&rotation_time_array[0], 5) = rotation_time;
        rotation_time_data.push_back(rotation_time_array);
        std::cout << "New flexion: " << angle_flexion*180/M_PI << " and varus: " << angle_varus*180/M_PI << " and quaternion \n" << rot_d;
        std::cout << "\nCurrent acceleration factor: " << factor_vel << ", and time: " << time_global << "\n\n";  
      }

      // -------------------------------------------------------------------
      // ----      Acceleration and decceleration factor                ----
      // -------------------------------------------------------------------
      if ((time_global+period_dec)>time_max){ // Deccelerate if close to end time
        factor_vel = (1 + std::cos(M_PI * (time_global-(time_max-period_dec))/period_dec))/2 * factor_vel;
      }
      else if (pos_reached==true or decceleration==true){  // Desired position (or limits) reached, slow down to 0 and then get new position
        factor_vel = (1 + std::cos(M_PI * (time_cycle-t_dec)/period_dec))/2 * factor_vel;
        if (factor_vel<0.001 and pos_reached==true) {
          factor_vel = 0.0;
          new_pos = true;
          std::cout << "New position called, time: " << time_global << "\n";
        }
      }
      else if (time_cycle<period_acc) {  // Normal start up
        factor_vel = (1 - std::cos(M_PI * time_cycle/period_acc))/2;
      }
      else {
        factor_vel = 1;
      }

      // -------------------------------------------------------------------
      // ----                 Calculate trajectory                      ----
      // -------------------------------------------------------------------

      // Positional error (nor used in calculations,only for debugging)
      error.setZero();
      error.head(3) << position_init - position; 
      // Rotational error
      if (rot_d.coeffs().dot(rotation.coeffs()) < 0.0) {
        rotation.coeffs() << -rotation.coeffs();
      }
      // "difference" quaternion
      error_quaternion = rotation.inverse() * rot_d;
      error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();

      // Transform error to base frame and scale with max_speed
      cart_vel.tail(3) << factor_vel * transform.rotation() * error.tail(3)/error.tail(3).norm() * vel_max;
      Eigen::VectorXd::Map(&cart_vel_array[0], 6) =  cart_vel;

      // -------------------------------------------------------------------
      // ----             Print stuff and send velocity                 ----
      // -------------------------------------------------------------------

      // For debug: Print current wrench, time, and absolute positional error every "sampling_interval"
      if (time_global >= next_sampling_time) {
        std::cout << std::fixed << std::setprecision(3);
        std::cout << "Time: " << time_global  << ", Acc factor: " << factor_vel << ", Error: \n" << error << "\n\n"; 
        next_sampling_time += sampling_interval;
      }

      // Stop motion when time is reached
      if (time_global >= time_max) {
        std::cout << std::fixed << std::setprecision(3);
        std::cout << "Time: " << time_global << ", Acc factor:" << factor_vel << ", Velocity_d: \n" << cart_vel; 
        std::cout << "\n\nFinished motion, shutting down example" << std::endl;
        return franka::MotionFinished(cart_vel_array);
      }
      // Send desired pose.
      return cart_vel_array;

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
      // Get sensor values
      alignas(alignof(float[6])) float aligned_forces[6]; // necessary cause otherwise i get the warning: taking address of packed member of ‘BotaForceTorqueSensorComm::AppOutput::<unnamed struct>’ may result in an unaligned pointer value
      if (sensor.readFrame() == BotaForceTorqueSensorComm::VALID_FRAME){
        std::memcpy(aligned_forces, sensor.frame.data.forces, sizeof(aligned_forces));
        std::copy(aligned_forces, aligned_forces + 6, F_sensor_temp_array.begin());
        for (size_t i=0; i<6; i++) {
          F_sensor_array[i] = F_sensor_temp_array[i] - F_sensor_start_array[i];
        }
        Eigen::Map<Eigen::Matrix<float, 6, 1>> F_sensor(F_sensor_array.data());
        F_sensor_data.push_back(F_sensor_array);
      }

      // -------------------------------------------------------------------
      // ----               Check if deceleration needed                ----
      // -------------------------------------------------------------------

        // Deccelerate if forces to high, or set new pos if desired position reached or no joint movement present
      if (decceleration==false) { // only call when the robot is not already decelerating
        if (F_ext.tail(3).norm()>5) {   // Torque too high
          decceleration = true;
          t_dec = time_cycle;
          std::cout << "Torques too high; Time: " << time_global << ", wrench: \n" << F_ext << "\n";
        }
        else if (F_ext.head(3).norm()>15) {   // Force too high
          decceleration = true;
          t_dec = time_cycle;
          std::cout << "Forces too high; Time: " << time_global << ", wrench: \n" << F_ext << "\n";
        }
        else if (error.tail(3).norm()<0.035 and pos_reached==false) { // Position reached & position not reached yet
          pos_reached = true;
          t_dec = time_cycle; 
          std::cout << "Desired rotation reached; Time: " << time_global << ", error: \n" << error << std::endl;
        }
        else if (dq.norm()<0.005 and time_cycle>period_acc and pos_reached==false) { // Standstill reached, robot should be moving & position not reached yet
          pos_reached = true;
          t_dec = time_cycle; 
          std::cout << "No joint movement present; Time: " << time_global << ", error: \n" << error << std::endl;
        }
      }

      // -------------------------------------------------------------------
      // ----                Calculate de/acceleration                  ----
      // -------------------------------------------------------------------

      if (decceleration==true){  // Decceleration called due to exceeded wrench
        factor_tau = (1 + std::cos(M_PI * (time_cycle-t_dec)/period_dec))/2 * factor_tau;
        if (factor_tau<0.001 and (time_cycle-t_dec)>period_dec) { // Restart if no force on joints and deceleration period is over
          factor_tau = 0.0;
          decceleration = false;
          acceleration = true;
          t_acc = time_global;
          pos_reached = true;
          std::cout << "New position to set, time: " << time_global << std::endl;
        }
      }
      else if (acceleration == true){ // if decceleration was called because high wrench -> smoothly reaccelerate the joint torque
        factor_tau = (1 - std::cos(M_PI * (time_global-t_acc)/period_reacceleration))/2;
        if (factor_tau>0.995){
          acceleration = false;
          std::cout << "\nFinished joint acceleration, time: " << time_global << "\n";
        }
      }
      else {
        factor_tau = 1;
      }

      // -------------------------------------------------------------------
      // ----           Calculate tau and write variables               ----
      // -------------------------------------------------------------------

        // Calculate torque with aid of cartesian velocity
      tau_d = Kp.cwiseProduct(q_d-q) - Kd.cwiseProduct(dq);
      Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;

      // Add the current data to the array
      tau_data.push_back(tau_d_array);
      tau_filter_data.push_back(robot_state.tau_ext_hat_filtered);
      F_robot_data.push_back(robot_state.K_F_ext_hat_K);
      F_sensor_total_data.push_back(F_sensor_array);
      joint_position_data.push_back(robot_state.q);
      O_T_EE_data.push_back(robot_state.O_T_EE);
      F_T_EE_data.push_back(robot_state.F_T_EE);

      // Send desired tau to the robot
      return tau_d_array;

      // // Return 0 for debugging
      // std::array<double, 7> return_0 = {0, 0, 0, 0, 0, 0, 0}; 
      // return return_0; 
      
    };

    // start control loop
    robot.control(force_control_callback, cartesian_velocity_callback);

    // Write Data to .txt file
    writeDataToFile(tau_data);
    writeDataToFile(tau_filter_data);
    writeDataToFile(F_robot_data);
    writeDataToFile(joint_position_data);
    writeDataToFile(O_T_EE_data);
    writeDataToFile(F_T_EE_data);
    writeDataToFile(rotation_time_data);
    writeDataToFile(F_sensor_data);
    writeDataToFile(F_sensor_total_data);

  }
  // Catches Exceptions caused within the execution of a program (I think)
  catch (const franka::ControlException& e) {
    std::cout << "Control Exception: \n" << e.what() << std::endl;
    writeDataToFile(tau_data);
    writeDataToFile(tau_filter_data);
    writeDataToFile(F_robot_data);
    writeDataToFile(joint_position_data);
    writeDataToFile(O_T_EE_data);
    writeDataToFile(F_T_EE_data);
    writeDataToFile(rotation_time_data);
    writeDataToFile(F_sensor_data);
    writeDataToFile(F_sensor_total_data);
    return -1;
  }
  // General exceptions
  catch (const franka::Exception& e) {
    std::cout << "Other Exception: \n" << e.what() << std::endl;
    writeDataToFile(tau_data);
    return -1;
  }
  return 0;
}