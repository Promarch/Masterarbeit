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

// Macro to convert variable name to string for the function
#define writeDataToFile(data) writeDataToFileImpl(data, #data)
// Get current time; used to add the date to the file writer
std::string getCurrentDateTime() {
  std::time_t now = std::time(nullptr);
  char buf[80];
  std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", std::localtime(&now));
  return std::string(buf);
}
// Function to write all elements of a vector to a text file
template <typename T, std::size_t N>
void writeDataToFileImpl(const std::vector<std::array<T, N>>& data, const std::string& var_name) {
  std::string filename = "data_grav/" + var_name + "_" + getCurrentDateTime() + ".txt";
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


int main() {

  // Variables to store the torque or position of the robot during the movement
  std::vector<std::array<double, 7>> tau_grav_data, tau_filter_data; 
  std::vector<std::array<double, 6>> F_tau_grav_data, F_tau_filter_data, F_ext_data, F_grav_data; 
  std::vector<std::array<float, 6>> F_sensor_data;

    // Sensor set up
  // Serial port set up
  set_up_serial();
  // Set up sensor
  check_sensor();

  // Get starting values of the external sensor
  float F_sensor_start_aligned[6];
  std::memcpy(F_sensor_start_aligned, sensor.frame.data.forces, 6 * sizeof(float));
  Eigen::Map<Eigen::Matrix<float, 1, 6>> F_sensor_start(F_sensor_start_aligned);
  std::cout << "Starting sensor force: " << F_sensor_start << "\n";

  try {
    // Set Up basic robot function
    franka::Robot robot("192.168.1.11");
    // Set new end-effector position
    // robot.setEE({1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.245, 1.0});
    // robot.setK({1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -0.0105, 1.0});
    // robot.setEE({1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0});
    robot.setK({1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0});
    
    setDefaultBehavior(robot);
    franka::Model model = robot.loadModel();
    franka::RobotState initial_state = robot.readOnce();

    // Get starting robot sensor values
    std::array<double, 6> F_ext_start_array = initial_state.K_F_ext_hat_K;
    Eigen::Map<Eigen::Matrix<double, 1, 6>> F_ext_start(F_ext_start_array.data());

    // Variables to control loop time
    double time{0};
    double time_max{5};
    double sampling_interval = 0.25; // Interval at which the console outputs the current error
    double next_sampling_time = 0;  // Needed for the sampling interval

    // Joa mal schauen
    std::array<float, 6> F_sensor_array;


    auto force_control_callback = [&] (const franka::RobotState& robot_state, franka::Duration period) -> franka::Torques {
      
      time += period.toSec();

        // Get state variables
      // Jacobian
      std::array<double, 42> jacobian_array = model.bodyJacobian(franka::Frame::kEndEffector, robot_state);
      Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
      // Tau with gravity
      std::array<double, 7> tau_j_array = robot_state.tau_J; 
      Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J(tau_j_array.data()); 
      // Gravity on each joint
      std::array<double, 7> gravity_array = model.gravity(robot_state);
      Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());
      // Tau filtered
      std::array<double, 7> tau_filter_array = robot_state.tau_ext_hat_filtered;
      Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_filter(tau_filter_array.data());
      // Forces by robot sensor
      std::array<double, 6> F_ext_array = robot_state.K_F_ext_hat_K;
      Eigen::Map<Eigen::Matrix<double, 1, 6>> F_ext(F_ext_array.data());
      // External Sensor (using BotaSys header)
      alignas(alignof(float[6])) float aligned_forces[6]; // necessary cause otherwise i get the warning: taking address of packed member of ‘BotaForceTorqueSensorComm::AppOutput::<unnamed struct>’ may result in an unaligned pointer value
      if (sensor.readFrame() == BotaForceTorqueSensorComm::VALID_FRAME){
        std::memcpy(aligned_forces, sensor.frame.data.forces, sizeof(aligned_forces));
        std::copy(aligned_forces, aligned_forces + 6, F_sensor_array.begin());
        F_sensor_data.push_back(F_sensor_array);
      }
      Eigen::Map<Eigen::Matrix<float, 1, 6>> F_sensor(F_sensor_array.data());

        // Compute stuff
      Eigen::Matrix<double, 7, 1> tau_grav = tau_J-gravity;
      // Eigen::Matrix<double, 6, 1> F_tau_filter = jacobian.transpose().completeOrthogonalDecomposition().pseudoInverse() * tau_filter; 

        // Map the Eigen vectors to an array
      std::array<double, 7> tau_grav_array{}; 
      Eigen::VectorXd::Map(&tau_grav_array[0], 7) = tau_grav;
      // std::array<double, 6> F_tau_filter_array{}; 
      // Eigen::VectorXd::Map(&F_tau_filter_array[0], 6) = F_tau_filter;

        // Add current measurements to the vector  
      tau_grav_data.push_back(tau_grav_array);
      tau_filter_data.push_back(tau_filter_array);
      F_ext_data.push_back(robot_state.K_F_ext_hat_K);
      // F_tau_filter_data.push_back(F_tau_filter_array);

      // Print current wrench, time, and absolute positional error
      if (time >= next_sampling_time) {
        std::cout << std::fixed << std::setprecision(3);
        // for (uint8_t i=0; i<6; i++)
        // {
        //   printf("%f",F_sensor_array[i]);
        //   printf("\t");
        // }
        // printf("\n");
        std::cout << "Time: " << time << "\nForce sensor: " << F_sensor-F_sensor_start << "\nForce robot:  " << F_ext-F_ext_start << "\n \n"; 
        next_sampling_time += sampling_interval;
      }

        // Send torque = 0 to ensure no movement
      std::array<double, 7> tau_d = {0,0,0,0,0,0,0};
      if (time >= time_max) {
        franka::Torques output = tau_d;
        return franka::MotionFinished(output);
      }
      return tau_d;
    };

    // start control loop
    robot.control(force_control_callback);

    std::cout << "Length of F_sensor_data: " << F_sensor_data.size() << ", entries per second: " << F_sensor_data.size()/time_max << std::endl;

      // Write Data to .txt file
    // writeDataToFile(tau_grav_data);
    // writeDataToFile(tau_filter_data);
    // writeDataToFile(F_sensor_data);
    // // writeDataToFile(F_tau_filter_data);
    // writeDataToFile(F_ext_data);

    printf("\nClose serial port.\n");
    close(serial_port);

  }
  // Catches Exceptions caused within the execution of a program (I think)
  catch (const franka::ControlException& e) {
    std::cout << e.what() << std::endl;
    writeDataToFile(tau_grav_data);
    writeDataToFile(tau_filter_data);
    // writeDataToFile(F_tau_filter_data);
    writeDataToFile(F_ext_data);

    return -1;
  }
  // General exceptions
  catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }
  
}
