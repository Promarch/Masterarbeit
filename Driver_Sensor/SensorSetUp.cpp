// C library headers
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include <chrono>
#include <iostream>
#include <array>
#include <vector>
#include <thread>

// Linux headers
#include <fcntl.h>      // Contains file controls like O_RDWR
#include <errno.h>      // Error integer and strerror() function
#include <termios.h>    // Contains POSIX terminal control definitions
#include <unistd.h>     // write(), read(), close()
#include <sys/ioctl.h>
#include <linux/serial.h>

#include "BotaForceTorqueSensorComm.h"
#include "SensorSetUp.h"




void set_up_serial(int _serial_port){

    /* Open the serial port. Change device path as needed.
     */
    printf("Open serial port.\n");
    _serial_port = open("/dev/ttyUSB0", O_RDWR);
    printf("Opened port %i.\n",_serial_port);

    if (_serial_port < 0) {
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
    if(tcgetattr(_serial_port, &tty) != 0) {
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
    if (tcsetattr(_serial_port, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    }

    // Enable linux FTDI low latency mode
    ioctl(_serial_port, TIOCGSERIAL, &ser_info);
    ser_info.flags |= ASYNC_LOW_LATENCY;
    ioctl(_serial_port, TIOCSSERIAL, &ser_info);
    printf("Finished serial port set up\n");
}

void check_sensor(myBotaForceTorqueSensorComm _sensor){

    // Variables for debug/ensure the program cancels correctly
    int counter=0;  // Counts how many times the sensor could be read
    auto t_start = std::chrono::high_resolution_clock::now();
    auto t_current = std::chrono::high_resolution_clock::now();
    auto t_limit = std::chrono::seconds(5);
    auto elapsedTime = std::chrono::duration_cast<std::chrono::seconds>(t_current - t_start);

    while (counter<20) {
      switch(_sensor.readFrame())
      {
        case BotaForceTorqueSensorComm::VALID_FRAME:
          if (_sensor.frame.data.status.val>0)
          {
            printf("No valid forces:\n");
            printf(" app_took_too_long: %i\n",_sensor.frame.data.status.app_took_too_long);
            printf(" overrange: %i\n",_sensor.frame.data.status.overrange);
            printf(" invalid_measurements: %i\n",_sensor.frame.data.status.invalid_measurements);
            printf(" raw_measurements: %i\n",_sensor.frame.data.status.raw_measurements);
          }
          else
          {
            // for (uint8_t i=0; i<6; i++)
            // {
            //   printf("%f",sensor.frame.data.forces[i]);
            //   printf("\t");
            // }
            counter++;
            printf("Counter: %i\n", counter);
          }
          break;
        case BotaForceTorqueSensorComm::NOT_VALID_FRAME:
          printf("No valid frame: %i\n",_sensor.get_crc_count());
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
}


