cmake_minimum_required(VERSION 3.4)
project(FrankaCmake CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_CURRENT_LIST_DIR}/cmake)

if(NOT FRANKA_IS_FOUND)
    find_package(Franka REQUIRED)
endif()
find_package(Eigen3 REQUIRED)
find_package(Poco REQUIRED COMPONENTS Foundation)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

add_library(examples_common STATIC
  cmake/examples_common.cpp
)

add_library(sensor_botasys STATIC
  cmake/BotaForceTorqueSensorComm.cpp
)

target_link_libraries(examples_common PUBLIC Franka::Franka Eigen3::Eigen3)

SET(PROGRAM_files
  trystuffCmake
  GetPosition
  Waypoints
  ReadForce
  HandMoveRobot
  ImpedanceCartesianControl
  First_ImpedanceVel
  ImpedanceJointVel_test
  InverseKin_test
  TrueImpedance_test
  OpSpace
  VelImpedance_test
)

foreach(singleFile ${PROGRAM_files})
  add_executable(${singleFile} ${singleFile}.cpp)
  target_link_libraries(${singleFile} Franka::Franka examples_common Eigen3::Eigen3 sensor_botasys)
endforeach()

include(GNUInstallDirs)
install(TARGETS ${singleFile}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# target_link_libraries(JointImpedanceControl_example Threads::Threads)
target_link_libraries(HandMoveRobot Threads::Threads)