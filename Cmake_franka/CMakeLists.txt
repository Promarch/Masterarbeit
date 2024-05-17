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

target_link_libraries(examples_common PUBLIC Franka::Franka Eigen3::Eigen3)

SET(PROGRAM_files
  JointPositions
  JointVelocity
  JointPositions2
  Waypoints
  JointImpedanceControl
  JointImpedanceControl_example
)

foreach(singleFile ${PROGRAM_files})
  add_executable(${singleFile} ${singleFile}.cpp)
  target_link_libraries(${singleFile} Franka::Franka examples_common Eigen3::Eigen3)    
endforeach()

include(GNUInstallDirs)
install(TARGETS ${singleFile}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

target_link_libraries(JointImpedanceControl_example Threads::Threads)