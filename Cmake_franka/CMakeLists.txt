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

add_executable(JointPositions JointPositions.cpp)
target_link_libraries(JointPositions Franka::Franka Eigen3::Eigen3)

include(GNUInstallDirs)
install(TARGETS JointPositions
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)