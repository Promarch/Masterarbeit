cmake_minimum_required(VERSION 3.4)
project(FrankaCmake CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(sensor_botasys STATIC
  BotaForceTorqueSensorComm.cpp
)
add_library(sensor_setup STATIC
  SensorSetUp.cpp
)

SET(Program_file
  sensor_cmake
)

foreach(singleFile ${Program_file})
  add_executable(${singleFile} ${singleFile}.cpp)
  target_link_libraries(${singleFile} sensor_botasys sensor_setup)
endforeach()




# include(GNUInstallDirs)
# install(TARGETS ${singleFile}
#   LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
#   RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
# )
