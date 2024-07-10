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

void setDefaultBehavior(franka::Robot &robot) {

    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
    robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
    robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
};


int main() {
  try {
    // Set Up basic robot function
    franka::Robot robot("192.168.1.11");
    setDefaultBehavior(robot);
    franka::Model model = robot.loadModel();
    franka::RobotState initial_state = robot.readOnce();

    // equilibrium point is the initial position
    Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
    Eigen::Vector3d position_d(initial_transform.translation());
    Eigen::Quaterniond orientation_d(initial_transform.linear());

    Eigen::Matrix<double, 6,1> x_e;
    x_e.head(3) = position_d;
    x_e.tail(3) << orientation_d.x(), orientation_d.y(), orientation_d.z();

    std::cout << "x_e: \n" << x_e << std::endl; 
    std::array<double, 7> gravity_array = model.gravity(initial_state);
    Eigen::Map<Eigen::Matrix<double, 7, 1>> initial_gravity(gravity_array.data());
    std::cout << "gravity: \n" << initial_gravity << std::endl; 
    std::array<double, 7> tau_array = initial_state.tau_J;
    Eigen::Map<Eigen::Matrix<double, 7, 1>> initial_tau(tau_array.data());
    std::array<double, 7> tau_filter_array = initial_state.tau_ext_hat_filtered;
    Eigen::Map<Eigen::Matrix<double, 7, 1>> initial_tau_filter(tau_filter_array.data());
    std::cout << "Tau: \n" << initial_tau-initial_gravity << "\nTau filter: \n" << initial_tau_filter << std::endl; 

  }

  catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }
}