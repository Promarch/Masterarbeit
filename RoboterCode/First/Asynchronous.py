import time
import numpy as np
from franky import Affine, CartesianMotion, Robot, ReferenceType

robot = Robot("192.168.1.11")
robot.relative_dynamics_factor = 0.05

motion1 = CartesianMotion(Affine([0.0, 0.15, 0.0]), ReferenceType.Relative)
robot.move(motion1, asynchronous=True)
time.sleep(0.1)
print("\nForce orig CoSy:", np.round(robot.state.O_F_ext_hat_K,2))
#print("External Torque, filtered:", np.round(robot.state.tau_ext_hat_filtered,2))
print("Force rel CoSy: ", np.round(robot.state.K_F_ext_hat_K,2))


time.sleep(0.5)
print("\nForce orig CoSy:", np.round(robot.state.O_F_ext_hat_K,2))
#print("External Torque, filtered:", np.round(robot.state.tau_ext_hat_filtered,2))
print("Force rel CoSy: ", np.round(robot.state.K_F_ext_hat_K,2))

time.sleep(0.5)
print("\nForce orig CoSy:", np.round(robot.state.O_F_ext_hat_K,2))
#print("External Torque, filtered:", np.round(robot.state.tau_ext_hat_filtered,2))
print("Force rel CoSy: ", np.round(robot.state.K_F_ext_hat_K,2))

time.sleep(0.5)
motion2 = CartesianMotion(Affine([0.0, -0.1, 0.0]), ReferenceType.Relative)
robot.move(motion2, asynchronous=True)

robot.join_motion()

robot.control_rate()
