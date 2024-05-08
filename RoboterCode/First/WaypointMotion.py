import math
import numpy as np
import time
from franky import JointWaypointMotion, JointWaypoint, Robot, JointPositionStopMotion, Measure, RobotPose

robot = Robot("192.168.1.11")
robot.relative_dynamics_factor = 0.6


pos1 = [0.23, -0.75, 0.72, -2.63, 0, 1.86, 1.21]
pos2 = [0, -0.77, 0, -2.64, 0, 1.8, 0.6]
pos3 = [-0.32, -0.23, -0.33, -2.68, 0.09, 2.27, 0.43]
pos4 = [0, -0.89, 0, -3.06, 0.1, 2.2, 0.76]

m1 = JointWaypointMotion([
    JointWaypoint(pos1),
    JointWaypoint(pos2),
    JointWaypoint(pos3),
])
stopMotion = JointPositionStopMotion()

for i in range(2):
    robot.move(m1)
    # print("Force:", np.round(robot.state.tau_J_d,1))
    robot.move(stopMotion)
    time.sleep(0.3)
