from franky import Robot, Measure
import numpy as np

robot = Robot("192.168.1.11")
print("Current Joint position:", np.round(robot.current_joint_positions,2))
#print("Current pose:", robot.current_pose)

print("Current x-force: ", Measure.FORCE_X)
print("Current y-force: ", Measure.FORCE_Y)
print("Current z-force: ", Measure.FORCE_Z)

#print("EoL")