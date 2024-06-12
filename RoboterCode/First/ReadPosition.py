import franky
import numpy as np

robot = franky.Robot("192.168.1.11")

# Get positions and stuff
q = np.round(robot.current_joint_positions,2)
transformation_matrix_vec = robot.state.O_T_EE
trans_matrix = np.array(transformation_matrix_vec).reshape(4,4).transpose()
rot_matrix = trans_matrix[:3,:3]
z_rot = np.arctan2(rot_matrix[1,0], rot_matrix[0,0])
y_rot = np.arctan2(-rot_matrix[2, 0], np.sqrt(rot_matrix[2, 1]**2 + rot_matrix[2, 2]**2))
x_rot = np.arctan2(rot_matrix[2, 1], rot_matrix[2, 2])

print("Current Joint position:", q)
print("Current Cartesian position:", robot.current_pose)
print("Translation matrix: \n", np.round(trans_matrix,3))
print(f"X-Rotation: {np.rad2deg(x_rot)}, y-rotation: {np.rad2deg(y_rot)}, z-rotation: {np.rad2deg(z_rot)}")


#print("Current x-force: ", Measure.FORCE_X)
#print("Current y-force: ", Measure.FORCE_Y)
#print("Current z-force: ", Measure.FORCE_Z)

print("EoL")