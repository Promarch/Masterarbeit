#%%
import numpy as np
import quaternion
import matplotlib.pyplot as plt
import matplotlib as mpl
import pandas as pd
import glob
import os


def readFile(path):
    list_of_files = glob.glob(path)
    filePath = max(list_of_files, key=os.path.getctime)
    array = np.loadtxt(filePath, delimiter=",")
    return array

def generateMissingAngles(angle_flex, angle_intern, startRange = -30, endRange=-20, tolerance=1):
    allAngles = np.linspace(startRange, endRange, endRange-startRange+1)
    angleMatrix = angle_flex[:, np.newaxis] - allAngles
    boolMatrix = np.abs(angleMatrix)<tolerance
    boolMatrixFilter = (boolMatrix.transpose() * np.sign(angle_intern)).transpose()
    boolValidAnglesIntern = ~np.any(boolMatrixFilter<0, axis=0)
    boolValidAnglesExtern = ~np.any(boolMatrixFilter>0, axis=0)
    validAnglesIntern = allAngles[boolValidAnglesIntern]
    validAnglesExtern = allAngles[boolValidAnglesExtern]

    return validAnglesIntern, validAnglesExtern

folder_path = "/home/alexandergerard/Masterarbeit/Cmake_franka/build/data_output_knee/"
np.set_printoptions(precision=3, suppress=True)

#%%
# quaternion takes w,x,y,z
startRange = -25
endRange=-15 
tolerance=1
array_rot = readFile(folder_path + "quat_stop*")[:,:4]
# array_rot = readFile("/home/alexandergerard/Documents/quat_stop_tests.txt")[:,:4]
array_rot = array_rot[:,[3,0,1,2]]
angle_flex = 2*np.arctan(array_rot[:,1]/array_rot[:,0])*180/np.pi
angle_intern = 2*np.arctan(array_rot[:,2]/array_rot[:,0])*180/np.pi
test = np.array([angle_flex, angle_intern]).transpose()
print(f"These are the x- and y-angles of the quaternion: \n{test}")
validIntern, validExtern = generateMissingAngles(angle_flex, angle_intern, startRange,endRange,tolerance)
print(f"Missing Intern: {validIntern.transpose()} \nMissing Extern: {validExtern.transpose()}")

#%%
# quat_rot = quaternion.from_float_array(array_rot)
# quat_init = np.quaternion(0.0, 0.7071068, 0.7071068, 0.0)
# quat_rot_base = quat_init.inverse() * quat_rot
# array_rot_base = quaternion.as_float_array(quat_rot_base)
array_rot = readFile(folder_path + "quat_stop*")[:,:4]
array_rot = array_rot[:,[3,0,1,2]]
array_rot_base = array_rot
test_flex = np.arctan2(2*(array_rot_base[:,0]*array_rot_base[:,1]+array_rot_base[:,2]*array_rot_base[:,3]), 1-2*(array_rot_base[:,1]*array_rot_base[:,1]+array_rot_base[:,2]*array_rot_base[:,2]))*180/np.pi
test_intern = np.arcsin(2*(array_rot_base[:,0]*array_rot_base[:,2]-array_rot_base[:,3]*array_rot_base[:,1]))*180/np.pi
test_intern = np.arctan2(2*(array_rot_base[:,0]*array_rot_base[:,2]-array_rot_base[:,3]*array_rot_base[:,1]), np.sqrt(1-(2*(array_rot_base[:,0]*array_rot_base[:,2]+array_rot_base[:,3]*array_rot_base[:,1]))**2))*180/np.pi
angle_flex = 2*np.arctan(array_rot_base[:,1]/array_rot_base[:,0])*180/np.pi
angle_intern = 2*np.arctan(array_rot_base[:,2]/array_rot_base[:,0])*180/np.pi
test = np.array([test_flex, angle_flex, test_intern, angle_intern]).transpose()
np.set_printoptions(precision=3, suppress=True)
print(f"These are the x- and y-angles of the quaternion: \n{test}")
#%%
# quaternion takes w,x,y,z
quat_init = np.quaternion(0.0, 0.7071068, 0.7071068, 0.0)
quat_15_0 = np.quaternion(0.9914449, 0.1305262, 0, 0)
quat_0_15 = np.quaternion(0.9914449, 0, 0.1305262, 0)
quat_robot = np.quaternion(0.9914449, 0, 0.1305262, 0)
quat_combined = quat_0_15 * quat_15_0
quat_rel = quat_init * quat_combined * quat_init**-1
quat_d = quat_rel * quat_init
quat_test = quat_init * quat_0_15 * quat_15_0
print(f"quat_comb: {quat_combined}, \nquat_rel: {quat_rel}, \nquat_d: {quat_d}, \nquat_test: {quat_test}")
#%%
quat_5_0 = np.array([ -0.0399001, -0.0002, 0.002, 0.9992017 ])
quat_3_3 = np.array([ 0.026168, 0.026168, 0.0006852, 0.9993148 ])
quat_10_10 = np.array([ 0.0868241, 0.0868241, 0.0075961, 0.9924039 ])
quat_30_30 = np.array([ 0.25, 0.25, 0.0669873, 0.9330127 ])
quat_50_50 = np.array([ 0.3830222, 0.3830222, 0.1786062, 0.8213938 ])
quat_90_90 = np.array([ 0.5, 0.5, 0.5, 0.5 ])
quat = quat_5_0
norm_xyz = np.linalg.norm(quat[:-1])
scaled_xyz = quat[:-1]/np.linalg.norm(quat[:-1])
print(f"Norm of xyz: {np.round(norm_xyz,3)}, scales xyz:{np.round(scaled_xyz,3)}")

#%%
    # Plot sphere
# draw sphere
r = 10
n_fine = 8
theta = np.linspace(0,np.pi/2, n_fine)
phi = np.linspace(0,2*np.pi, n_fine)
u, v = np.meshgrid(theta, phi)
x = r * np.sin(u)*np.cos(v)
y = r * np.sin(u)*np.sin(v)
z = r * np.cos(u)
n_grid = 8
theta_grid = np.linspace(0,np.pi/2, n_grid)[1:]
phi_grid = np.linspace(0,np.pi*2, n_grid)
theta_grid = np.array([np.pi/9, np.pi/8, np.pi/3, np.pi/2])
phi_grid = np.array([np.pi/9, np.pi/8, np.pi/3, np.pi/2, 3*np.pi/2, 3.2*np.pi/2])
u_grid, v_grid = np.meshgrid(theta_grid, phi_grid)
x_grid = r * np.sin(u_grid)*np.cos(v_grid)
y_grid = r * np.sin(u_grid)*np.sin(v_grid)
z_grid = r * np.cos(u_grid)

u_surf, v_surf = np.meshgrid(theta_grid[0:2], phi_grid[1:3])
x_surf = r * np.sin(u_surf)*np.cos(v_surf)
y_surf = r * np.sin(u_surf)*np.sin(v_surf)
z_surf = r * np.cos(u_surf)

fig = plt.figure(figsize=(6,6))
ax = fig.add_subplot(projection="3d")
# ax.plot_surface(x, y, z, alpha = 0.4, cmap=plt.cm.coolwarm, linewidth=0, antialiased=False)
ax.plot_wireframe(x_grid, y_grid, z_grid, alpha=0.7, color="red")
# ax.plot_surface(x_surf, y_surf, z_surf, alpha = 1, color="black")
lim = (-10,10)
ax.set_xlim(lim)
ax.set_ylim(lim)
ax.set_zlim(lim)

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
# plt.show()

#%%
    # testing meshgrids

r=10
n_grid = 4
theta_grid = np.arange(n_grid)
phi_grid = np.arange(10, 10 + n_grid)
u_grid, v_grid = np.meshgrid(theta_grid, phi_grid)
x_grid = r * np.sin(u_grid)*np.cos(v_grid)
y_grid = r * np.sin(u_grid)*np.sin(v_grid)
z_grid = r * np.cos(u_grid)

# Try some colors
test = np.arange((n_grid-1)**2).reshape((n_grid-1), (n_grid-1))
norm = mpl.colors.Normalize(vmin=test.min(), vmax=test.max())
norm(test)
print(test)

# Plot some points
pts_start = 1
pts = 3

fig = plt.figure(figsize=(6,6))
ax = fig.add_subplot(projection="3d")
ax.plot_surface(x_grid, y_grid, z_grid, facecolors=plt.cm.jet(norm(test))) #, facecolors = colors

# ax.plot_surface(x_grid[pts_start:pts], y_grid[pts_start:pts], z_grid[pts_start:pts], alpha=0.7, color = "c")

m = mpl.cm.ScalarMappable(cmap=plt.cm.jet, norm=norm)
m.set_array([])
plt.colorbar(m)

lim = (-10,10)
ax.set_xlim(lim)
ax.set_ylim(lim)
ax.set_zlim(lim)

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.show()
