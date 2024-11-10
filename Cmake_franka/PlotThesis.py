#%%
import glob
import os

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def readFile(path):
    list_of_files = glob.glob(path)
    filePath = max(list_of_files, key=os.path.getctime)
    array = np.loadtxt(filePath, delimiter=",")
    return array

#%% Comparison impedance control vs simulation

def matrix2euler(rot_matrix):
    # Ensure that the input is a valid 3x3 rotation matrix
    assert rot_matrix.shape[1:] == (3, 3), "Each matrix must be a 3x3 rotation matrix."    

    # Extract the relevant elements from the rotation matrix
    R32 = rot_matrix[:, 2, 1]  
    R33 = rot_matrix[:, 2, 2]  
    R31 = rot_matrix[:, 2, 0]  
    R21 = rot_matrix[:, 1, 0]  
    R11 = rot_matrix[:, 0, 0]  
    
    # Compute Euler angles (roll, pitch, yaw) using vectorized operations
    rot_x = np.arctan2(R32, R33)  # rotation around the x-axis
    rot_y = np.arctan2(-R31, np.sqrt(R32**2 + R33**2))  # rotation around the y-axis
    rot_z = np.arctan2(R21, R11)  # rotation around the z-axis
    
    # Stack the Euler angles into a single array with shape (n, 3)
    euler_angles = np.stack((rot_x, rot_y, rot_z), axis=1)*180/np.pi

    return euler_angles

# Read in files
folder_path_libfranka = "build/data_thesis/ImpedanceVsLibfranka/Libfranka/"
folder_path_normal = "build/data_thesis/ImpedanceVsLibfranka/Normal/"
O_T_EE_franka = readFile(folder_path_libfranka + "O_T_EE_data*")
O_T_EE_normal = readFile(folder_path_normal + "O_T_EE_da*")
# Interpolate sim variables since the sampling rate is only ~250Hz 
size_franka = len(O_T_EE_franka)
size_normal = len(O_T_EE_normal)
indices_normal = np.arange(size_normal)
indices_full = np.linspace(0, size_normal - 1, size_franka)
O_T_EE_normal_interpol = np.zeros((size_franka, 16))
for i in range(O_T_EE_normal_interpol.shape[1]):
    O_T_EE_normal_interpol[:, i] = np.interp(indices_full, indices_normal, O_T_EE_normal[:, i])
# Transform to array of transformation matrices
O_T_EE_mat_franka = np.transpose(O_T_EE_franka.reshape(size_franka,4,4), axes=(0,2,1))
O_T_EE_mat_normal = np.transpose(O_T_EE_normal_interpol.reshape(size_franka,4,4), axes=(0,2,1))
# Transform from base CoSy to sensor CoSy
rotation_O_local = np.array([[0,1,0,0], [1,0,0,0], [0,0,-1,0], [0,0,0,1]])
local_T_EE_franka = np.linalg.inv(rotation_O_local)@O_T_EE_mat_franka
local_T_EE_normal = np.linalg.inv(rotation_O_local)@O_T_EE_mat_normal

# --------------------------------------------------------
# -----              Positional error                -----
# --------------------------------------------------------
# Calculate absolute positional error
pos_franka = np.array([O_T_EE_franka[:,12], O_T_EE_franka[:,13], O_T_EE_franka[:,14]])
pos_abs_franka = np.linalg.norm(pos_franka.transpose()-pos_franka[:,0], axis=1)*1000
pos_normal = np.array([O_T_EE_normal_interpol[:,12], O_T_EE_normal_interpol[:,13], O_T_EE_normal_interpol[:,14]])
pos_abs_normal = np.linalg.norm(pos_normal.transpose()-pos_normal[:,0], axis=1)*1000
# Plot position
# x = np.arange(len(O_T_EE_franka))/1000
# fig, ax = plt.subplots(1,1,figsize=(7,3))
# ax.plot(x, pos_abs_franka, label = "robot")
# ax.plot(x, pos_abs_normal, label = "simulation")
# ax.grid(True)
# plt.legend()
# plt.xlabel("Time [s]")
# plt.ylabel("Error [mm]")
# plt.title("Absolute positional error")
# plt.show()


# --------------------------------------------------------
# -----               Rotation error                 -----
# --------------------------------------------------------
# Get euler angles from transformation matrix
euler_angles_franka = matrix2euler(local_T_EE_franka[:,:3,:3])
euler_angles_normal = matrix2euler(local_T_EE_normal[:,:3,:3])
ylim_top = 2
ylim_bottom = -22

nEntries = 5000
ylim_top = 22
ylim_bottom = -2
x = np.arange(len(O_T_EE_franka))/1000
fig, ax = plt.subplots(1,2, figsize=(10,4))
ax[0].plot(x[:nEntries], euler_angles_franka[:nEntries,0]+20, label="robot")
ax[0].plot(x[:nEntries], euler_angles_normal[:nEntries,0]+20, label="simulation")
ax[0].set_ylabel(r"Angle $\phi_x$ [°]")
ax[0].set_xlabel("Time [s]")
ax[0].set_ylim(ylim_bottom, ylim_top)
ax[0].grid(True)
ax[0].set_title(r"Rotation error $\Delta \phi_x$")
ax[1].plot(x[:nEntries], pos_abs_franka[:nEntries], label = "robot")
ax[1].plot(x[:nEntries], pos_abs_normal[:nEntries], label = "simulation")
ax[1].grid(True)
ax[1].yaxis.tick_right()
ax[1].yaxis.set_label_position("right")
ax[1].yaxis.label.set_rotation(-90)
ax[1].set_xlabel("Time [s]")
ax[1].set_ylabel("Error [mm]")
ax[1].set_title("Absolute positional error")
ax[1].legend()
plt.show()

#%% F_robot during movement
    # Plot Forces
folder_path = "/home/alexandergerard/Masterarbeit/Cmake_franka/build/data_thesis/"
F_robot = readFile(folder_path + 'F_robot_*')
x = np.arange(len(F_robot))/1000
fig = plt.figure(figsize=(8,4))
ax_force = fig.add_subplot(211)
ax_force.plot(x, F_robot[:,0], 'r', label="$F_x$")
ax_force.plot(x, F_robot[:,1], 'g', label="$F_y$")
ax_force.plot(x, F_robot[:,2], 'b', label="$F_z$")
ax_force.get_xaxis().set_ticklabels([])
ax_force.grid(True)
ax_force.legend(loc = "upper right")
ax_force.set_ylabel("Forces [N]")
    # Plot torque
ax_torque = fig.add_subplot(212)
ax_torque.plot(x, F_robot[:,3], 'r', label=r"$\tau _x$")
ax_torque.plot(x, F_robot[:,4], 'g', label=r"$\tau _y$")
ax_torque.plot(x, F_robot[:,5], 'b', label=r"$\tau _z$")
ax_torque.grid(True)
ax_torque.legend(loc = "upper right")
ax_torque.set_xlabel("Time [s]")
ax_torque.set_ylabel("Torques [Nm]")

fig.suptitle("Sensor measurements")
plt.show()
#%% Comparison F_ext vs F_sensor

folder_path = "/home/alexandergerard/Masterarbeit/Cmake_franka/build/data_thesis/"

F_robot = readFile(folder_path+"F_robot*")
F_sensor = readFile(folder_path+"F_sensor_total*")
x = np.arange(len(F_robot))/1000
labels_axis = [r"$F_x$", r"$F_y$", r"$F_z$", r"$\tau_x$", r"$\tau_y$", r"$\tau_z$"]

min_F = np.min([F_robot[:,:3], F_sensor[:,:3]])
max_F = np.max([F_robot[:,:3], F_sensor[:,:3]])
max_F_abs = np.max([np.abs(min_F),max_F])

min_tau = np.min([F_robot[:,3:], F_sensor[:,3:]])
max_tau = np.max([F_robot[:,3:], F_sensor[:,3:]])
max_tau_abs = np.max([np.abs(min_tau),max_tau])

fig = plt.figure(figsize=(6,6))
ax = fig.subplots(3,2, sharex=True)
for i in range(6):
    ax[i%3, round(np.floor(i/3))].plot(x, F_robot[:,i], label=r"$F_{robot}$")
    ax[i%3, round(np.floor(i/3))].plot(x, F_sensor[:,i], label=r"$F_{sensor}$")
    ax[i%3, round(np.floor(i/3))].set_ylabel(labels_axis[i])
    ax[i%3, round(np.floor(i/3))].grid(True)
for i in range(3):
    ax[i,0].set_ylim(-max_F_abs, max_F_abs)
    ax[i,1].set_ylim(-max_tau_abs, max_tau_abs)
    ax[i,0].yaxis.label.set_verticalalignment("center")
    ax[i,0].set_ylabel(labels_axis[i], labelpad=0)
    # ax[i,0].set_ylim(min_F,max_F)
    # ax[i,1].set_ylim(min_tau,max_tau)
    ax[i,1].yaxis.tick_right()
    ax[i,1].yaxis.set_label_position("right")
    ax[i,1].yaxis.label.set_rotation(-90)

ax[0,1].legend(loc = "upper right")
# ax[0,0].set_title("Force [N]")
# ax[0,1].set_title("Torque [Nm]")
ax[2, 0].set_xlabel("Time [s]")
ax[2, 1].set_xlabel("Time [s]")
fig.text(0.03, 0.5, "Forces [N]", va='center', ha='center', rotation='vertical')
fig.text(0.97, 0.5, "Torque [Nm]", va='center', ha='center', rotation=-90)
fig.suptitle("Comparison between external and internal Sensor")
plt.tight_layout(rect=[0.03, 0, 0.97, 1]) #rect=[0.03, 0, 0.97, 1]
# plt.show()


#%% Comparison ext torque vs commanded torque
# --------------------------------------------------------
# -----              tau_ext vs tau_c                -----
# --------------------------------------------------------
folder_path = "/home/alexandergerard/Masterarbeit/Cmake_franka/build/data_thesis/"
tau_c = readFile(folder_path+"tau_data*")
tau_ext = readFile(folder_path+"tau_filter_data*")
tau_array = [tau_c, tau_ext]
labels = [r"$\tau_{c}$", r"$\tau_{ext}$"]

x = np.arange(len(tau_array[0]))/1000
# Get min and max of torque
min_tau = np.min(tau_array)
max_tau = np.max(tau_array)
max_tau_abs = np.max([np.abs(min_tau), max_tau])

fig, axs = plt.subplots(4, 2, figsize=(8, 10), sharex="col")

for i in range(4):
    axs[i,0].plot(x, tau_c[:,i], label = r"$\tau_{c}$")
    axs[i,0].plot(x, tau_ext[:,i], label = r"$\tau_{sensor}$")
    axs[i,0].set_ylabel(f"Joint {i+1}", labelpad=-2)
    axs[i,0].set_ylim(min_tau-.2, max_tau+.2)
    axs[i,0].set_ylim(-max_tau_abs-.2, max_tau_abs+.2)
    axs[i,0].grid(True)
for i in range(3):
    axs[i,1].plot(x, tau_c[:,4+i], label = r"$\tau_{c}$")
    axs[i,1].plot(x, tau_ext[:,4+i], label = r"$\tau_{sensor}$")
    axs[i,1].set_ylabel(f"Joint {5+i}", labelpad=8)
    axs[i,1].set_ylim(min_tau-.2, max_tau+.2)
    axs[i,1].set_ylim(-max_tau_abs-.2, max_tau_abs+.2)
    axs[i,1].grid(True)
    axs[i,1].yaxis.tick_right()
    axs[i,1].yaxis.set_label_position("right")
    axs[i,1].yaxis.label.set_rotation(-90)
axs[0,1].legend(loc = "upper right")
axs[3, 0].set_xlabel("time [s]")
axs[2, 1].tick_params(labelbottom=True)
axs[2, 1].set_xlabel("time [s]")
axs[3, 1].axis('off')

# Set the y-axis label for the whole figure
fig.text(0.03, 0.5, 'Torque [Nm]', va='center', rotation='vertical')
plt.tight_layout(rect=[0.05,0,1,0.97])
fig.suptitle("Torque of each joint")
# plt.show()

#%% Comparison impedance control vs simulation

def matrix2euler(rot_matrix):
    # Ensure that the input is a valid 3x3 rotation matrix
    assert rot_matrix.shape[1:] == (3, 3), "Each matrix must be a 3x3 rotation matrix."    

    # Extract the relevant elements from the rotation matrix
    R32 = rot_matrix[:, 2, 1]  
    R33 = rot_matrix[:, 2, 2]  
    R31 = rot_matrix[:, 2, 0]  
    R21 = rot_matrix[:, 1, 0]  
    R11 = rot_matrix[:, 0, 0]  
    
    # Compute Euler angles (roll, pitch, yaw) using vectorized operations
    rot_x = np.arctan2(R32, R33)  # rotation around the x-axis
    rot_y = np.arctan2(-R31, np.sqrt(R32**2 + R33**2))  # rotation around the y-axis
    rot_z = np.arctan2(R21, R11)  # rotation around the z-axis
    
    # Stack the Euler angles into a single array with shape (n, 3)
    euler_angles = np.stack((rot_x, rot_y, rot_z), axis=1)*180/np.pi

    return euler_angles

# Read in files
folder_path = "build/data_thesis/TrueImpedanceVsSim/"
filePath_sim = "/home/alexandergerard/Masterarbeit/mjctrl/O_T_EE.txt"
O_T_EE_robot = readFile(folder_path + "O_T_EE_data*")
O_T_EE_sim = readFile(folder_path + "O_T_EE_sim*")
# Interpolate sim variables since the sampling rate is only ~250Hz 
size_robot = len(O_T_EE_robot)
size_sim = len(O_T_EE_sim)
indices_sim = np.arange(size_sim)
indices_full = np.linspace(0, size_sim - 1, size_robot)
O_T_EE_sim_interpol = np.zeros((size_robot, 16))
for i in range(O_T_EE_sim_interpol.shape[1]):
    O_T_EE_sim_interpol[:, i] = np.interp(indices_full, indices_sim, O_T_EE_sim[:, i])
# Transform to array of transformation matrices
O_T_EE_mat_robot = np.transpose(O_T_EE_robot.reshape(size_robot,4,4), axes=(0,2,1))
O_T_EE_mat_sim = np.transpose(O_T_EE_sim_interpol.reshape(size_robot,4,4), axes=(0,2,1))
# Transform from base CoSy to sensor CoSy
rotation_O_local = np.array([[0,1,0,0], [1,0,0,0], [0,0,-1,0], [0,0,0,1]])
local_T_EE_robot = np.linalg.inv(rotation_O_local)@O_T_EE_mat_robot
local_T_EE_sim = np.linalg.inv(rotation_O_local)@O_T_EE_mat_sim

# --------------------------------------------------------
# -----              Positional error                -----
# --------------------------------------------------------
# Calculate absolute positional error
pos_robot = np.array([O_T_EE_robot[:,12], O_T_EE_robot[:,13], O_T_EE_robot[:,14]])
pos_abs_robot = np.linalg.norm(pos_robot.transpose()-pos_robot[:,0], axis=1)*1000
pos_sim = np.array([O_T_EE_sim_interpol[:,12], O_T_EE_sim_interpol[:,13], O_T_EE_sim_interpol[:,14]])
pos_abs_sim = np.linalg.norm(pos_sim.transpose()-pos_sim[:,0], axis=1)*1000
# Plot position
# x = np.arange(len(O_T_EE_robot))/1000
# fig, ax = plt.subplots(1,1,figsize=(7,3))
# ax.plot(x, pos_abs_robot, label = "robot")
# ax.plot(x, pos_abs_sim, label = "simulation")
# ax.grid(True)
# plt.legend()
# plt.xlabel("Time [s]")
# plt.ylabel("Error [mm]")
# plt.title("Absolute positional error")
# plt.show()


# --------------------------------------------------------
# -----               Rotation error                 -----
# --------------------------------------------------------
# Get euler angles from transformation matrix
euler_angles_robot = matrix2euler(local_T_EE_robot[:,:3,:3])
euler_angles_sim = matrix2euler(local_T_EE_sim[:,:3,:3])
ylim_top = 2
ylim_bottom = -22
# fig, axs = plt.subplots(3, 1, figsize=(7,9), sharex=True)
# x = np.arange(size_robot)/1000
# axs[0].plot(x, euler_angles_robot[:,0], label="robot")
# axs[0].plot(x, euler_angles_sim[:,0], label="simulation")
# axs[0].set_ylabel(r"$\phi_x$")
# axs[0].set_ylim(ylim_bottom, ylim_top)
# axs[0].legend()
# axs[0].grid(True)
# axs[1].plot(x, euler_angles_robot[:,1], label="rot_y_robot")
# axs[1].plot(x, euler_angles_sim[:,1], label="rot_y_sim")
# axs[1].set_ylim(ylim_bottom, ylim_top)
# axs[1].set_ylabel(r"$\phi_y$")
# axs[1].grid(True)
# axs[2].plot(x, euler_angles_robot[:,2], label="rot_z_robot")
# axs[2].plot(x, euler_angles_sim[:,2], label="rot_z_sim")
# axs[2].set_ylim(ylim_bottom, ylim_top)
# axs[2].set_ylabel(r"$\phi_z$")
# axs[2].set_xlabel("Time [s]")
# axs[2].grid(True)
# # Set the y-axis label for the whole figure
# fig.text(0.03, 0.5, 'Rotation [°]', va='center', rotation='vertical')
# fig.tight_layout()
# plt.tight_layout(rect=[0.03,0,1,0.96])
# fig.suptitle("Rotation error")
# plt.show()

nEntries = 5000
ylim_top = 22
ylim_bottom = -2
x = np.arange(len(O_T_EE_robot))/1000
fig, ax = plt.subplots(1,2, figsize=(10,4))
ax[0].plot(x[:nEntries], euler_angles_robot[:nEntries,0]+20, label="robot")
ax[0].plot(x[:nEntries], euler_angles_sim[:nEntries,0]+20, label="simulation")
ax[0].set_ylabel(r"Angle $\phi_x$ [°]")
ax[0].set_xlabel("Time [s]")
ax[0].set_ylim(ylim_bottom, ylim_top)
ax[0].grid(True)
ax[0].set_title(r"Rotation error $\Delta \phi_x$")
ax[1].plot(x[:nEntries], pos_abs_robot[:nEntries], label = "robot")
ax[1].plot(x[:nEntries], pos_abs_sim[:nEntries], label = "simulation")
ax[1].grid(True)
ax[1].yaxis.tick_right()
ax[1].yaxis.set_label_position("right")
ax[1].yaxis.label.set_rotation(-90)
ax[1].set_xlabel("Time [s]")
ax[1].set_ylabel("Error [mm]")
ax[1].set_title("Absolute positional error")
ax[1].legend()
# plt.show()


#%% Comparison impedance control vs simulation

folder_path = "/home/alexandergerard/Masterarbeit/Cmake_franka/build/data_thesis/TrueImpedance/"
O_T_EE_robot = readFile(folder_path + "O_T_EE*")
pathSim_O_T_EE = "/home/alexandergerard/Masterarbeit/mjctrl/output.txt"
O_T_EE_sim = np.loadtxt(pathSim_O_T_EE, delimiter=",")
indices_sim = np.arange(len(O_T_EE_sim))
pos_robot = np.array([O_T_EE_robot[:,12], O_T_EE_robot[:,13], O_T_EE_robot[:,14]])
pos_abs_robot = np.linalg.norm(pos_robot.transpose()-pos_robot[:,0], axis=1)*1000
indices_full = np.arange(len(O_T_EE_robot))
O_T_sim_interpol = np.zeros((len(O_T_EE_robot), 3))
for i in range(O_T_EE_sim.shape[1]):
    O_T_sim_interpol[:, i] = np.interp(indices_full, indices_sim, O_T_EE_sim[:, i])
pos_abs_sim = np.linalg.norm(O_T_sim_interpol-O_T_sim_interpol[0,:], axis=1)*1000

plt.plot(pos_abs_robot, label="pos_robot")
plt.plot(pos_abs_sim, label="pos_sim")
plt.legend()
plt.xlabel("Time [s]")
plt.ylabel("Position error [mm]")
plt.title("Absolute positional error during the trajectory")
plt.show()

#%% Difference Nullspace
# --------------------------------------------------------
# -----                Nullspace Plot                -----
# --------------------------------------------------------

    # Get data
folder_path = '/home/alexandergerard/Masterarbeit/Cmake_franka/build/nullspace_test/'
# Position without nullspace
pos_norm = readFile(folder_path+'position_data_20240809_170329.txt')
# Position with nullspace 
pos_null_100 = readFile(folder_path+'position_data_20240809_170454.txt')
# Position with nullspace 
pos_null_400 = readFile(folder_path+'position_data_20240809_173340.txt')
# Position with nullspace 
pos_null_1000 = readFile(folder_path+'position_data_20240809_173915.txt')
# Position with nullspace 
pos_null_300 = readFile(folder_path+'position_data_20240809_180014.txt')


    # Calculate absolute position error
factor_mm = 1000
distance_norm = np.linalg.norm(pos_norm-pos_norm[0,:], axis=1)*factor_mm
distance_null_100 = np.linalg.norm(pos_null_100-pos_null_100[0,:], axis=1)*factor_mm
distance_null_400 = np.linalg.norm(pos_null_400-pos_null_400[0,:], axis=1)*factor_mm
distance_null_1000 = np.linalg.norm(pos_null_1000-pos_null_1000[0,:], axis=1)*factor_mm
distance_null_300 = np.linalg.norm(pos_null_300-pos_null_300[0,:], axis=1)*factor_mm

fig = plt.figure(figsize=(5,3))

lim5 = 5000
x = np.arange(0,len(distance_norm))[:lim5]/1000
plt.plot(x, distance_norm[:lim5], label = "normal")
plt.plot(x, distance_null_100[:lim5], label = r"$K_n = 100$")
plt.plot(x, distance_null_400[:lim5], label = r"$K_n = 400$")
plt.plot(np.arange(0,len(distance_null_1000))/1000, distance_null_1000, label = r"$K_n = 1000$")
# plt.plot(distance_null_300, label = r"Ausrichtung")
plt.grid(True)
plt.legend()
plt.title("Absolute positional error")
plt.xlabel("Time [s]")
plt.ylabel("Error [mm]")
# plt.show()

#%% Acceleration factor
# --------------------------------------------------------
# -----            Beschleunigungsfaktor             -----
# --------------------------------------------------------

period_acc = 1
t = np.linspace(0,period_acc,100)
y_acc = (1 - np.cos(np.pi * t/period_acc))/2
y_dec = (1 + np.cos(np.pi * t/period_acc))/2

fig = plt.figure(figsize=(5,3))
plt.plot(t,y_acc, label=r"$\alpha_{acc}$")
plt.plot(t,y_dec, label=r"$\alpha_{dec}$")
plt.grid(True)
# plt.title("Acceleration factors")
plt.xlabel("Time [s]")
plt.legend(loc="center right")
# plt.show()

#%% F_robot during movement
# --------------------------------------------------------
# -----         F_robot during movement              -----
# --------------------------------------------------------

folder_path = "/home/alexandergerard/Masterarbeit/Cmake_franka/build/data_thesis/"
# folder_path = "/home/alexandergerard/Masterarbeit/Cmake_franka/build/data_ball_joint_manual/"

O_T_EE = readFile(folder_path + 'O_T_EE*')
F_T_EE = readFile(folder_path + 'F_T_EE*')
F_robot = readFile(folder_path + 'F_robot_*')

# Reshape the nx16 array into an array of n 4x4 matrices
size_mat = len(O_T_EE)
O_T_EE_mat = np.transpose(O_T_EE.reshape(size_mat,4,4), axes=(0,2,1))
F_T_EE_mat = np.transpose(F_T_EE.reshape(size_mat,4,4), axes=(0,2,1))

# Calculate the transformation matrix from base to flange
O_T_F_mat = O_T_EE_mat@np.linalg.inv(F_T_EE_mat)
O_T_F_flat = O_T_F_mat.reshape(size_mat,16)

# Only plot every hundreth entry
pos_matrix = O_T_F_flat[:-1:100,:] # position of the flange
x = pos_matrix[:,3]
y = pos_matrix[:,7]
z = pos_matrix[:,11]
orient_matrix = O_T_EE[:-1:100,:] # orientation of the EE
color = np.arange(len(x))/len(x)*len(O_T_EE)/1000
# Get ranges for the plot
x_max = np.max((x.max(), O_T_EE[:,12].max()))
x_min = np.min((x.min(), O_T_EE[:,12].min()))
y_max = np.max((y.max(), O_T_EE[:,13].max()))
y_min = np.min((y.min(), O_T_EE[:,13].min()))
z_max = np.max((z.max(), O_T_EE[:,14].max()))
z_min = np.min((z.min(), O_T_EE[:,14].min()))
max_range = np.array([x_max-x_min, y_max-y_min, z_max-z_min]).max() / 2.0 * 1.05
mid_x = (x_max+x_min) * 0.5
mid_y = (y_max+y_min) * 0.5
mid_z = (z_max+z_min) * 0.5

    # Plot
fig = plt.figure(figsize=(8,4))
ax_3d = fig.add_subplot(121, projection="3d")

    # Plot trajectory
p = ax_3d.scatter(x,y,z, c = color, cmap=plt.cm.viridis)
cbar = fig.colorbar(p, ax=ax_3d, orientation="horizontal", pad=0.2, label="time [s]")
ax_3d.plot(O_T_EE[0,12], O_T_EE[0,13], O_T_EE[0,14], "r*") #, O_T_EE[:,13]
# ax_3d.text(O_T_EE[0,12]+0.005, O_T_EE[0,14]+0.005, "EE")
ax_3d.set_xlim(mid_x - max_range, mid_x + max_range)
ax_3d.set_ylim(mid_y - max_range, mid_y + max_range)
ax_3d.set_zlim(mid_z - max_range, mid_z + max_range)
# ax_3d.set_aspect("equal")
# ax_3d.grid(True)
ax_3d.set_xlabel('X [mm]')
ax_3d.set_ylabel('Y [mm]')
ax_3d.set_title("Position of the Robot")
ax_3d.set_zlabel('Z [mm]')

    # Plot Forces
x = np.arange(len(F_robot))/1000
ax_force = fig.add_subplot(222)
ax_force.plot(x, F_robot[:,0], 'r', label="$F_x$")
ax_force.plot(x, F_robot[:,1], 'g', label="$F_y$")
ax_force.plot(x, F_robot[:,2], 'b', label="$F_z$")
ax_force.get_xaxis().set_ticklabels([])
ax_force.grid(True)
ax_force.legend(loc = "upper right")
ax_force.set_ylabel("Forces [N]")
    # Plot torque
ax_torque = fig.add_subplot(224)
ax_torque.plot(x, F_robot[:,3], 'r', label=r"$\tau _x$")
ax_torque.plot(x, F_robot[:,4], 'g', label=r"$\tau _y$")
ax_torque.plot(x, F_robot[:,5], 'b', label=r"$\tau _z$")
ax_torque.grid(True)
ax_torque.legend(loc = "upper right")
ax_torque.set_xlabel("Time [s]")
ax_torque.set_ylabel("Torques [Nm]")

# fig.suptitle("Forces during motion")
# plt.show()

#%% Rotation um EE
# --------------------------------------------------------
# -----              Bsp Rotation um EE              -----
# --------------------------------------------------------

def plot_quiver(X,Y,axs):
    quiver_angle = np.arctan([Y+1]/X)
    axs.quiver(X,Y,np.cos(quiver_angle), np.sin(quiver_angle))
    axs.quiver(X,Y,np.cos(quiver_angle+np.pi/2), np.sin(quiver_angle+np.pi/2))
def plot_text(X,Y,axs):
    for i in range(len(X)):
        axs.text(X[i]+0.02, Y[i]-0.05, rf"$x_{i}$") # :[{np.round(X[i],2)}, {np.round(Y[i],2)}]

angle = np.pi/4
t = np.linspace(angle*2,angle,100)
r = 1
x = r*np.cos(t)
y = r*np.sin(t)-r

fig, axs = plt.subplots(figsize=(4,4))
axs.plot(x,y)
plot_quiver(x[[0,-1]],y[[0,-1]],axs)
# plot_text(x[[0,-1]],y[[0,-1]],axs)
axs.text(0.04,-r-0.01, rf"$x_0 = x_1$")
axs.plot(0,-r, 'b+')
axs.plot([x[-1],0], [y[-1],-r], 'b--')
axs.text(0.4, -1+0.4-0.05, "r")
y_lim = [-1.1, 0.1]
x_lim = [-0.1, 1.1]
#axs.set_xlim(x_lim)
#axs.set_ylim(y_lim)
axs.grid(True)
axs.set_axis_off()
axs.axis("equal")
# plt.show()
