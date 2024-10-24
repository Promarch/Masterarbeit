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

#%% Comparison F_ext vs F_sensor

folder_path = "/home/alexandergerard/Masterarbeit/Cmake_franka/build/data_thesis/"

F_robot = readFile(folder_path+"F_robot*")
F_sensor = readFile(folder_path+"F_sensor_total*")
x = np.arange(len(F_robot))/1000
labels_axis = [r"$F_x$", r"$F_y$", r"$F_z$", r"$\tau_x$", r"$\tau_y$", r"$\tau_z$"]

min_F = np.min([F_robot[:,:3], F_sensor[:,:3]])
max_F = np.max([F_robot[:,:3], F_sensor[:,:3]])

min_tau = np.min([F_robot[:,3:], F_sensor[:,3:]])
max_tau = np.max([F_robot[:,3:], F_sensor[:,3:]])

fig,ax = plt.subplots(3,2, sharex=True)
for i in range(6):
    ax[i%3, round(np.floor(i/3))].plot(x, F_robot[:,i], label=r"$F_{robot}$")
    ax[i%3, round(np.floor(i/3))].plot(x, F_sensor[:,i], label=r"$F_{sensor}$")
    ax[i%3, round(np.floor(i/3))].set_ylabel(labels_axis[i])
    ax[i%3, round(np.floor(i/3))].grid(True)
    ax[i%3, round(np.floor(i/3))].legend(loc = "upper right")
for i in range(3):
    ax[i,0].set_ylim(min_F,max_F)
    ax[i,1].set_ylim(min_tau,max_tau)
    # ax[i,1].yaxis.tick_right()
    # ax[i,1].yaxis.set_label_position("right")


ax[2, 0].set_xlabel("Time [s]")
ax[2, 1].set_xlabel("Time [s]")
fig.suptitle("Comparison between external and internal Sensor")
plt.tight_layout()
plt.show()

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
plt.show()

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

fig, axs = plt.subplots(7, 1, figsize=(8, 12), sharex=True)
for i in range(7):
    for j, df in enumerate(tau_array):
        axs[i].plot(x, df[:,i], label = labels[j])
    axs[i].set_ylabel(f"Joint {i+1}")
    axs[i].set_ylim(min_tau-.2, max_tau+.2)
    axs[i].grid(True)
    axs[i].legend(loc = "lower right")
axs[i].set_xlabel("time [s]")
# Set the y-axis label for the whole figure
fig.text(0.03, 0.5, 'Torque [Nm]', va='center', rotation='vertical')
fig.suptitle("Torque of each joint")
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
distance_norm = np.linalg.norm(pos_norm-pos_norm[0,:], axis=1)
distance_null_100 = np.linalg.norm(pos_null_100-pos_null_100[0,:], axis=1)
distance_null_400 = np.linalg.norm(pos_null_400-pos_null_400[0,:], axis=1)
distance_null_1000 = np.linalg.norm(pos_null_1000-pos_null_1000[0,:], axis=1)
distance_null_300 = np.linalg.norm(pos_null_300-pos_null_300[0,:], axis=1)
x = np.arange(0,len(distance_norm))/1000

fig = plt.figure(figsize=(8,6))
factor_mm = 1000
plt.plot(x, distance_norm*factor_mm, label = "normal")
plt.plot(x, distance_null_100*factor_mm, label = r"$K_n = 100$")
plt.plot(x, distance_null_400*factor_mm, label = r"$K_n = 400$")
plt.plot(np.arange(0,len(distance_null_1000))/1000, distance_null_1000*factor_mm, label = r"$K_n = 1000$")
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

period_acc = 4
t = np.linspace(0,period_acc,100)
y_acc = (1 - np.cos(np.pi * t/period_acc))/2
y_dec = (1 + np.cos(np.pi * t/period_acc))/2

plt.plot(t,y_acc, label=r"$\alpha_{acc}$")
plt.plot(t,y_dec, label=r"$\alpha_{dec}$")
plt.grid(False)
plt.title("Acceleration factors")
plt.xlabel("Time [s]")
plt.legend(loc="center right")
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
