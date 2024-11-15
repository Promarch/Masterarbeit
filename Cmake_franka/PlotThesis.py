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

np.set_printoptions(precision=3, suppress=True)


#%% Plot trajectory of robot in polar plot

def SphereCartesian(r, theta, phi, center):
    if (np.size(theta)>1):
        u, v = np.meshgrid(theta, phi)
    else:
        u = theta
        v = phi
    x = r * np.sin(u)*np.cos(v)+ center[0]
    y = r * np.sin(u)*np.sin(v)+ center[1]
    z = r * np.cos(u)+ center[2]
    return x,y,z

def CartSphere(x, y, z, center):
    x = x-center[0]
    y = y-center[1]
    z = z-center[2]
    r = np.sqrt(x**2 + y**2 + z**2)
    theta = np.arccos(z / r)
    phi = np.arctan2(y, x)
    return theta, phi

def set_axes_equal(ax):
    """
    Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc.

    Input
      ax: a matplotlib axis, e.g., as output from plt.gca().
    """

    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    # The plot bounding box is a sphere in the sense of the infinity
    # norm, hence I call half the max range the plot radius.
    plot_radius = 0.5*max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])

folder_path = "build/data_thesis/FullRun/"
# folder_path = "/home/alexandergerard/Masterarbeit/Cmake_franka/build/data_output_knee/"
O_T_EE = readFile(folder_path + "O_T_EE_dat*")
F_T_EE = readFile(folder_path + "F_T_EE_dat*")
quat_stop_time = readFile(folder_path + "quat_stop_time*")
for i in range(len(quat_stop_time)-1):
    if quat_stop_time[i+1,-1]<quat_stop_time[i,-1]:
        quat_stop_time[i+1:,-1] += 30
stop_time = np.round(quat_stop_time[:,-1]*1000).astype(int)

# Get Radius of the sphere and set center of rotation to [0,0,0]
r = F_T_EE[0,14]
pos_EE = O_T_EE[0,12:15]
O_T_EE[:,12:15] = O_T_EE[:,12:15] - pos_EE
# Reshape so that I can do matrix multiplication
size_mat = len(O_T_EE)
O_T_EE_mat = np.transpose(O_T_EE.reshape(size_mat,4,4), axes=(0,2,1))
F_T_EE_mat = np.transpose(F_T_EE.reshape(size_mat,4,4), axes=(0,2,1))
# Calculate the transformation matrix from base to flange
O_T_F_mat = O_T_EE_mat@np.linalg.inv(F_T_EE_mat)
O_T_F_flat = O_T_F_mat.reshape(size_mat,16)
# Get position
sampleSize = 1 # Not sure if needed, is there so that it doesnt lag to much
pos_matrix = O_T_F_flat[:-1:sampleSize,:] # position of the flange
x = pos_matrix[:,3]
y = pos_matrix[:,7]
z = pos_matrix[:,11]
theta, phi = CartSphere(x,y,z,[0,0,0])
theta_stop, phi_stop = CartSphere(O_T_F_flat[stop_time,3], O_T_F_flat[stop_time,7], O_T_F_flat[stop_time,11],[0,0,0])

    # Plot polar
nSwitch = 30000
fig_polar, ax_polar = plt.subplots(subplot_kw={'projection': 'polar'})
ax_trajectory_random = ax_polar.plot(phi[:nSwitch]+np.pi, theta[:nSwitch]*180/np.pi, label="Random")
ax_trajectory_guided = ax_polar.plot(phi[nSwitch:]+np.pi, theta[nSwitch:]*180/np.pi, label="Guided")
ax_stop = ax_polar.plot(phi_stop+np.pi, theta_stop*180/np.pi, 'r*', markersize=6, label=r"$F>F_{max}$")
ax_polar.set_xticks(np.pi/180 * np.linspace(180, -180, 12, endpoint=False)) #, endpoint=False
xTicks = np.append("",np.linspace(180, -180, 12, endpoint=False).astype(int)[1:])
ax_polar.set_xticklabels(xTicks)

ax_polar.set_thetalim(-np.pi, np.pi)
ax_polar.set_rticks([5,10,15,20,25,30,35])
ax_polar.set_rmax(35)
ax_polar.set_rlabel_position(180)
ax_polar.yaxis.set_label_position("left")
ax_polar.set_ylabel("Flexion [°]", fontsize = 11, labelpad=10, rotation=90)
ax_polar.set_xlabel("External [°]", fontsize = 11)

fig_polar.text(0.47, 0.95, 'Internal [°]', va='center', fontsize = 11)
ax_polar.legend(bbox_to_anchor=(1.15,1.10))
fig_polar.show()


    # Create data to plot a hemisphere
# Generate angle values for polar coordinates
n_full = 40
theta_full = np.linspace(0,np.pi/2, n_full)
phi_full = np.linspace(-np.pi,np.pi, n_full)
x_full, y_full, z_full = SphereCartesian(r, theta_full, phi_full, pos_EE)

# fig = plt.figure(figsize=(6,6))
# ax = fig.add_subplot(projection="3d")
# ax.view_init(azim=165, elev=40)
# ax_surface = ax.plot_surface(x_full, y_full, z_full, color=[0.8, 0.8, 0.8], linewidth=0, alpha = 0.4, antialiased=True, zorder=0) # 
# ax_trajectory_guided = ax.plot(x[nSwitch:],y[nSwitch:],z[nSwitch:], 'r-', zorder=0.5)
# ax_trajectory_random = ax.plot(x[:nSwitch],y[:nSwitch],z[:nSwitch], 'b-', zorder=0.5)
# ax.set_xlim(-r, r)
# ax.set_ylim(-r, r)
# ax.set_zlim(r/2-r, r/2+r)
# ax.set_xlabel('X [mm]')
# ax.set_ylabel('Y [mm]')
# ax.set_zlabel('Z [mm]', rotation=90)
# plt.show()





#%% Difference Nullspace
# --------------------------------------------------------
# -----                Nullspace Plot                -----
# --------------------------------------------------------
fontSize=14
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
plt.plot(x, distance_norm[:lim5], label = "default")
plt.plot(x, distance_null_100[:lim5], label = r"$K_6 = 100$")
plt.plot(x, distance_null_400[:lim5], label = r"$K_6 = 400$")
plt.plot(np.arange(0,len(distance_null_1000))/1000, distance_null_1000, label = r"$K_6 = 1000$")
# plt.plot(distance_null_300, label = r"Ausrichtung")
plt.grid(True)
plt.tick_params(axis='both', labelsize=fontSize)
plt.legend(loc = "upper right", fontsize=fontSize)
plt.title("Absolute positional error", fontsize=fontSize+2)
plt.xlabel("Time [s]", fontsize=fontSize+1)
plt.ylabel("Error [mm]", fontsize=fontSize+1)
# plt.show()




#%% Comparison impedance libfranka vs traditional

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
ax[0].plot(x[:nEntries], euler_angles_franka[:nEntries,0]+20, label="Libfranka")
ax[0].plot(x[:nEntries], euler_angles_normal[:nEntries,0]+20, label="Default")
ax[0].set_ylabel(r"Angle $\phi_x$ [°]")
ax[0].set_xlabel("Time [s]")
ax[0].set_ylim(ylim_bottom, ylim_top)
ax[0].grid(True)
ax[0].set_title(r"Rotation error $\Delta \phi_x$")
ax[1].plot(x[:nEntries], pos_abs_franka[:nEntries], label = "Libfranka")
ax[1].plot(x[:nEntries], pos_abs_normal[:nEntries], label = "Default")
ax[1].grid(True)
# ax[1].yaxis.tick_right()
# ax[1].yaxis.set_label_position("right")
# ax[1].yaxis.label.set_rotation(-90)
ax[1].set_xlabel("Time [s]")
ax[1].set_ylabel("Error [mm]", labelpad=0)
ax[1].set_title("Absolute positional error")
ax[1].legend()
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
# ax[1].yaxis.tick_right()
# ax[1].yaxis.set_label_position("right")
# ax[1].yaxis.label.set_rotation(-90)
ax[1].set_xlabel("Time [s]")
ax[1].set_ylabel("Error [mm]", labelpad = 0)
ax[1].set_title("Absolute positional error")
ax[1].legend()
# plt.show()



#%% Comparison F_ext vs F_sensor

folder_path = "/home/alexandergerard/Masterarbeit/Cmake_franka/build/data_thesis/"

F_robot = readFile(folder_path+"F_robot*")
F_sensor = readFile(folder_path+"F_sensor_total*")
x = np.arange(len(F_robot))/1000
labels_axis = [r"$F_x$", r"$F_y$", r"$F_z$", r"$\tau_x$", r"$\tau_y$", r"$\tau_z$"]
fontSize = 12

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
    # ax[i%3, round(np.floor(i/3))].set_ylabel(labels_axis[i], fontsize=fontSize+1)
    ax[i%3, round(np.floor(i/3))].grid(True)
for i in range(3):
    ax[i,0].set_ylim(-max_F_abs, max_F_abs)
    ax[i,1].set_ylim(-max_tau_abs, max_tau_abs)
    ax[i,0].yaxis.label.set_verticalalignment("center")
    ax[i,0].set_ylabel(labels_axis[i], labelpad=2, fontsize=fontSize)
    ax[i,1].set_ylabel(labels_axis[i+3], labelpad=-8, fontsize=fontSize)
    # ax[i,0].set_ylim(min_F,max_F)
    # ax[i,1].set_ylim(min_tau,max_tau)
    # ax[i,1].yaxis.tick_right()
    # ax[i,1].yaxis.set_label_position("right")
    # ax[i,1].yaxis.label.set_rotation(-90)

ax[0,0].set_title("Force [N]", fontsize=fontSize+2)
ax[0,1].set_title("Torque [Nm]", fontsize=fontSize+2)
ax[2, 0].set_xlabel("Time [s]", fontsize=fontSize)
ax[2, 1].set_xlabel("Time [s]", fontsize=fontSize)
ax[0,1].legend(loc = "upper right", fontsize=fontSize)

for a in ax.flat:
    a.tick_params(axis='both', labelsize=fontSize)

# fig.text(0.02, 0.5, "Forces [N]", va='center', ha='center', rotation='vertical', fontsize=fontSize+1)
# fig.text(0.5, 0.5, "Torque [Nm]", va='center', ha='center', rotation=90, fontsize=fontSize+1)
fig.suptitle("Comparison between external and internal sensor", fontsize=fontSize+3)
plt.tight_layout() #rect=[0.03, 0, 0.97, 1]
# plt.show()




#%% Comparison ext torque vs commanded torque
# --------------------------------------------------------
# -----              tau_ext vs tau_c                -----
# --------------------------------------------------------
folder_path = "/home/alexandergerard/Masterarbeit/Cmake_franka/build/data_thesis/"
tau_c = readFile(folder_path+"tau_data*")
tau_ext = readFile(folder_path+"tau_filter_data*")
tau_array = [tau_c, tau_ext]
fontSize = 12

x = np.arange(len(tau_array[0]))/1000
# Get min and max of torque
min_tau = np.min(tau_array)
max_tau = np.max(tau_array)
max_tau_abs = np.max([np.abs(min_tau), max_tau])

fig, axs = plt.subplots(4, 2, figsize=(8, 10), sharex="col")
for i in range(4):
    axs[i,0].plot(x, tau_c[:,i], label = r"$\tau_{c}$")
    axs[i,0].plot(x, tau_ext[:,i], label = r"$\tau_{sensor}$")
    axs[i,0].set_ylabel(f"Joint {i+1}", labelpad=-2, fontsize=fontSize+1)
    axs[i,0].set_ylim(min_tau-.2, max_tau+.2)
    axs[i,0].set_ylim(-max_tau_abs-.2, max_tau_abs+.2)
    axs[i,0].grid(True)
for i in range(3):
    axs[i,1].plot(x, tau_c[:,4+i], label = r"$\tau_{c}$")
    axs[i,1].plot(x, tau_ext[:,4+i], label = r"$\tau_{sensor}$")
    axs[i,1].set_ylabel(f"Joint {5+i}", labelpad=8, fontsize=fontSize+1)
    axs[i,1].set_ylim(min_tau-.2, max_tau+.2)
    axs[i,1].set_ylim(-max_tau_abs-.2, max_tau_abs+.2)
    axs[i,1].grid(True)
    axs[i,1].yaxis.tick_right()
    axs[i,1].yaxis.set_label_position("right")
    axs[i,1].yaxis.label.set_rotation(-90)
axs[0,1].legend(loc = "upper right", fontsize=fontSize)
axs[3, 0].set_xlabel("Time [s]", fontsize=fontSize+1)
axs[2, 1].tick_params(labelbottom=True)
axs[2, 1].set_xlabel("Time [s]", fontsize=fontSize+1)
axs[3, 1].axis('off')

for a in axs.flat:
    a.tick_params(axis='both', labelsize=fontSize)

# Set the y-axis label for the whole figure
fig.text(0.03, 0.5, 'Torque [Nm]', va='center', rotation='vertical', fontsize=fontSize+1)
plt.tight_layout(rect=[0.05,0,1,0.97])
fig.suptitle("Torque of each joint", fontsize=fontSize+3)
# plt.show()



#%% Sensor during movement
from matplotlib import rc
rc('font',**{'family':'afm','sans-serif':['Helvetica']})
fontSize = 12
    # Plot Forces
folder_path = "/home/alexandergerard/Masterarbeit/Cmake_franka/build/data_thesis/"
F_robot = readFile(folder_path + 'F_robot_*')
x = np.arange(len(F_robot))/1000
fig = plt.figure()
ax_force = fig.add_subplot(211)
ax_force.plot(x, F_robot[:,0], 'r', label="$F_x$")
ax_force.plot(x, F_robot[:,1], 'g', label="$F_y$")
ax_force.plot(x, F_robot[:,2], 'b', label="$F_z$")
ax_force.get_xaxis().set_ticklabels([])
ax_force.legend(loc = "upper right", fontsize = fontSize)
ax_force.set_ylabel("Forces [N]", fontsize = fontSize+1)
ax_force.tick_params(axis='both', labelsize=fontSize)
ax_force.grid(True)
    # Plot torque
ax_torque = fig.add_subplot(212)
ax_torque.plot(x, F_robot[:,3], 'r', label=r"$\tau _x$")
ax_torque.plot(x, F_robot[:,4], 'g', label=r"$\tau _y$")
ax_torque.plot(x, F_robot[:,5], 'b', label=r"$\tau _z$")
ax_torque.set_xlabel("Time [s]", fontsize = fontSize+1)
ax_torque.set_ylabel("Torques [Nm]", fontsize = fontSize+1)
ax_torque.legend(loc = "upper right", fontsize = fontSize)
ax_torque.tick_params(axis='both', labelsize=fontSize)
ax_torque.grid(True)

fig.suptitle("Sensor measurements", fontsize = fontSize+3)
# plt.show()

#%% Plot forces during the entire trajectory
from matplotlib import rc
rc('font',**{'family':'afm','sans-serif':['Helvetica']})
# plt.rcParams["font.family"] = "Helvetica"
# hfont = {'fontname':'fonts/afm/pcrbo8a.afm'}
def lowpassFilterDf(df, F_cutoff=10):
    sample_rate = 1000
    df_filter = df.copy()
    for i in range(np.size(df, 1)):
        data = np.array(df.iloc[:,i])
        fft_sig = np.fft.fft(data)
        cutoff_filter = 1.0 * np.abs(np.fft.fftfreq(fft_sig.size, 1.0/sample_rate)) <= F_cutoff
        data_filter = np.real(np.fft.ifft(fft_sig * cutoff_filter))
        df_filter.iloc[:,i] = data_filter
    return df_filter

def lowpassFilter(array, F_cutoff=10):
    sample_rate = 1000
    data_filtered = array
    for i in range(np.size(array, 1)):
        data = np.array(array[:,i])
        fft_sig = np.fft.fft(data)
        cutoff_filter = 1.0 * np.abs(np.fft.fftfreq(fft_sig.size, 1.0/sample_rate)) <= F_cutoff
        data_filter = np.real(np.fft.ifft(fft_sig * cutoff_filter))
        data_filtered[:,i] = data_filter
    return data_filtered

fontSize=14
folder_path = "build/data_thesis/FullRun/"
F_array = readFile(folder_path + "F_sensor_to*")
F_array = lowpassFilter(F_array, 3)
x = np.arange(len(F_array))/1000
fig, ax = plt.subplots(2, 1, figsize=(8,8), sharex=True)
    # Plot Forces
ax[0].plot(x, F_array[:,0], 'r', label="$F_x$")
ax[0].plot(x, F_array[:,1], 'g', label="$F_y$")
ax[0].plot(x, F_array[:,2], 'b', label="$F_z$")
ax[0].set_ylabel("Forces [N]", fontsize=fontSize+1)
ax[0].legend(loc = "upper right", fontsize=fontSize)
ax[0].grid(True)
    # Plot torque
ax[1].plot(x, F_array[:,3], 'r', label=r"$\tau_x$")
ax[1].plot(x, F_array[:,4], 'g', label=r"$\tau_y$")
ax[1].plot(x, F_array[:,5], 'b', label=r"$\tau_z$")
ax[1].legend(loc = "upper right", fontsize=fontSize)
ax[1].set_xticks([0,15,30,45,60,75,90])
ax[1].set_xlabel("Time [s]", fontsize=fontSize+1)
ax[1].set_ylabel("Torques [Nm]", fontsize=fontSize+1)
ax[1].grid(True)

# # Add grid to all plots
for a in ax.flat:
    a.grid(True)
    a.tick_params(axis='both', labelsize=fontSize)
plt.tight_layout()
plt.suptitle("Sensor measurements during the trajectory", fontsize=fontSize+3)
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

#%% F_robot during movement with trajectory
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
cbar = fig.colorbar(p, ax=ax_3d, orientation="horizontal", pad=0.2, label="Time [s]")
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

#%% Acceleration factor
# --------------------------------------------------------
# -----            Beschleunigungsfaktor             -----
# --------------------------------------------------------

period_acc = 1
t = np.linspace(0,period_acc,100)
y_acc = (1 - np.cos(np.pi * t/period_acc))/2
y_dec = (1 + np.cos(np.pi * t/period_acc))/2

fig, ax = plt.subplots(1, 1, figsize=(5,3))
ax.plot(t,y_acc, label=r"$\alpha_{acc}$")
ax.plot(t,y_dec, label=r"$\alpha_{dec}$")
ax.grid(True)
# plt.title("Acceleration factors")
ax.set_xlabel("Time [s]", fontsize = 12) #, fontsize = 11
ax.tick_params(axis='both', labelsize=12)
plt.legend(loc="center right", fontsize = 12)
# plt.show()

#%%
