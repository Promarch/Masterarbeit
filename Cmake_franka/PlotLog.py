# %%
import glob
import os

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl

def lowpassFilter(df, F_cutoff=10):
    sample_rate = 1000
    df_filter = df.copy()
    for i in range(np.size(df, 1)):
        data = np.array(df.iloc[:,i])
        fft_sig = np.fft.fft(data)
        cutoff_filter = 1.0 * np.abs(np.fft.fftfreq(fft_sig.size, 1.0/sample_rate)) <= F_cutoff
        data_filter = np.real(np.fft.ifft(fft_sig * cutoff_filter))
        df_filter.iloc[:,i] = data_filter
    return df_filter

# Function to create 7 Subplots from a dataframe
def plot_torque(df_tau, df_filter, df_q, columns = None):
    if "time" in df_tau.columns:
        x = df_tau["time"]
    else:
        x = df_tau.index.values

    if columns == None:
        columns = np.arange(7)
    
    # Get min and max of torque
    min_tau = np.min([df_tau, df_filter]) # , df_mass
    max_tau = np.max([df_tau, df_filter]) # , df_mass

    # Get max difference between min and max in q
    max_range = np.max(np.max(df_q,0)-np.min(df_q,0)) * 1.1
    

    fig, axs = plt.subplots(7, 1, figsize=(10, 15), sharex=True)
    
    for i, col in enumerate(columns):
        axs[i].plot(x, df_tau[col].to_numpy(), label = r"$\tau_d$")
        axs[i].plot(x, df_filter[col].to_numpy(), label = r"$\tau_{filter}$")
        # axs[i].plot(x, df_mass[col].to_numpy(), label = r"$\tau_{mass}$")
        axs[i].set_ylabel(col+1)
        axs[i].set_ylim(min_tau, max_tau)
        axs[i].grid(True)
        
        axs[i].legend(loc = "upper right")
        ax_twin = axs[i].twinx()
        ax_twin.plot(x, df_q[col].to_numpy(), "r--", label = r"$q_{pos}$")
        ax_twin.legend(loc = "lower right")
        # Set y_lim so that you always have the same tick range
        y_lim_min = (np.max(df_q[col])+np.min(df_q[col]))/2 - max_range/2
        y_lim_max = (np.max(df_q[col])+np.min(df_q[col]))/2 + max_range/2
        ax_twin.set_ylim(y_lim_min, y_lim_max)
    
    axs[-1].set_xlabel('Time')
    # Adjust layout
    plt.tight_layout()
    # Show the plot
    plt.show()
# Plot position history in 3D and absolute distance 
# Input is x,y,z as a dataframe
def plot_position(df):

    # Get Original position and calculate the absolute distance
    pos_origin = df.iloc[0,:].values
    distance_abs = np.linalg.norm(df-pos_origin, axis=1)

    # Plot
    fig = plt.figure(figsize=(8,8))
        # Plot original position and position during movement
    ax1 = fig.add_subplot(211, projection='3d')
    ax1.plot(df.iloc[:,0].to_numpy(), df.iloc[:,1].to_numpy(), df.iloc[:,2].to_numpy())
    ax1.plot(pos_origin[0], pos_origin[1], pos_origin[2], 'o')
    # Set y-lim
    ax1.set_xlim([df.iloc[:,0].min()-.1, df.iloc[:,0].min()+.1])
    # Set Label of axis
    ax1.set_xlabel("x")
    ax1.set_ylabel("y")
    ax1.set_zlabel("z")
        # Plot absolute distance from starting position 
    ax2 = fig.add_subplot(212)
    ax2.plot(distance_abs)

    plt.show()

def plot_rotation(df):

    x = df.index.values
    plt.plot(x, df.iloc[:,0].to_numpy(), label = "x-rotation")
    plt.plot(x, df.iloc[:,1].to_numpy(), label = "y-rotation")
    plt.plot(x, df.iloc[:,2].to_numpy(), label = "z-rotation")
    plt.legend()
    plt.show()

def plot_orientation_error(df_pos, df_rot):
    # Get Original position and calculate the absolute distance
    pos_origin = df_pos.iloc[0,:].values
    distance_abs = np.linalg.norm(df_pos-pos_origin, axis=1)

    # Correct the errors in the rotation that cause sudden jumps in the value
    for col in df_rot.columns:
        df_rot[col] = df_rot[col].apply(lambda x: x - 360 if x > 180 else x)
    # Plot
    fig = plt.figure(figsize=(12,8))
        # Plot original position and position during movement
    ax1 = fig.add_subplot(122, projection='3d')
    ax1.plot(df_pos.iloc[:,0].to_numpy(), df_pos.iloc[:,1].to_numpy(), df_pos.iloc[:,2].to_numpy(), label="trajectory")
    ax1.plot(pos_origin[0], pos_origin[1], pos_origin[2], 'o', label="start")
    # Set plot limits (https://stackoverflow.com/questions/13685386/how-to-set-the-equal-aspect-ratio-for-all-axes-x-y-z)
    x = df_pos.iloc[:, 0].to_numpy()
    y = df_pos.iloc[:, 1].to_numpy()
    z = df_pos.iloc[:, 2].to_numpy()
    max_range = np.array([x.max()-x.min(), y.max()-y.min(), z.max()-z.min()]).max() / 2.0
    mid_x = (x.max()+x.min()) * 0.5
    mid_y = (y.max()+y.min()) * 0.5
    mid_z = (z.max()+z.min()) * 0.5
    ax1.set_xlim(mid_x - max_range, mid_x + max_range)
    ax1.set_ylim(mid_y - max_range, mid_y + max_range)
    ax1.set_zlim(mid_z - max_range, mid_z + max_range)
    # Set View
    ax1.view_init(elev=19, azim=-154)
    # Set Label of axis
    ax1.set_xlabel("x")
    ax1.set_ylabel("y")
    ax1.set_zlabel("z")
    ax1.legend()
    ax1.set_title("Absolute position of the end effector [m]")
        # Plot absolute distance from starting position 
    ax2 = fig.add_subplot(221)
    ax2.plot(distance_abs*1000) # convert to mm
    ax2.grid(True)
    ax2.set_title("Position error [mm]")
        # Plot Error of rotation
    ax3 = fig.add_subplot(223)
    x = df_rot.index.values
    ax3.plot(x, df_rot.iloc[:,0].to_numpy(), label = "x-rotation")
    ax3.plot(x, df_rot.iloc[:,1].to_numpy(), label = "y-rotation")
    ax3.plot(x, df_rot.iloc[:,2].to_numpy(), label = "z-rotation")
    ax3.grid(True)
    ax3.legend()
    ax3.set_title("Angle error [°]")

    plt.show()

def plot_force_F_T(df_force):
    fig, ax = plt.subplots(2, 1, figsize=(8,8), sharex=True)
        # Plot Forces
    ax[0].plot(df_force.iloc[:,0].to_numpy(), 'r', label="$F_x$")
    ax[0].plot(df_force.iloc[:,1].to_numpy(), 'g', label="$F_y$")
    ax[0].plot(df_force.iloc[:,2].to_numpy(), 'b', label="$F_z$")
    ax[0].legend(loc = "upper right")
    ax[0].set_title("Forces [N]")
        # Plot torque
    ax[1].plot(df_force.iloc[:,3].to_numpy(), 'r', label=r"$\tau _x$")
    ax[1].plot(df_force.iloc[:,4].to_numpy(), 'g', label=r"$\tau _y$")
    ax[1].plot(df_force.iloc[:,5].to_numpy(), 'b', label=r"$\tau _z$")
    ax[1].legend(loc = "upper right")
    ax[1].set_title("Torques [Nm]")

    # Add grid to all plots
    for a in ax.flat:
        a.grid(True)
    plt.suptitle("Forces from sensor")
    plt.show()

def plot_force_tau_F(df_F_ext, df_force_tau, labels=["F Robot", "Tau+Jacobi"]):
    fig, axs = plt.subplots(6, 1, figsize=(10, 15), sharex=True)
    x = df_F_ext.index.values
    plot_name = ["$F_x$", "$F_y$", "$F_z$", r"$\tau _x$", r"$\tau _y$", r"$\tau _z$"]
    for i in range(6):
        axs[i].plot(x, df_F_ext.iloc[:,i].to_numpy(), label = labels[0])
        axs[i].plot(x, df_force_tau.iloc[:,i].to_numpy(), label = labels[1])
        axs[i].set_ylabel(plot_name[i])
        axs[i].grid(True)
        #axs[i].set_ylim([df[useCols[1:]].min().min(), df[useCols[1:]].max().max()])
        axs[i].legend(loc = "upper right")

    axs[-1].set_xlabel('Time')
    # Adjust layout
    plt.suptitle("Force/Torque in workspace")
    # Show the plot
    plt.show()

def plot_force_error(df_error):
    fig, ax = plt.subplots(2, 1, figsize=(8,8), sharex=True)
        # Plot Forces
    ax[0].plot(df_error.iloc[:,0].to_numpy(), 'r', label="X")
    ax[0].plot(df_error.iloc[:,1].to_numpy(), 'g', label="Y")
    ax[0].plot(df_error.iloc[:,2].to_numpy(), 'b', label="Z")
    ax[0].legend(loc = "upper right")
    ax[0].set_title("Forces [N]")
        # Plot torque
    ax[1].plot(df_error.iloc[:,3].to_numpy(), 'r', label="X")
    ax[1].plot(df_error.iloc[:,4].to_numpy(), 'g', label="Y")
    ax[1].plot(df_error.iloc[:,5].to_numpy(), 'b', label="Z")
    ax[1].legend(loc = "upper right")
    ax[1].set_title("Torques [Nm]")

    # Add grid to all plots
    for a in ax.flat:
        a.grid(True)
    plt.suptitle("Force Error")
    plt.show()

def plot7(df_array, labels=[r"$\tau_{robotsensor}$", r"$\tau_{command}$"]):
    # If array is of size one make it iteratable
    num_df = len(df_array)
    if num_df > 10:
        df_array = [df_array]

    x = np.arange(0,len(df_array[0]))
    # Get min and max of torque
    min_tau = np.min(df_array)
    max_tau = np.max(df_array)

    fig, axs = plt.subplots(7, 1, figsize=(10, 15), sharex=True)
    for i in range(7):
        for j, df in enumerate(df_array):
            axs[i].plot(x, df[:,i], label = labels[j])
        axs[i].set_ylabel(f"Joint {i+1}")
        axs[i].set_ylim(min_tau-.2, max_tau+.2)
        axs[i].grid(True)
        axs[i].legend(loc = "upper right")
    axs[i].set_xlabel("time [ms]")
    fig.suptitle("Torque of each joint")
    # plt.tight_layout()
    plt.show()

def plot_PosOri(O_T_EE, F_T_EE):
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

    # Get ranges for the plot
    x_max = np.max((x.max(), O_T_EE[:,12].max()))
    x_min = np.min((x.min(), O_T_EE[:,12].min()))
    y_max = np.max((y.max(), O_T_EE[:,13].max()))
    y_min = np.min((y.min(), O_T_EE[:,13].min()))
    z_max = np.max((z.max(), O_T_EE[:,14].max()))
    z_min = np.min((z.min(), O_T_EE[:,14].min()))

    
    max_range = np.array([x_max-x_min, y_max-y_min, z_max-z_min]).max() / 2.0
    mid_x = (x_max+x_min) * 0.5
    mid_y = (y_max+y_min) * 0.5
    mid_z = (z_max+z_min) * 0.5

    # Plot original position and position
    l_quiver = 0.01
    fig = plt.figure(figsize=(6,6))
    ax = fig.add_subplot(projection="3d")
    ax.quiver(x, y, z, orient_matrix[:,0], orient_matrix[:,1], orient_matrix[:,2], length=l_quiver)
    ax.quiver(x, y, z, orient_matrix[:,4], orient_matrix[:,5], orient_matrix[:,6], length=l_quiver)
    ax.quiver(x, y, z, orient_matrix[:,8], orient_matrix[:,9], orient_matrix[:,10], length=l_quiver)
    # ax.quiver(x[40], y[40], z[40], orient_matrix[40,8], orient_matrix[40,9], orient_matrix[40,10], length=l_quiver, color="r")
    ax.plot(x, y, z, "c-")
    ax.plot(O_T_EE[:,12], O_T_EE[:,13], O_T_EE[:,14], "r-")

    ax.view_init(elev=22, azim=22)
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    plt.show()

def plot_ForcePos(O_T_EE, F_T_EE, F_sensor, n_colors = 6):
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
    orient_matrix = O_T_EE_orig[:-1:100,:] # orientation of the EE

    # Get ranges for the plot
    x_max = np.max((x.max(), O_T_EE[:,12].max()))
    x_min = np.min((x.min(), O_T_EE[:,12].min()))
    y_max = np.max((y.max(), O_T_EE[:,13].max()))
    y_min = np.min((y.min(), O_T_EE[:,13].min()))
    z_max = np.max((z.max(), O_T_EE[:,14].max()))
    z_min = np.min((z.min(), O_T_EE[:,14].min()))

    
    max_range = np.array([x_max-x_min, y_max-y_min, z_max-z_min]).max() / 2.0
    mid_x = (x_max+x_min) * 0.5
    mid_y = (y_max+y_min) * 0.5
    mid_z = (z_max+z_min) * 0.5

        # Sort values by absolut torque into n different categories
    torque_abs = np.linalg.norm(F_sensor[:-1:100,3:], axis=1)
    # Sort the indices to then sort the values (and calculate percentiles from there)
    sorted_indices = np.argsort(torque_abs)
    sorted_data = torque_abs[sorted_indices]
    # Calculate the percentiles on the sorted values
    percentiles = np.percentile(sorted_data, np.round(np.linspace(0, 100, n_colors+1))[1:])
    # Use a loop to create subsets of indices
    percentile_index = []
    prev = -np.inf
    for p in percentiles:
        subset_mask = (sorted_data > prev) & (sorted_data <= p)
        percentile_index.append(sorted_indices[subset_mask])
        prev = p

    # Calculate the hex values for the points
    opacity = 0.8
    percentiles_red = -np.percentile(np.linspace(0,-2,100), np.round(np.linspace(0,100, n_colors+1))[:-1])
    percentiles_green = np.percentile(np.linspace(0,2,100), np.round(np.linspace(0,100, n_colors+1))[1:])
    percentiles_red[percentiles_red>1]=1
    percentiles_green[percentiles_green>1]=1

        # Plot 
    fig = plt.figure(figsize=(6,6))
    ax = fig.add_subplot(projection="3d")
    # Plot path of the robot
    ax.plot(x, y, z, "k-")
    # Plot points with colors according to the force
    for i, index in enumerate(percentile_index):
        ax.plot(x[index], y[index], z[index], "o", color=(percentiles_red[i], percentiles_green[i], 0, opacity))
    # Plot path of the virtuel center of rotation (configured EE)
    ax.plot(O_T_EE_orig[:,12], O_T_EE_orig[:,13], O_T_EE_orig[:,14], "r-")

    ax.view_init(elev=22, azim=22)
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    plt.show()

def plot_PosError(O_T_EE):
    pos = np.array([O_T_EE[:,12], O_T_EE[:,13], O_T_EE[:,14]])
    pos_abs = np.linalg.norm(pos.transpose()-pos[:,0], axis=1)*1000
    plt.plot(pos_abs)
    plt.xlabel("Time [ms]")
    plt.ylabel("Position error [mm]")
    plt.title("Absolute positional error during the trajectory")
    plt.show()

def CartSphere(x, y, z, center):
    x = x-center[0]
    y = y-center[1]
    z = z-center[2]
    r = np.sqrt(x**2 + y**2 + z**2)
    theta = np.arccos(z / r)
    phi = np.arctan2(y, x)
    return theta, phi

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

# folder path
folder_path = "/home/alexandergerard/Masterarbeit/Cmake_franka/build/data_grav/"
folder_path = "/home/alexandergerard/Masterarbeit/Cmake_franka/build/data_ball_joint/"
folder_path = "/home/alexandergerard/Masterarbeit/Cmake_franka/build/data_ball_joint_manual/"
# folder_path = "/home/alexandergerard/Masterarbeit/Cmake_franka/build/data_output_knee/"
# folder_path = "/home/alexandergerard/Masterarbeit/Cmake_franka/build/data_thesis/"

# %%
# Plot position and orientation
list_of_files_O_T_EE = glob.glob(folder_path + 'O_T_EE*')
filePath_O_T_EE = max(list_of_files_O_T_EE, key=os.path.getctime)
O_T_EE_orig = np.loadtxt(filePath_O_T_EE, delimiter=",")

list_of_files_F_T_EE = glob.glob(folder_path + 'F_T_EE*')
filePath_F_T_EE = max(list_of_files_F_T_EE, key=os.path.getctime)
F_T_EE_orig = np.loadtxt(filePath_F_T_EE, delimiter=",")

plot_PosOri(O_T_EE_orig, F_T_EE_orig)
# plot_PosError(O_T_EE_orig)

#%%
    # Import torque with numpy
list_of_files_tau = glob.glob(folder_path + 'tau_da*')
filePath_tau = max(list_of_files_tau, key=os.path.getctime)
np_tau = np.loadtxt(filePath_tau, delimiter=",")

list_of_files_tau_filter = glob.glob(folder_path + 'tau_filter*')
filePath_tau_filter = max(list_of_files_tau_filter, key=os.path.getctime)
np_tau_filter = np.loadtxt(filePath_tau_filter, delimiter=",")

list_of_files_q = glob.glob(folder_path + 'joint_pos*')
filePath_q = max(list_of_files_q, key=os.path.getctime)
np_q = np.loadtxt(filePath_q, delimiter=",")
dq = (np_q[:-1:100,:] - np_q[1::100,:])*1000

# plot7([np_tau, np_tau_filter], labels=[r"$\tau_c$", r"$\tau_{sensor}$"]) #, labels=[r"$\tau_c$", r"$\tau_r$"]

#%%
# Plot torques
list_of_files_tau = glob.glob(folder_path + 'tau_da*')
filePath_tau = max(list_of_files_tau, key=os.path.getctime)
df_orig_tau = pd.read_csv(filePath_tau, header=None)
df_tau = df_orig_tau.copy()

list_of_files_tau_filter = glob.glob(folder_path + 'tau_filter*')
filePath_tau_filter = max(list_of_files_tau_filter, key=os.path.getctime)
df_orig_tau_filter = pd.read_csv(filePath_tau_filter, header=None)
df_tau_filter = df_orig_tau_filter.copy()

list_of_files_q = glob.glob(folder_path + 'joint_pos*')
filePath_q = max(list_of_files_q, key=os.path.getctime)
df_orig_q = pd.read_csv(filePath_q, header=None)
df_q = df_orig_q.copy()

# plot7([df_tau, df_tau_filter], labels=[r"$\tau_{command}$", r"$\tau_{robotsensor}$"])
# plot_torque(df_tau, df_tau_filter, df_q)
# %%
    # Get external force data
list_of_files_F_ext = glob.glob(folder_path + 'F_robot_*')
filePath_F_ext = max(list_of_files_F_ext, key=os.path.getctime)
df_orig_F_ext = pd.read_csv(filePath_F_ext, header=None)
df_F_ext = df_orig_F_ext.copy()

    # Get force data from sensor
list_of_files_F_sensor = glob.glob(folder_path + 'F_sensor_to*')
filePath_F_sensor = max(list_of_files_F_sensor, key=os.path.getctime)
df_orig_F_sensor = pd.read_csv(filePath_F_sensor, header=None)
df_F_sensor = df_orig_F_sensor.copy()
df_F_lowpass = lowpassFilter(df_F_sensor)

    # Plot Force
plot_force_F_T(df_F_lowpass)
# plot_force_tau_F(df_F_ext, df_F_lowpass, labels = [r"$F_{robot}$", r"$F_{sensor}$"])

# %%
# # Plot orientation error
#     # Get position
# list_of_files_pos = glob.glob(folder_path + 'position_*')
# filePath_pos = max(list_of_files_pos, key=os.path.getctime)
# df_orig_pos = pd.read_csv(filePath_pos, header=None)
# df_pos = df_orig_pos.copy()
#     # Get rotation
# list_of_files_rot = glob.glob(folder_path + 'rotati*')
# filePath_rot = max(list_of_files_rot, key=os.path.getctime)
# df_orig_rot = pd.read_csv(filePath_rot, header=None)
# df_rot = df_orig_rot.copy()

# Plot error
# plot_orientation_error(df_pos, df_rot)

# %%

    # Get force error data
# list_of_files_error = glob.glob(folder_path + 'error*')
# filePath_error = max(list_of_files_error, key=os.path.getctime)
# df_orig_error = pd.read_csv(filePath_error, header=None)
# df_error = df_orig_error.copy()
    # Plot Force
# plot_force_error(df_error)
