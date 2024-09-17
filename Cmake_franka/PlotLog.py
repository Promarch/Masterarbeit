# %%
import glob
import os

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

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

def plot7(df_array):
    # If array is of size one make it iteratable
    num_df = len(df_array)
    if num_df > 10:
        df_array = [df_array]

    x = df_array[0].index.values
    # Get min and max of torque
    min_tau = np.min(df_array)
    max_tau = np.max(df_array)

    fig, axs = plt.subplots(7, 1, figsize=(10, 15), sharex=True)
    for i in range(7):
        for j, df in enumerate(df_array):
            axs[i].plot(x, df.iloc[:,i].to_numpy(), label = f"df{j}")
        axs[i].set_ylabel(i+1)
        axs[i].set_ylim(min_tau-.2, max_tau+.2)
        axs[i].grid(True)
        axs[i].legend(loc = "upper right")
    plt.show()

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

# folder path
folder_path = "/home/alexandergerard/Masterarbeit/Cmake_franka/build/data_output_knee/"
folder_path = "/home/alexandergerard/Masterarbeit/Cmake_franka/build/data_grav/"
folder_path = "/home/alexandergerard/Masterarbeit/Cmake_franka/build/data_ball_joint/"

# %%
# Plot torques
list_of_files_tau = glob.glob(folder_path + 'tau_da*')
filePath_tau = max(list_of_files_tau, key=os.path.getctime)
df_orig_tau = pd.read_csv(filePath_tau, header=None)
df_tau = df_orig_tau.copy()

list_of_files_tau_filter = glob.glob(folder_path + 'tau_filter*')
filePath_tau_filter = max(list_of_files_tau_filter, key=os.path.getctime)
df_orig_tau_filter = pd.read_csv(filePath_tau_filter, header=None)
df_tau_filter = df_orig_tau_filter.copy()

# list_of_files_tau_mass = glob.glob('/home/alexandergerard/Masterarbeit/Cmake_franka/build/data_output/tau_mass*')
# filePath_tau_mass = max(list_of_files_tau_mass, key=os.path.getctime)
# df_orig_tau_mass = pd.read_csv(filePath_tau_mass, header=None)
# df_tau_mass = df_orig_tau_mass.copy()

list_of_files_q = glob.glob(folder_path + 'joint_pos*')
filePath_q = max(list_of_files_q, key=os.path.getctime)
df_orig_q = pd.read_csv(filePath_q, header=None)
df_q = df_orig_q.copy()

# plot7([df_tau, df_q])
plot_torque(df_tau, df_tau_filter, df_q)
# %%
    # Get external force data
list_of_files_force_ext = glob.glob(folder_path + 'F_robot_*')
filePath_force_ext = max(list_of_files_force_ext, key=os.path.getctime)
df_orig_force_ext = pd.read_csv(filePath_force_ext, header=None)
df_force_ext = df_orig_force_ext.copy()

#     # Get force data from torque+jacobian
# list_of_files_force_tau = glob.glob(folder_path + 'tau_force*')
# filePath_force_tau = max(list_of_files_force_tau, key=os.path.getctime)
# df_orig_force_tau = pd.read_csv(filePath_force_tau, header=None)
# df_force_tau = df_orig_force_tau.copy()

    # Get force data from sensor
list_of_files_F_sensor = glob.glob(folder_path + 'F_sensor_to*')
filePath_F_sensor = max(list_of_files_F_sensor, key=os.path.getctime)
df_orig_F_sensor = pd.read_csv(filePath_F_sensor, header=None)
df_F_sensor = df_orig_F_sensor.copy()
df_F_lowpass = lowpassFilter(df_F_sensor)

    # Plot Force
plot_force_F_T(df_F_sensor)
plot_force_tau_F(df_F_sensor, df_F_lowpass, labels = ["F_robot", "F_sens"])
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
