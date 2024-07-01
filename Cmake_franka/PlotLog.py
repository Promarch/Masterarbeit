# %%
import glob
import os

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Function to create 7 Subplots from a dataframe
def plot_torque(df, columns = None):
    if "time" in df.columns:
        x = df["time"]
    else:
        x = df.index.values

    if columns == None:
        columns = np.arange(7)
    
    fig, axs = plt.subplots(7, 1, figsize=(10, 15), sharex=True)
    
    for i, col in enumerate(columns):
        axs[i].plot(x, df[col].to_numpy())
        axs[i].set_ylabel(col)
        axs[i].grid(True)
        #axs[i].set_ylim([df[useCols[1:]].min().min(), df[useCols[1:]].max().max()])


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

def plot_error(df_pos, df_rot):
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
    # Set y-lim
    ax1.set_xlim([df_pos.iloc[:,0].min()-.1, df_pos.iloc[:,0].min()+.1])
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

def plot_force(df_force):
    fig, ax = plt.subplots(2, 1, figsize=(8,8), sharex=True)
        # Plot Forces
    ax[0].plot(df_force.iloc[:,0].to_numpy(), 'r', label="X")
    ax[0].plot(df_force.iloc[:,1].to_numpy(), 'g', label="Y")
    ax[0].plot(df_force.iloc[:,2].to_numpy(), 'b', label="Z")
    ax[0].legend(loc = "upper right")
    ax[0].set_title("Forces [N]")
        # Plot torque
    ax[1].plot(df_force.iloc[:,3].to_numpy(), 'r', label="X")
    ax[1].plot(df_force.iloc[:,4].to_numpy(), 'g', label="Y")
    ax[1].plot(df_force.iloc[:,5].to_numpy(), 'b', label="Z")
    ax[1].legend(loc = "upper right")
    ax[1].set_title("Torques [Nm]")

    # Add grid to all plots
    for a in ax.flat:
        a.grid(True)
    
    plt.show()
# %%
# Plot torques
list_of_files_tau = glob.glob('/home/alexandergerard/Masterarbeit/Cmake_franka/build/data_output/tau_da*')
filePath_tau = max(list_of_files_tau, key=os.path.getctime)
df_orig_tau = pd.read_csv(filePath_tau, header=None)
df_tau = df_orig_tau.copy()

plot_torque(df_tau)

# %%
# Plot orientation error
    # Get position
list_of_files_pos = glob.glob('/home/alexandergerard/Masterarbeit/Cmake_franka/build/data_output/position_*')
filePath_pos = max(list_of_files_pos, key=os.path.getctime)
df_orig_pos = pd.read_csv(filePath_pos, header=None)
df_pos = df_orig_pos.copy()
    # Get rotation
list_of_files_rot = glob.glob('/home/alexandergerard/Masterarbeit/Cmake_franka/build/data_output/rotati*')
filePath_rot = max(list_of_files_rot, key=os.path.getctime)
df_orig_rot = pd.read_csv(filePath_rot, header=None)
df_rot = df_orig_rot.copy()

# Plot error
plot_error(df_pos, df_rot)

# %%
    # Get force data
list_of_files_force = glob.glob('/home/alexandergerard/Masterarbeit/Cmake_franka/build/data_output/force*')
filePath_force = max(list_of_files_force, key=os.path.getctime)
df_orig_force = pd.read_csv(filePath_force, header=None)
df_force = df_orig_force.copy()
    # Plot Force
plot_force(df_force)

