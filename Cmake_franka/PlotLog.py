# %%
import glob
import os

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Function to create 7 Subplots from a dataframe
def plot7(df, columns = None):
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

    # Plot
    fig = plt.figure(figsize=(8,12))
        # Plot original position and position during movement
    ax1 = fig.add_subplot(311, projection='3d')
    ax1.plot(df_pos.iloc[:,0].to_numpy(), df_pos.iloc[:,1].to_numpy(), df_pos.iloc[:,2].to_numpy())
    ax1.plot(pos_origin[0], pos_origin[1], pos_origin[2], 'o')
    # Set y-lim
    ax1.set_xlim([df_pos.iloc[:,0].min()-.1, df_pos.iloc[:,0].min()+.1])
    # Set Label of axis
    ax1.set_xlabel("x")
    ax1.set_ylabel("y")
    ax1.set_zlabel("z")
    ax1.set_title("Absolute position of the end effector [m]")
        # Plot absolute distance from starting position 
    ax2 = fig.add_subplot(312)
    ax2.plot(distance_abs*1000) # convert to mm
    ax2.grid(True)
    ax2.set_title("Position error [mm]")
        # Plot Error of rotation
    ax3 = fig.add_subplot(313)
    x = df_rot.index.values
    ax3.plot(x, df_rot.iloc[:,0].to_numpy(), label = "x-rotation")
    ax3.plot(x, df_rot.iloc[:,1].to_numpy(), label = "y-rotation")
    ax3.plot(x, df_rot.iloc[:,2].to_numpy(), label = "z-rotation")
    ax3.grid(True)
    ax3.legend()
    ax3.set_title("Angle error [°]")

    plt.show()

# %%
# Plot torques
list_of_files_tau = glob.glob('/home/alexandergerard/Masterarbeit/Cmake_franka/build/tau_da*')
filePath_tau = max(list_of_files_tau, key=os.path.getctime)
df_orig_tau = pd.read_csv(filePath_tau, header=None)
df_tau = df_orig_tau.copy()

plot7(df_tau)

# %%
# Get position
list_of_files_pos = glob.glob('/home/alexandergerard/Masterarbeit/Cmake_franka/build/position_*')
filePath_pos = max(list_of_files_pos, key=os.path.getctime)
df_orig_pos = pd.read_csv(filePath_pos, header=None)
df_pos = df_orig_pos.copy()
# Get rotation
list_of_files_rot = glob.glob('/home/alexandergerard/Masterarbeit/Cmake_franka/build/rotati*')
filePath_rot = max(list_of_files_rot, key=os.path.getctime)
df_orig_rot = pd.read_csv(filePath_rot, header=None)
df_rot = df_orig_rot.copy()

# Plot error
plot_error(df_pos, df_rot)
# %%
