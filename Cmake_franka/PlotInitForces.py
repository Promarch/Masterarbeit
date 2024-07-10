# %%
import glob
import os

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# %% 
    # Import all data
# Torque without gravity
list_of_files = glob.glob('/home/alexandergerard/Masterarbeit/Cmake_franka/build/data_grav/tau_grav*')
filePath = max(list_of_files, key=os.path.getctime)
df_tau_grav = pd.read_csv(filePath, header=None)

# Torque filtered
list_of_files = glob.glob('/home/alexandergerard/Masterarbeit/Cmake_franka/build/data_grav/tau_filter*')
filePath = max(list_of_files, key=os.path.getctime)
df_tau_filter = pd.read_csv(filePath, header=None)

# Forces from torque without gravity
list_of_files = glob.glob('/home/alexandergerard/Masterarbeit/Cmake_franka/build/data_grav/F_tau_grav*')
filePath = max(list_of_files, key=os.path.getctime)
df_F_tau_grav = pd.read_csv(filePath, header=None)

# Forces from filtered torque
list_of_files = glob.glob('/home/alexandergerard/Masterarbeit/Cmake_franka/build/data_grav/F_tau_filter*')
filePath = max(list_of_files, key=os.path.getctime)
df_F_tau_filter = pd.read_csv(filePath, header=None)

# Force from robot measurement
list_of_files = glob.glob('/home/alexandergerard/Masterarbeit/Cmake_franka/build/data_grav/F_ext*')
filePath = max(list_of_files, key=os.path.getctime)
df_F_ext = pd.read_csv(filePath, header=None)

# Force from gravity
list_of_files = glob.glob('/home/alexandergerard/Masterarbeit/Cmake_franka/build/data_grav/F_grav*')
filePath = max(list_of_files, key=os.path.getctime)
df_F_grav = pd.read_csv(filePath, header=None)

# %%
# Plot torque

fig, axs = plt.subplots(7, 1, figsize=(10, 15), sharex=True)
x = df_tau_filter.index.values
joint_num = np.arange(1,8)
for i in range(7):
    axs[i].plot(x, df_tau_grav.iloc[:,i].to_numpy(), label = "Tau minus grav")
    axs[i].plot(x, df_tau_filter.iloc[:,i].to_numpy(), label = "Tau filter")
    axs[i].set_ylabel(i+1)
    axs[i].grid(True)
    #axs[i].set_ylim([df[useCols[1:]].min().min(), df[useCols[1:]].max().max()])
    axs[i].legend()

axs[-1].set_xlabel('Time')
# Adjust layout
plt.suptitle("Torque of each joint")
# Show the plot
plt.show()
# %%
# Plot forces calculated with the jacobian
df_test = df_F_ext-df_F_ext.iloc[250:-1, :].mean()
fig, axs = plt.subplots(6, 1, figsize=(10, 15), sharex=True)
x = df_tau_filter.index.values
plot_name = ["X", "Y", "Z", "rot_x", "rot_y", "rot_z"]
for i in range(6):
    axs[i].plot(x, df_F_tau_grav.iloc[:,i].to_numpy(), label = "Tau minus grav")
    axs[i].plot(x, df_F_tau_filter.iloc[:,i].to_numpy(), label = "Tau filter")
    axs[i].plot(x, df_F_ext.iloc[:,i].to_numpy(), label = "F_filter")
    axs[i].plot(x, df_test.iloc[:,i].to_numpy(), label = "F_test")
    axs[i].set_ylabel(plot_name[i])
    axs[i].grid(True)
    #axs[i].set_ylim([df[useCols[1:]].min().min(), df[useCols[1:]].max().max()])
    axs[i].legend()

axs[-1].set_xlabel('Time')
# Adjust layout
plt.suptitle("Torque of each joint")
# Show the plot
plt.show()

# %% 
# plot gravity 
fig, axs = plt.subplots(1, 1, figsize=(8, 4), sharex=True)
x = df_tau_filter.index.values
labels = ["X", "Y", "Z", "rot_x", "rot_y", "rot_z"]
for i in range (6):
    axs.plot(x, df_F_grav.iloc[:,i].to_numpy(), label = labels[i])
    axs.grid(True)
axs.legend()
