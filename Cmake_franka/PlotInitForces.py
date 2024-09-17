# %%
import glob
import os

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# %% 
    # Import all data
folder_path = '/home/alexandergerard/Masterarbeit/Cmake_franka/build/data_grav/'
# Torque without gravity
list_of_files = glob.glob(folder_path+'tau_grav*')
filePath = max(list_of_files, key=os.path.getctime)
df_tau_grav = pd.read_csv(filePath, header=None)

# Torque filtered
list_of_files = glob.glob(folder_path+'tau_filter*')
filePath = max(list_of_files, key=os.path.getctime)
df_tau_filter = pd.read_csv(filePath, header=None)

# # Forces from torque without gravity
# list_of_files = glob.glob('/home/alexandergerard/Masterarbeit/Cmake_franka/build/data_grav/F_tau_grav*')
# filePath = max(list_of_files, key=os.path.getctime)
# df_F_tau_grav = pd.read_csv(filePath, header=None)

# Forces from filtered torque
list_of_files = glob.glob(folder_path+'F_tau_filter*')
filePath = max(list_of_files, key=os.path.getctime)
df_F_tau_filter = pd.read_csv(filePath, header=None)

# Force from robot measurement (until 22/07 14:20 in K_frame)
list_of_files = glob.glob(folder_path+'F_ext*')
filePath = max(list_of_files, key=os.path.getctime)
df_F_ext_orig = pd.read_csv(filePath, header=None)

# # %%
# # Low pass filter for data grav
# Fs = 1000
# F_cutoff = 5.0
# sig = np.array(df_tau_grav.iloc[:,3])

# fft_sig = np.fft.fft(sig)
# plt.plot(np.abs(fft_sig)/sig.size);

# cutoff_filter = 1.0 * np.abs(np.fft.fftfreq(fft_sig.size, 1.0/Fs)) <= F_cutoff
# sig_filtered = np.real(np.fft.ifft(fft_sig * cutoff_filter))
# plt.plot(np.abs(np.fft.fft(sig_filtered))/sig_filtered.size, '--')
# # plt.show()

# plt.plot(sig, '--')
# plt.plot(sig_filtered)
# plt.plot(df_tau_filter.iloc[:,3])
# # plt.show()

# %%
# Full Low pass filter
# from: https://colab.research.google.com/drive/1RR_9EYlApDMg4jAS2HuJIpSqwg5RLzGW?usp=sharing#scrollTo=KIbejmOR_rwI
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

#%%
sample_rate = 1000
F_cutoff = 10
df_tau_grav_filter = df_tau_grav.copy()
for i in range(np.size(df_tau_grav,1)):
    data = np.array(df_tau_grav.iloc[:,i])
    fft_sig = np.fft.fft(data)
    cutoff_filter = 1.0 * np.abs(np.fft.fftfreq(fft_sig.size, 1.0/sample_rate)) <= F_cutoff
    data_filter = np.real(np.fft.ifft(fft_sig * cutoff_filter))
    df_tau_grav_filter.iloc[:,i] = data_filter

# %%
# Plot torque

fig, axs = plt.subplots(7, 1, figsize=(10, 15), sharex=True)
x = df_tau_filter.index.values
for i in range(7):
    axs[i].plot(x, df_tau_grav.iloc[:,i].to_numpy(), label = "Tau minus grav")
    axs[i].plot(x, df_tau_filter.iloc[:,i].to_numpy(), label = "Tau filter")
    axs[i].plot(df_tau_grav_filter.iloc[:,i].to_numpy(), label = "Tau grav smooth")
    axs[i].set_ylabel(i+1)
    axs[i].grid(True)
    #axs[i].set_ylim([df[useCols[1:]].min().min(), df[useCols[1:]].max().max()])
    axs[i].legend(loc = "upper right")

axs[-1].set_xlabel('Time')
# Adjust layout
plt.suptitle("Torque of each joint")
# Show the plot
plt.show()


#%%
# imported code to plot
# Plot position history in 3D and absolute distance 
# Input is x,y,z as a dataframe
def plot_position(df):

    # Get Original position and calculate the absolute distance
    pos_origin = [0,0,0]
    distance_abs = np.linalg.norm(df-pos_origin, axis=1)

    # Plot
    fig = plt.figure(figsize=(8,8))
        # Plot original position and position during movement
    ax1 = fig.add_subplot(211, projection='3d')
    ax1.plot(df.iloc[1:,0].to_numpy(), df.iloc[1:,1].to_numpy(), df.iloc[1:,2].to_numpy())
    ax1.plot(pos_origin[0], pos_origin[1], pos_origin[2], 'o')
    # Set y-lim
#    ax1.set_xlim([df.iloc[:,0].min()-.1, df.iloc[:,0].min()+.1])
    # Set Label of axis
    ax1.set_xlabel("x")
    ax1.set_ylabel("y")
    ax1.set_zlabel("z")
        # Plot absolute distance from starting position 
    ax2 = fig.add_subplot(212)
    ax2.plot(distance_abs)

    plt.show()
    # Calculate the location of the applied force
pos_force = np.array([])
df_F_EE = pd.read_csv("build/force_EE_data.txt" , header=None)
df_F_ext = df_F_ext_orig[np.all(np.abs(df_F_ext_orig.iloc[:,0:3])>.05, axis=1)]
df_F_ext = df_F_EE 
try:
    for i in range(len(df_F_ext)):
        A = np.array(
            [[0, df_F_ext.iloc[i,2], df_F_ext.iloc[i,1]], 
            [df_F_ext.iloc[i,2], 0, df_F_ext.iloc[i,0]], 
            [df_F_ext.iloc[i,1], df_F_ext.iloc[i,0], 0]])
        b = np.array(
            [df_F_ext.iloc[i,3], df_F_ext.iloc[i,4], df_F_ext.iloc[i,5]])
        x = np.linalg.lstsq(A,b, rcond=None)
        pos_force = np.append(pos_force, x)
    
    pos_force = pos_force.reshape(int(len(pos_force)/4), 4)
    df_pos_force = pd.DataFrame(np.vstack(pos_force[:,0]))
    df_pos_test = df_pos_force[np.all(np.abs(df_pos_force)<50, axis=1)]
    # plot_position(df_pos_test)
except:

    print(f"Failed at step {i}")
    print("A = ", A)
    print("b = ", b)
    
# %%
    # Plot forces calculated with the jacobian
# Adjust the average values by the mean value after 250ms
df_F_ext = df_F_ext_orig
df_test = df_F_ext-df_F_ext.iloc[250:-1, :].mean()
fig, axs = plt.subplots(6, 1, figsize=(10, 15), sharex=True)
x = df_tau_filter.index.values
plot_name = ["$F_x$", "$F_y$", "$F_z$", r"$\tau _x$", r"$\tau _y$", r"$\tau _z$"]
for i in range(6):
    # axs[i].plot(x, df_F_tau_grav.iloc[:,i].to_numpy(), label = "Tau minus grav")
    axs[i].plot(x, df_F_tau_filter.iloc[:,i].to_numpy(), label = r"$^{unten}F$")
    axs[i].plot(x, df_F_ext.iloc[:,i].to_numpy(), "--", label = r"$^{Greifer}F$")
    #axs[i].plot(x, df_test.iloc[:,i].to_numpy(), label = "F_test")
    axs[i].set_ylabel(plot_name[i])
    axs[i].grid(True)
    #axs[i].set_ylim([df[useCols[1:]].min().min(), df[useCols[1:]].max().max()])
    axs[i].legend(loc = "upper right")

axs[-1].set_xlabel('Time')
# Adjust layout
plt.suptitle("Force/Torque in workspace")
# Show the plot
#plt.show()
