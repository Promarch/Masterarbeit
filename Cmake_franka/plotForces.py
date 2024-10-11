#%%
import glob
import os
import time

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
from matplotlib.animation import FuncAnimation

def readFile(path):
    list_of_files = glob.glob(path)
    filePath = max(list_of_files, key=os.path.getctime)
    array = np.loadtxt(filePath, delimiter=",")
    return array

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


def calculateQuiver(pos_matrix, orient_matrix, axe=0):
    # This function assumes all matrices have the same size and shape(nx16), and that the entries 12,13,14 are the positions
        # Position
    x=pos_matrix[12]
    y=pos_matrix[13]
    z=pos_matrix[14]
        # Orientation
    if axe<0 or axe>2:
        print("No correct axe, values must be [0,1,2], your value was:", axe)
        return
    u=orient_matrix[0+axe*4]
    v=orient_matrix[1+axe*4]
    w=orient_matrix[2+axe*4]
    return x,y,z,u,v,w

def calculateColor(Force, F_lim=[], l_max=[]): # Force needs the shape (nx3)
    # Calculate color    
    if F_lim==[]: # If no limits for the colors given, use min/max values
        F_max = Force.max()
        norm = mpl.colors.Normalize(vmin=np.abs(Force).min(), vmax=Force.max())
    else:
        F_max = F_lim[1]
        norm = mpl.colors.Normalize(vmin=F_lim[0], vmax=F_lim[1])
    color = plt.cm.jet(norm(np.abs(Force)))
    # Calculate quiver length if given
    if l_max==[]:
        return norm, color
    else: 
        l_quiver = Force/F_max 
        l_quiver[l_quiver>1]=1
        l_quiver[l_quiver<-1]=-1
        l_quiver = l_quiver * l_max
        return norm, color, l_quiver

# folder path
folder_path = "/home/alexandergerard/Masterarbeit/Cmake_franka/build/data_grav/"
folder_path = "/home/alexandergerard/Masterarbeit/Cmake_franka/build/data_ball_joint/"
folder_path = "/home/alexandergerard/Masterarbeit/Cmake_franka/build/data_ball_joint_manual/"
folder_path = "/home/alexandergerard/Masterarbeit/Cmake_franka/build/data_output_knee/"
# folder_path = "/home/alexandergerard/Masterarbeit/Cmake_franka/build/data_thesis/"

#%%
    # Plot position and orientation
# Get Data
O_T_EE_orig = readFile(folder_path + 'O_T_EE*')
F_T_EE_orig = readFile(folder_path + 'F_T_EE*')
wrench_sensor_orig = readFile(folder_path + 'F_sensor_to*')

# Sample down data if needed
sample = 1
O_T_EE = O_T_EE_orig[:-1:sample,:]
F_T_EE = F_T_EE_orig[:-1:sample,:]
wrench_sensor = wrench_sensor_orig[:-1:sample,:]
F_sensor = wrench_sensor[:,:3]
tau_sensor = wrench_sensor[:,3:]

# Reshape the nx16 array into an array of n 4x4 matrices
size_mat = len(O_T_EE)
O_T_EE_mat = np.transpose(O_T_EE.reshape(size_mat,4,4), axes=(0,2,1))
F_T_EE_mat = np.transpose(F_T_EE.reshape(size_mat,4,4), axes=(0,2,1))

# Calculate the transformation matrix from base to flange
O_T_F_mat = O_T_EE_mat@np.linalg.inv(F_T_EE_mat)
O_T_F = np.transpose(O_T_F_mat, axes=(0,2,1)).reshape(size_mat,16)


    # Calculate torque (torque is not measured in the center, so I need to add the F*distance)
dist_EE_Sensor = F_T_EE[0,14]-0.052
tau_sensor = tau_sensor + F_sensor[:,[1,0,2]] * [dist_EE_Sensor, -dist_EE_Sensor, 0]
# Calculate color
l_quiver = 0.03
tau_norm, tau_color, tau_length = calculateColor(tau_sensor, F_lim=[], l_max=l_quiver*3)
F_norm, F_color_full = calculateColor(F_sensor, F_lim=[], l_max=[])


    # Create sphere for visualization (not animated)
# Generate angle values for polar coordinates
n_full = 20
theta_full = np.linspace(0,np.pi/2, n_full)
phi_full = np.linspace(-np.pi,np.pi, n_full)
# Generate meshgrid in polar CoSy and transform into cartesian CoSy
theta_grid, phi_grid = np.meshgrid(np.arange(n_full), np.arange(n_full))
x_full, y_full, z_full = SphereCartesian(F_T_EE[0,14], theta_full, phi_full, O_T_EE[0,12:15])

# Get ranges for the plot
x_max = np.max((O_T_F[0,12].max(), O_T_EE[:,12].max()))
x_min = np.min((O_T_F[0,12].min(), O_T_EE[:,12].min()))
y_max = np.max((O_T_F[0,13].max(), O_T_EE[:,13].max()))
y_min = np.min((O_T_F[0,13].min(), O_T_EE[:,13].min()))
z_max = np.max((O_T_F[0,14].max(), O_T_EE[:,14].max()))
z_min = np.min((O_T_F[0,14].min(), O_T_EE[:,14].min()))

max_range = np.array([x_max-x_min, y_max-y_min, z_max-z_min]).max() / 2.0
mid_x = (x_max+x_min) * 0.5
mid_y = (y_max+y_min) * 0.5
mid_z = (z_max+z_min) * 0.5
#%%
# Try to emulate real time robot movement
t=0

# Plot initial state
fig = plt.figure(figsize=(6,6))
ax = fig.add_subplot(projection="3d")
# Plot torque quiver
# x[t], y[t], z[t], orient_matrix[t,0], orient_matrix[t,1], orient_matrix[t,2]
quiv_taux = ax.quiver(*calculateQuiver(O_T_F, O_T_EE, axe=0), color = tau_color[t,0,:], length=l_quiver) #, color = tau_color[:,0,:], length=l_quiver
quiv_tauy = ax.quiver(*calculateQuiver(O_T_F, O_T_EE, axe=1), color = tau_color[t,1,:], length=l_quiver) #, color = tau_color[:,0,:], length=l_quiver
quiv_tauz = ax.quiver(*calculateQuiver(O_T_F, O_T_EE, axe=2), color = tau_color[t,2,:], length=l_quiver) #, color = tau_color[:,0,:], length=l_quiver
# Plot force quiver
quiv_x = ax.quiver(*calculateQuiver(O_T_EE, O_T_EE, axe=0), color = F_color_full[t,0,:], length=l_quiver) #, color = tau_color[:,0,:], length=l_quiver
quiv_y = ax.quiver(*calculateQuiver(O_T_EE, O_T_EE, axe=1), color = F_color_full[t,1,:], length=l_quiver) #, color = tau_color[:,0,:], length=l_quiver

ax.plot_surface(x_full, y_full, z_full, color = [0.9, 0.9, 0.9, 0.1], linewidth=0, antialiased=False)
ax.plot(O_T_F[:,12], O_T_F[:,13], O_T_F[:,14], "k--")
ax.plot(O_T_EE[:,12], O_T_EE[:,13], O_T_EE[:,14], "r-")

ax.view_init(elev=22, azim=22)
ax.set_xlim(mid_x - max_range, mid_x + max_range)
ax.set_ylim(mid_y - max_range, mid_y + max_range)
ax.set_zlim(mid_z - max_range, mid_z + max_range)

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
# plt.show()

fps = 10
t_interval = 1000/fps
n_frames = round(len(O_T_EE)*0.001*fps)
frames = np.linspace(0,len(O_T_EE)-1, n_frames).astype(int)

def update(t):
    # Set quiver variables global
    global quiv_x, quiv_y, quiv_taux, quiv_tauy, quiv_tauz
    # Remove old quiver
    quiv_x.remove()
    quiv_y.remove()
    quiv_taux.remove()
    quiv_tauy.remove()
    quiv_tauz.remove()
    # Plot new quiver
    quiv_x = ax.quiver(*calculateQuiver(O_T_EE[t], O_T_EE[t], axe=0), color = F_color_full[t,0,:], length=l_quiver)
    quiv_y = ax.quiver(*calculateQuiver(O_T_EE[t], O_T_EE[t], axe=1), color = F_color_full[t,0,:], length=l_quiver)
    quiv_taux = ax.quiver(*calculateQuiver(O_T_F[t], O_T_EE[t], axe=0), color = tau_color[t,0,:], length=l_quiver)
    quiv_tauy = ax.quiver(*calculateQuiver(O_T_F[t], O_T_EE[t], axe=1), color = tau_color[t,1,:], length=tau_length[t,1])
    quiv_tauz = ax.quiver(*calculateQuiver(O_T_F[t], O_T_EE[t], axe=2), color = tau_color[t,2,:], length=l_quiver)

    print("Time: ", t)

ani = FuncAnimation(fig, update, frames=frames, interval=t_interval)
plt.show()
