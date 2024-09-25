#%%
# https://stackoverflow.com/questions/42924993/colorbar-for-matplotlib-plot-surface-using-facecolors
import glob
import os
import time
import datetime
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl


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

def readFile(path):
    list_of_files = glob.glob(path)
    filePath = max(list_of_files, key=os.path.getctime)
    array = np.loadtxt(filePath, delimiter=",")
    return array

def createPointDf(O_T_EE, F_T_EE, F_sensor, df_surf_index, sample=1):

    # Sample down the datapoints
    O_T_EE = O_T_EE[::sample, :]
    F_T_EE = F_T_EE[::sample, :]
    F_sensor = F_sensor[::sample, :]
    # Reshape the nx16 array into an array of n 4x4 matrices
    size_mat = len(O_T_EE)
    O_T_EE_mat = np.transpose(O_T_EE.reshape(size_mat,4,4), axes=(0,2,1))
    F_T_EE_mat = np.transpose(F_T_EE.reshape(size_mat,4,4), axes=(0,2,1))

    # Calculate the transformation matrix from base to flange
    O_T_F_mat = O_T_EE_mat@np.linalg.inv(F_T_EE_mat)
    O_T_F_flat = O_T_F_mat.reshape(size_mat,16)
    # Create point dataframe
    pos_matrix = O_T_F_flat#[:-1:100,:] # position of the flange
    dic_points = {"x": pos_matrix[:,3], "y": pos_matrix[:,7], "z": pos_matrix[:,11]}
    df_points = pd.DataFrame(dic_points)
    # Calculate theta and phi angle of the data points
    df_points["theta"], df_points["phi"] = CartSphere(df_points["x"], df_points["y"], df_points["z"], pos_EE)

    # Add abs. torque to each point
    df_points["force"] =  np.linalg.norm(F_sensor[:,3:], axis=1) #:-1:100

    # --------------------------------------------------------
    # -----          Get Surface of datapoint            -----
    # --------------------------------------------------------

        # Find closest gridpoints to data point
    # Convert df_points to numpy arrays for faster operations
    theta_points = df_points["theta"].to_numpy()
    phi_points = df_points["phi"].to_numpy()
    # Calculate differences for all points with broadcasting
    theta_diff = theta_full[:, np.newaxis] - theta_points
    phi_diff = phi_full[:, np.newaxis] - phi_points
    # Find the indices of the closest gridpoints
    theta_indices = np.abs(theta_diff).argmin(axis=0)  # Closest theta index for each point (size: 25000)
    phi_indices = np.abs(phi_diff).argmin(axis=0)      # Closest phi index for each point (size: 25000)
    # Calculate second closest gridpoints
    theta_signs = np.sign(theta_diff[theta_indices, np.arange(len(theta_indices))])
    phi_signs = np.sign(phi_diff[phi_indices, np.arange(len(phi_indices))])
    # Create arrays with the closest and second closest gridpoints
    theta_surf = np.vstack((theta_indices, theta_indices - theta_signs)).astype(int).T
    phi_surf = np.vstack((phi_indices, phi_indices - phi_signs)).astype(int).T
    # Reshape the angle indexes to size [n x 2]
    phi_surf = np.sort(phi_surf)
    theta_surf = np.sort(theta_surf)
    # Concatenate the results
    angles_together = np.concatenate([theta_surf, phi_surf], axis=1)

        # Match the surfaces of each datapoint to a surface on the sphere
    # Use broadcasting and np.all() to compare each row; colNames are explicit cause otherwise the forces would also be considered
    matches = np.all(df_surf_index[['theta_0', 'theta_1', 'phi_0', 'phi_1']].values[:, None] == angles_together, axis=2)
    # Find the indices where there is a match
    matching_indices = np.where(matches)
    df_points.loc[matching_indices[1], "surface"] = matching_indices[0]

    return df_points

def CalculateColormap(df_points, df_surf_index, lim_forces=[]):
    # --------------------------------------------------------
    # -----          Add force to dataframes             -----
    # --------------------------------------------------------

    df_surf_index["force"] = np.nan
    df_surf_index["count"] = np.nan
    colored_surfaces = np.unique(df_points["surface"]).astype(int)
    # Add the average abs. torque to the surface dataframe
    for surf_id in colored_surfaces:
        pts = np.where(df_points["surface"]==surf_id)
        avrg_force = df_points.loc[pts, "force"].mean()
        df_surf_index.loc[surf_id, "force"] = avrg_force
        df_surf_index.loc[surf_id, "count"] = np.size(pts)

    # --------------------------------------------------------
    # -----        Create colormap for surfaces          -----
    # --------------------------------------------------------

    # Get the force value of each surface
    forces_surf = df_surf_index.loc[colored_surfaces, "force"]
    # Normalize the forces for colormap (see: https://stackoverflow.com/questions/42924993/colorbar-for-matplotlib-plot-surface-using-facecolors)
    if lim_forces==[]:
        norm = mpl.colors.Normalize(vmin=forces_surf.min(), vmax=forces_surf.max())
    else:
        norm = mpl.colors.Normalize(vmin=lim_forces[0], vmax=lim_forces[1])
    forces_color = plt.cm.jet(norm(forces_surf))
    # Create color array where only valid surfaces get the colormap
    surf_opacity = 0.8
    forces_color[:,-1] = surf_opacity
    surf_colors = np.zeros([len(df_surf_index), 4])
    # Base color is white with low opacity
    surf_unused_opacity = 0.2
    surf_unused_color = [0.99, 0.99, 0.99, surf_unused_opacity]
    surf_colors = np.full((len(df_surf_index),4), surf_unused_color)
    # Add colors to the concerned surfaces
    surf_colors[colored_surfaces] = forces_color
    surf_colors = surf_colors.reshape(n_full-1,n_full-1,4)

    return surf_colors, norm


#%%
t_start = time.time()

    # Get Data
folder_path = "/home/alexandergerard/Masterarbeit/Cmake_franka/build/data_ball_joint_manual/"
# Determine if a valid temp variable is present
list_filePath_temp = glob.glob(folder_path + 'print_data.F_sensor*')
list_filePath_full = glob.glob(folder_path + 'F_sensor_total*')
# Modification time of the latest file of this name
t_modification_temp = os.path.getmtime(max(list_filePath_temp))
t_modification_full = os.path.getmtime(max(list_filePath_full))
# Add the filename of the temporary file to the filepath if is is newer
fileName_temp = ''
if t_modification_temp>t_modification_full:
    fileName_temp = 'print_data.'
    print("Temp is newer than full one")
O_T_EE = readFile(folder_path + fileName_temp + 'O_T_EE*')
F_T_EE = readFile(folder_path + fileName_temp + 'F_T_EE*')
F_sensor = readFile(folder_path + fileName_temp + 'F_sensor*')

    # Create coordinates of a sphere with radius F_T_EE[0,14]
r = F_T_EE[0,14]
pos_EE = O_T_EE[0,12:15]
# Generate angle values for polar coordinates
n_full = 10
theta_full = np.linspace(0,np.pi/2, n_full)
phi_full = np.linspace(-np.pi,np.pi, n_full)
# Generate meshgrid in polar CoSy and transform into cartesian CoSy
theta_grid, phi_grid = np.meshgrid(np.arange(n_full), np.arange(n_full))
x_full, y_full, z_full = SphereCartesian(r, theta_full, phi_full, pos_EE)
# Create dataframe to categorize surfaces
surf_columns = ['theta_0', 'theta_1', 'phi_0', 'phi_1']
surf_data = np.array([theta_grid[:-1,:-1].flatten(), theta_grid[1:,1:].flatten(), phi_grid[:-1,:-1].flatten(), phi_grid[1:,1:].flatten()])
df_surf_index = pd.DataFrame(surf_data.transpose(), columns=surf_columns)

# Create Point dataframe
samples = 5
df_points = createPointDf(O_T_EE, F_T_EE, F_sensor, df_surf_index, sample=samples)
# Calculate the colors of the individual surfaces
colors, norm = CalculateColormap(df_points, df_surf_index, lim_forces=[0,1])

    # DEBUG: Calculate nearest gridpoint to sample point of x,y,z
sample_point = 0
theta_sample, phi_sample = CartSphere(df_points.loc[sample_point, "x"], df_points.loc[sample_point, "y"], df_points.loc[sample_point, "z"], pos_EE)
theta_diff_sample = theta_full - theta_sample
theta_index_sample = np.abs(theta_diff_sample).argmin()
theta_surf_sample = np.array([theta_index_sample, theta_index_sample - np.sign(theta_diff_sample[theta_index_sample])]).astype(int)
phi_diff_sample = phi_full - phi_sample
phi_index_sample = np.abs(phi_diff_sample).argmin()
phi_surf_sample = np.array([phi_index_sample, phi_index_sample - np.sign(phi_diff_sample[phi_index_sample])]).astype(int)
x_sample, y_sample, z_sample = SphereCartesian(r, theta_full[theta_index_sample], phi_full[phi_index_sample], pos_EE)


    # Get ranges for the plot
# Calculate maximum values
x_max = np.max((df_points["x"].max(), O_T_EE[:,12].max()))
x_min = np.min((df_points["x"].min(), O_T_EE[:,12].min()))
y_max = np.max((df_points["y"].max(), O_T_EE[:,13].max()))
y_min = np.min((df_points["y"].min(), O_T_EE[:,13].min()))
z_max = np.max((df_points["z"].max(), O_T_EE[:,14].max()))
z_min = np.min((df_points["z"].min(), O_T_EE[:,14].min()))
# Calculate limits of the plot
max_range = np.array([x_max-x_min, y_max-y_min, z_max-z_min]).max() / 2.0
max_range = r
mid_x = (x_max+x_min) * 0.5
mid_y = (y_max+y_min) * 0.5
mid_z = (z_max+z_min) * 0.5
total_time = np.round(time.time()-t_start, 3)
print(f"Time total calculation: {total_time}")
#%%
    # Plots
pts_opacity = 0.9
plt.ion() # Turn on interactive mode
fig = plt.figure(figsize=(6,6))
ax = fig.add_subplot(111, projection="3d")
# Plot robot trajectory
ax.plot(df_points["x"].values, df_points["y"].values, df_points["z"].values, "k-")
# Plot trajectory of virtual EE
ax.plot(O_T_EE[:,12], O_T_EE[:,13], O_T_EE[:,14], "r-")
# Plot points with force as color and add colorbar
# p = ax.scatter(df_points.loc[sorted_indices,"x"], df_points.loc[sorted_indices,"y"], df_points.loc[sorted_indices,"z"], alpha = pts_opacity, c = color_points, cmap=plt.cm.jet)
# fig.colorbar(p, ax=ax)
# Plot half sphere and add colorbar
ax_sphere = ax.plot_surface(x_full, y_full, z_full,facecolors=colors, linewidth=0, antialiased=False) #
m = mpl.cm.ScalarMappable(cmap=plt.cm.jet, norm=norm)
m.set_array([])
cbar = plt.colorbar(m)
ax.view_init(elev=22, azim=22)
ax.set_xlim(mid_x - max_range, mid_x + max_range)
ax.set_ylim(mid_y - max_range, mid_y + max_range)
ax.set_zlim(mid_z - max_range, mid_z + max_range)

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

run_loop:bool = True
i:int = 0
while True:
    # Draw plot
    fig.canvas.draw()
    fig.canvas.flush_events()

    # Check if there is a new finished file
    list_filePath_full = glob.glob(folder_path + 'F_sensor_total*')
    t_modification_full_new = os.path.getmtime(max(list_filePath_full))
    t_modification_temp_new = os.path.getmtime(max(list_filePath_temp))
    if t_modification_full>t_modification_temp: # Full file is newer than update file, so nothing to update
        plt.show()
    elif t_modification_temp_new>t_modification_temp:   # Temp file was updated
        print("New update for temp file, i=",i)
        i=i+1
        fileName_temp = 'print_data.'
        # Read new data
        try:
            O_T_EE = readFile(folder_path + fileName_temp + 'O_T_EE*')
            F_T_EE = readFile(folder_path + fileName_temp + 'F_T_EE*')
            F_sensor = readFile(folder_path + fileName_temp + 'F_sensor*')
        except:
            print("Error occured during file reading")
        if len(O_T_EE) == len(F_T_EE) == len(F_sensor):
            # Calculate new points and colors
            df_points = createPointDf(O_T_EE, F_T_EE, F_sensor, df_surf_index)
            colors_new, norm_new = CalculateColormap(df_points, df_surf_index)
            # Update the facecolors and colorbar 
            ax_sphere.set_facecolors(colors_new.reshape((n_full-1)**2,4))
            m_new = mpl.cm.ScalarMappable(cmap=plt.cm.jet, norm=norm_new)
            m_new.set_array([])
            cbar.update_normal(m_new)
        else:
            print("Temporary data is not of same size")
    elif t_modification_full_new>t_modification_temp and run_loop:  # Full file has "appeared", stop updating
        print("Full File now available")
        fileName_temp = ''
        # Read new data
        O_T_EE = readFile(folder_path + fileName_temp + 'O_T_EE*')
        F_T_EE = readFile(folder_path + fileName_temp + 'F_T_EE*')
        F_sensor = readFile(folder_path + fileName_temp + 'F_sensor*')
        # Calculate new points and colors
        df_points = createPointDf(O_T_EE, F_T_EE, F_sensor, df_surf_index)
        colors_new, norm_new = CalculateColormap(df_points, df_surf_index)
        # Update the facecolors and colorbar 
        ax_sphere.set_facecolors(colors_new.reshape((n_full-1)**2,4))
        m_new = mpl.cm.ScalarMappable(cmap=plt.cm.jet, norm=norm_new)
        m_new.set_array([])
        cbar.update_normal(m_new)
        run_loop=False

plt.show()

#--------------------
#--  Debug plots   --
#--------------------

#     # Plot nearest gridpoint to testpoint
# ax.plot(x_sample, y_sample, z_sample, "mo", markersize=10)
# ax.plot(x[sample_point], y[sample_point], z[sample_point], "co", markersize=15)
# ax.plot_surface(x_surf, y_surf, z_surf, alpha = 0.6, color="black")
#     # Plot sample surfaces
# surf_to_plot = np.array([4, 30])
# for surf_id in surf_to_plot: 
#     theta_index = df_surf_index.loc[surf_id,["theta_0", "theta_1"]].astype(int)
#     phi_index = df_surf_index.loc[surf_id,["phi_0", "phi_1"]].astype(int)   
#     x_surf, y_surf, z_surf = SphereCartesian(r, theta_full[theta_index], phi_full[phi_index], pos_EE)    
#     ax.plot_surface(x_surf, y_surf, z_surf, alpha = 0.95, color="black")


#%%

n_pts = 3
x = np.arange(n_pts)
X,Y = np.meshgrid(x,x)
Y[1,:]= 1
Z = np.cos(X)

    # Create color array
V = np.arange((n_pts-1)**2).reshape((n_pts-1), (n_pts-1))

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

norm = mpl.colors.Normalize(vmin=V.min(), vmax=V.max())
colors = plt.cm.jet(norm(V))
colors[0,-1] = 0
# print(f"Z: \n{Z} \nV: {V}\nColor: {colors}")
ax.plot_surface(X, Y, Z, facecolors=colors, shade=False) #

m = mpl.cm.ScalarMappable(cmap=plt.cm.jet, norm=norm)
m.set_array([])
plt.colorbar(m)

ax.set_xlabel('x')
ax.set_ylabel('y')

# plt.show()

