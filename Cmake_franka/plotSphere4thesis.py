#%%
# https://stackoverflow.com/questions/42924993/colorbar-for-matplotlib-plot-surface-using-facecolors
import glob
import os
import time
import quaternion
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
    # Calculate rotation around x-axis (flexion). Also rotate x-axis by 180° to ensure values don't jump from 180 to -180
    # O_T_EE_xAngle = O_T_EE_mat@np.reshape
    df_points["x_angle"] = np.round(np.arctan(O_T_EE[:,6], O_T_EE[:,10])*180/np.pi)
    # Calculate rotation around z-axis (internal-external rotation)
    df_points["z_angle"] = np.round(np.arctan2(O_T_EE[:,1], O_T_EE[:,0])*180/np.pi)
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
    if np.any(theta_signs==0):
        theta_signs[np.where(theta_signs==0)] = 1
    if np.any(phi_signs==0):
        phi_signs[np.where(phi_signs==0)] = 1

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
    df_isNaN = df_points.isna()
    if np.any(df_isNaN):
        nan_indices = np.where(df_isNaN)
        nan_positions = list(zip(nan_indices[0], nan_indices[1]))
        print("NaN Entries were found: ", nan_positions[-5:])
        df_points.dropna(inplace=True, ignore_index=True)

    return df_points

def CalculateColormap(df_points, df_surf_index, lim_forces=[]):
    # --------------------------------------------------------
    # -----          Add force to dataframes             -----
    # --------------------------------------------------------

    df_surf_index["force"] = np.nan
    df_surf_index["count"] = np.nan
    df_surf_index["n_angles"] = np.nan
    colored_surfaces = np.unique(df_points["surface"]).astype(int)
    # Add the average abs. torque to the surface dataframe
    for surf_id in colored_surfaces:
        pts = np.where(df_points["surface"]==surf_id)
        avrg_force = df_points.loc[pts, "force"].mean()
        avrg_force = df_points.loc[pts, "force"].quantile(0.9)
        n_angles = df_points.loc[pts, "z_angle"].nunique()
        df_surf_index.loc[surf_id, "n_angles"] = n_angles
        df_surf_index.loc[surf_id, "force"] = avrg_force
        df_surf_index.loc[surf_id, "count"] = np.size(pts)

    # --------------------------------------------------------
    # -----        Create colormap for surfaces          -----
    # --------------------------------------------------------

    # Get the force value of each surface
    forces_surf = df_surf_index.loc[colored_surfaces, "force"]
    # Normalize the forces for colormap (see: https://stackoverflow.com/questions/42924993/colorbar-for-matplotlib-plot-surface-using-facecolors)
    if lim_forces==[]: # If no limits for the colors given, use min/max values
        norm = mpl.colors.Normalize(vmin=forces_surf.min(), vmax=forces_surf.max())
    else:
        norm = mpl.colors.Normalize(vmin=lim_forces[0], vmax=lim_forces[1])
    forces_color = plt.cm.jet(norm(forces_surf))
    # Create color array where only valid surfaces get the colormap
    surf_opacity = 0.8
    # surf_opacity = 1/15*df_surf_index.loc[colored_surfaces, "n_angles"]
    # surf_opacity[surf_opacity>1]=1
    # surf_opacity[surf_opacity>0.3]=0.3
    
    forces_color[:,-1] = surf_opacity
    surf_colors = np.zeros([len(df_surf_index), 4])
    # Base color is white with low opacity
    surf_unused_opacity = 0.5
    surf_unused_color = [0.90, 0.90, 0.90, surf_unused_opacity]
    surf_colors = np.full((len(df_surf_index),4), surf_unused_color)
    # Add colors to the concerned surfaces
    surf_colors[colored_surfaces] = forces_color
    surf_colors = surf_colors.reshape(n_full-1,n_full-1,4)

    return surf_colors, norm

def quaternion_conjugate(q):
    # q = (x, y, z, w)
    if len(q)==4:
        return np.array([-q[0], -q[1], -q[2], q[3]])
    else:
        return np.array([-q[:,0], -q[:,1], -q[:,2], q[:,3]]).transpose()

def quaternion_multiply(q1, q2): # input is x,y,z,w
    # Multiply two quaternions q1 * q2
    if len(q1)==4:
        x1, y1, z1, w1 = q1
    else: 
        x1 = q1[:,0]
        y1 = q1[:,1]
        z1 = q1[:,2]
        w1 = q1[:,3]
    if len(q2)==4:
        x2, y2, z2, w2 = q2
    else:
        x2 = q2[:,0]
        y2 = q2[:,1]
        z2 = q2[:,2]
        w2 = q2[:,3]
    return np.array([
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    ]).transpose() # output is x,y,z,w

def rotate_vector_by_quaternion(v, q):
    # Rotate the vector v by quaternion q
    v_quat = np.array([v[0], v[1], v[2], 0])  # Convert vector to pure quaternion
    q_conjugate = quaternion_conjugate(q)

    # Apply quaternion rotation: q * v * q^-1
    rotated_v_quat = quaternion_multiply(quaternion_multiply(q, v_quat), q_conjugate)
    
    # The rotated vector is the first three components
    return rotated_v_quat[:,:-1]

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

np.set_printoptions(precision=3, suppress=True)
#%%
t_start = time.time()

    # Get Data
folder_path = "/home/alexandergerard/Masterarbeit/Cmake_franka/build/data_ball_joint_manual/"
folder_path = "/home/alexandergerard/Masterarbeit/Cmake_franka/build/data_output_knee/"
folder_path = "build/data_thesis/FullRun/"

# Determine if a valid temp variable is present
list_filePath_temp = glob.glob(folder_path + 'print_data.F_sensor*')
list_filePath_full = glob.glob(folder_path + 'F_sensor_total*')
# Modification time of the latest file of this name
t_modification_temp = os.path.getmtime(max(list_filePath_temp))
t_modification_full = os.path.getmtime(max(list_filePath_full))
# Add the filename of the temporary file to the filepath if is is newer
fileName_temp = ''
# if t_modification_temp>t_modification_full:
#     fileName_temp = 'print_data.'
#     print("Temp is newer than full one")
O_T_EE = readFile(folder_path + fileName_temp + 'O_T_EE*')
F_T_EE = readFile(folder_path + fileName_temp + 'F_T_EE*')
F_sensor = readFile(folder_path + fileName_temp + 'F_sensor*')

    # Create coordinates of a sphere with radius F_T_EE[0,14]
r = F_T_EE[0,14]
pos_EE = O_T_EE[0,12:15]
O_T_EE[:,12:15] = O_T_EE[:,12:15] - pos_EE
# Generate angle values for polar coordinates
n_full = 20
theta_full = np.linspace(0,np.pi/3, n_full)
phi_full = np.linspace(-np.pi,np.pi, n_full)
# phi_full = np.linspace(-3*np.pi/2,np.pi/4, n_full)
# Generate meshgrid in polar CoSy and transform into cartesian CoSy
theta_grid, phi_grid = np.meshgrid(np.arange(n_full), np.arange(n_full))
x_full, y_full, z_full = SphereCartesian(r, theta_full, phi_full, pos_EE)
factor_mm=1000
x_full=x_full*factor_mm
y_full=y_full*factor_mm
z_full=z_full*factor_mm
# Create dataframe to categorize surfaces
surf_columns = ['theta_0', 'theta_1', 'phi_0', 'phi_1']
surf_data = np.array([theta_grid[:-1,:-1].flatten(), theta_grid[1:,1:].flatten(), phi_grid[:-1,:-1].flatten(), phi_grid[1:,1:].flatten()])
df_surf_index = pd.DataFrame(surf_data.transpose(), columns=surf_columns)

# Create Point dataframe
samples = 1
df_points = createPointDf(O_T_EE, F_T_EE, F_sensor, df_surf_index, sample=samples)
# Calculate the colors of the individual surfaces
force_minmax = []
colors, norm = CalculateColormap(df_points, df_surf_index) #, lim_forces=force_minmax

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
max_range = r*0.7*factor_mm
mid_x = (x_max+x_min) * 0.5
mid_x = pos_EE[0]*factor_mm
mid_y = (y_max+y_min) * 0.5
mid_y = pos_EE[1]*factor_mm
mid_z = (z_max+z_min) * 0.5*factor_mm
total_time = np.round(time.time()-t_start, 3)
print(f"Time total calculation: {total_time}")
#%%
    # Plots
fontSize=12
pts_opacity = 0.9
plt.ion() # Turn on interactive mode
fig = plt.figure(figsize=(6,6))
ax = fig.add_subplot(111, projection="3d", computed_zorder=False)
# # Plot robot trajectory
# ax_trajectory = ax.plot(df_points["x"].values, df_points["y"].values, df_points["z"].values, "k-", zorder = 0)
# # Plot trajectory of virtual EE
# ax_EE = ax.plot(O_T_EE[:,12], O_T_EE[:,13], O_T_EE[:,14], "r-", zorder = -0.5)
# Plot points with force as color and add colorbar
# p = ax.scatter(df_points.loc[sorted_indices,"x"], df_points.loc[sorted_indices,"y"], df_points.loc[sorted_indices,"z"], alpha = pts_opacity, c = color_points, cmap=plt.cm.jet)
# fig.colorbar(p, ax=ax)
# Plot half sphere and add colorbar
ax_sphere = ax.plot_surface(x_full, y_full, z_full,facecolors=colors, linewidth=0, antialiased=False, zorder = 0.5) #
ax_sphere.set_facecolors(colors.reshape((n_full-1)**2,4))
# ax_wireframe = ax.plot_wireframe(x_full, y_full, z_full, linewidth=0.5, color=(0.6, 0.6, 0.6), alpha=0.3, zorder = -0.1)
m = mpl.cm.ScalarMappable(cmap=plt.cm.jet, norm=norm)
m.set_array([])
cbar = plt.colorbar(m,fraction=0.033, pad=0.12)
cbar.set_label("Norm of the measured torque [Nm]", rotation=-90, labelpad=20, fontsize=fontSize+2)
cbar.ax.tick_params(labelsize=fontSize)
ax.view_init(azim=165, elev=40)
ax.set_xlim(mid_x - max_range, mid_x + max_range)
ax.set_ylim(mid_y - max_range, mid_y + max_range)
ax.set_zlim(mid_z - max_range, mid_z + max_range)

ax.tick_params(labelsize=fontSize)
ax.set_xlabel('X [mm]', fontsize=fontSize+2, labelpad=10)
ax.set_ylabel('Y [mm]', fontsize=fontSize+2, labelpad=5)
ax.set_zlabel('Z [mm]', fontsize=fontSize+2, labelpad=5, rotation=90)

run_loop:bool = True
i:int = 0
while run_loop:
    # Draw plot
    fig.canvas.draw()
    fig.canvas.flush_events()

    # Check if there is a new finished file
    list_filePath_full = glob.glob(folder_path + 'F_sensor_total*')
    t_modification_full_new = os.path.getmtime(max(list_filePath_full))
    t_modification_temp_new = os.path.getmtime(max(list_filePath_temp))
    if t_modification_full>t_modification_temp: # Full file is newer than update file, so nothing to update
        # quat_stop_time = readFile(folder_path + "quat_stop_time*")
        # quat_base = quaternion_conjugate(np.array([ 0.7071068, 0.7071068, 0, 0 ]))
        # quat_stop = quaternion_multiply(quat_base, quat_stop_time[:,:-1])
        # for i in range(len(quat_stop_time)-1):
        #     if quat_stop_time[i+1,-1]<quat_stop_time[i,-1]:
        #         quat_stop_time[i+1:,-1] += 30
        # stop_time = quat_stop_time[:,-1]*1000
        # vec = np.array([0,0,-r])
        # pos_stop = rotate_vector_by_quaternion(vec, quat_stop)+pos_EE
        # pos_stop = df_points.loc[stop_time,["x", "y", "z"]].values
        # quat_rot_time = readFile(folder_path + "rotation_time*")
        # for i in range(len(quat_rot_time)-1):
        #     if quat_rot_time[i+1,-1]<quat_rot_time[i,-1]:
        #         quat_rot_time[i+1:,-1] += 30
        # rot_time = quat_rot_time[:,-1]*1000
        # vec = np.array([0,0,-r])
        # pos_rot = rotate_vector_by_quaternion(vec, quat_rot_time[:,:-1])+pos_EE
        # ax_scatter = ax.scatter(pos_stop[:,0], pos_stop[:,1], pos_stop[:,2], c="red", linewidth=1, alpha=1, zorder = 1)
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
            t_modification_temp = t_modification_temp_new
        except:
            print("Error occured during file reading")
        if len(O_T_EE) == len(F_T_EE) == len(F_sensor):
            # Calculate new points and colors
            df_points = createPointDf(O_T_EE, F_T_EE, F_sensor, df_surf_index, sample=samples)
            colors_new, norm_new = CalculateColormap(df_points, df_surf_index, lim_forces=force_minmax)
            # Update the facecolors and colorbar 
            ax_sphere.set_facecolors(colors_new.reshape((n_full-1)**2,4))
            m_new = mpl.cm.ScalarMappable(cmap=plt.cm.jet, norm=norm_new)
            m_new.set_array([])
            cbar.update_normal(m_new)
        else:
            print("Temporary data is not of same size")
    elif t_modification_full_new>=t_modification_temp:  # Full file has "appeared", stop updating
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
        # run_loop=False

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

# n_pts = 3
# x = np.arange(n_pts)
# X,Y = np.meshgrid(x,x)
# Y[1,:]= 1
# Z = np.cos(X)

#     # Create color array
# V = np.arange((n_pts-1)**2).reshape((n_pts-1), (n_pts-1))

# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')

# norm = mpl.colors.Normalize(vmin=V.min(), vmax=V.max())
# colors = plt.cm.jet(norm(V))
# colors[0,-1] = 0
# # print(f"Z: \n{Z} \nV: {V}\nColor: {colors}")
# ax.plot_surface(X, Y, Z, facecolors=colors, shade=False) #

# m = mpl.cm.ScalarMappable(cmap=plt.cm.jet, norm=norm)
# m.set_array([])
# plt.colorbar(m)

# ax.set_xlabel('x')
# ax.set_ylabel('y')

# plt.show()

