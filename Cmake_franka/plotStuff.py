#%%
import numpy as np
import quaternion
import matplotlib.pyplot as plt
import matplotlib as mpl
import pandas as pd
import glob
import os


def readFile(path):
    list_of_files = glob.glob(path)
    filePath = max(list_of_files, key=os.path.getctime)
    array = np.loadtxt(filePath, delimiter=",")
    return array

def generateMissingAngles(angle_flex, angle_intern, startRange = -30, endRange=-20, tolerance=1):
    allAngles = np.linspace(startRange, endRange, endRange-startRange+1)
    angleMatrix = angle_flex[:, np.newaxis] - allAngles
    boolMatrix = np.abs(angleMatrix)<tolerance
    boolMatrixFilter = (boolMatrix.transpose() * np.sign(angle_intern)).transpose()
    boolValidAnglesIntern = ~np.any(boolMatrixFilter<0, axis=0)
    boolValidAnglesExtern = ~np.any(boolMatrixFilter>0, axis=0)
    validAnglesIntern = allAngles[boolValidAnglesIntern]
    validAnglesExtern = allAngles[boolValidAnglesExtern]

    return validAnglesIntern, validAnglesExtern

folder_path = "/home/alexandergerard/Masterarbeit/Cmake_franka/build/data_output_knee/"
np.set_printoptions(precision=3, suppress=True)


#%% Plot sphere
# draw sphere
r = 10
n_fine = 8
theta = np.linspace(0,np.pi/2, n_fine)
phi = np.linspace(0,2*np.pi, n_fine)
u, v = np.meshgrid(theta, phi)
x = r * np.sin(u)*np.cos(v)
y = r * np.sin(u)*np.sin(v)
z = r * np.cos(u)
n_grid = 8
theta_grid = np.linspace(0,np.pi/2, n_grid)
phi_grid = np.linspace(-3*np.pi/2, 0*np.pi/2, n_grid)
# theta_grid = np.array([np.pi/9, np.pi/8, np.pi/3, np.pi/2])
# phi_grid = np.array([np.pi/9, np.pi/8, np.pi/3, np.pi/2, 3*np.pi/2, 3.2*np.pi/2])
u_grid, v_grid = np.meshgrid(theta_grid, phi_grid)
x_grid = r * np.sin(u_grid)*np.cos(v_grid)
y_grid = r * np.sin(u_grid)*np.sin(v_grid)
z_grid = r * np.cos(u_grid)

u_surf, v_surf = np.meshgrid(theta_grid[0:2], phi_grid[1:3])
x_surf = r * np.sin(u_surf)*np.cos(v_surf)
y_surf = r * np.sin(u_surf)*np.sin(v_surf)
z_surf = r * np.cos(u_surf)

fig = plt.figure(figsize=(6,6))
ax = fig.add_subplot(projection="3d")
ax.view_init(azim=165, elev=40)
# ax.plot_surface(x, y, z, alpha = 0.4, cmap=plt.cm.coolwarm, linewidth=0, antialiased=False)
ax.plot_wireframe(x_grid, y_grid, z_grid, alpha=0.7, color="red")
# ax.plot_surface(x_surf, y_surf, z_surf, alpha = 1, color="black")
lim = (-10,10)
ax.set_xlim(lim)
ax.set_ylim(lim)
ax.set_zlim(lim)

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
# plt.show()


#%%
# quaternion takes w,x,y,z
startRange = -25
endRange=-10
tolerance=1
array_rot = readFile(folder_path + "quat_stop*")[:,:4]
# array_rot = readFile("/home/alexandergerard/Documents/quat_stop_tests.txt")[:,:4]
array_rot = array_rot[:,[3,0,1,2]]
angle_flex = 2*np.arctan(array_rot[:,1]/array_rot[:,0])*180/np.pi
angle_intern = 2*np.arctan(array_rot[:,2]/array_rot[:,0])*180/np.pi
test = np.array([angle_flex, angle_intern]).transpose()
print(f"These are the x- and y-angles of the quaternion: \n{test}")

allAnglesFlexIntern = np.arange(startRange, endRange+1)
allAnglesFlexExtern = np.arange(startRange, endRange+1)
allAnglesIntern = np.ones(len(allAnglesFlexIntern))*-10
allAnglesExtern = np.ones(len(allAnglesFlexExtern))*30
validIntern = np.array([allAnglesFlexIntern, allAnglesIntern])
validExtern = np.array([allAnglesFlexExtern, allAnglesExtern])

for i in range(len(angle_flex)):
    if angle_intern[i]<0: # is internal
        pos_in_allAngles = np.where(np.abs(validIntern[0]-angle_flex[i])<tolerance)[0]
        for j in range(pos_in_allAngles[-1]+1):
            if validIntern[1,j]<angle_intern[i]:
                validIntern[1,j] = angle_intern[i]
        validIntern = np.delete(validIntern, pos_in_allAngles, axis=1)
    elif angle_intern[i]>0: # is external 
        pos_in_allAngles = np.where(np.abs(validExtern[0]-angle_flex[i])<tolerance)[0]
        for j in range(pos_in_allAngles[-1]+1):
            if validExtern[1,j]>angle_intern[i]:
                validExtern[1,j] = angle_intern[i]
        validExtern = np.delete(validExtern, pos_in_allAngles, axis=1)

validExtern.transpose()
# test = np.array([angle_flex, angle_intern]).transpose()
# print(f"These are the x- and y-angles of the quaternion: \n{test}")
# validIntern, validExtern = generateMissingAngles(angle_flex, angle_intern, startRange,endRange,tolerance)
# print(f"Missing Intern: {validIntern.transpose()} \nMissing Extern: {validExtern.transpose()}")

#%%
# quaternion takes w,x,y,z
array_rot = readFile(folder_path + "quat_stop*")[:,:4]
array_rot = array_rot[:,[3,0,1,2]]
array_rot_base = array_rot
test_flex = np.arctan2(2*(array_rot_base[:,0]*array_rot_base[:,1]+array_rot_base[:,2]*array_rot_base[:,3]), 1-2*(array_rot_base[:,1]*array_rot_base[:,1]+array_rot_base[:,2]*array_rot_base[:,2]))*180/np.pi
test_intern = np.arcsin(2*(array_rot_base[:,0]*array_rot_base[:,2]-array_rot_base[:,3]*array_rot_base[:,1]))*180/np.pi
test_intern = np.arctan2(2*(array_rot_base[:,0]*array_rot_base[:,2]-array_rot_base[:,3]*array_rot_base[:,1]), np.sqrt(1-(2*(array_rot_base[:,0]*array_rot_base[:,2]+array_rot_base[:,3]*array_rot_base[:,1]))**2))*180/np.pi
angle_flex = 2*np.arctan(array_rot_base[:,1]/array_rot_base[:,0])*180/np.pi
angle_intern = 2*np.arctan(array_rot_base[:,2]/array_rot_base[:,0])*180/np.pi
test = np.array([test_flex, angle_flex, test_intern, angle_intern]).transpose()
np.set_printoptions(precision=3, suppress=True)
print(f"These are the x- and y-angles of the quaternion: \n{test}")

#%%
    # testing meshgrids

r=10
n_grid = 4
theta_grid = np.arange(n_grid)
phi_grid = np.arange(10, 10 + n_grid)
u_grid, v_grid = np.meshgrid(theta_grid, phi_grid)
x_grid = r * np.sin(u_grid)*np.cos(v_grid)
y_grid = r * np.sin(u_grid)*np.sin(v_grid)
z_grid = r * np.cos(u_grid)

# Try some colors
test = np.arange((n_grid-1)**2).reshape((n_grid-1), (n_grid-1))
norm = mpl.colors.Normalize(vmin=test.min(), vmax=test.max())
norm(test)
print(test)

# Plot some points
pts_start = 1
pts = 3

fig = plt.figure(figsize=(6,6))
ax = fig.add_subplot(projection="3d")
ax.plot_surface(x_grid, y_grid, z_grid, facecolors=plt.cm.jet(norm(test))) #, facecolors = colors

# ax.plot_surface(x_grid[pts_start:pts], y_grid[pts_start:pts], z_grid[pts_start:pts], alpha=0.7, color = "c")

m = mpl.cm.ScalarMappable(cmap=plt.cm.jet, norm=norm)
m.set_array([])
plt.colorbar(m)

lim = (-10,10)
ax.set_xlim(lim)
ax.set_ylim(lim)
ax.set_zlim(lim)

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
# plt.show()

#%%
# # Plot Accelerating factor
# nSteps = 20
# t = np.linspace(0,1,nSteps)
# t_dec = 1
# acc_0 = 1
# acc = np.array([])
# acc = np.append(acc, acc_0)
# factor = np.array([])
# reset:bool = True
# i=0
# resetPos = 5
# while i < (len(t)):
#     print(i)
#     if i>resetPos and reset==True:
#         print("here")
#         i=0
#         reset = False
#     factor = np.append(factor,0.5*(1+np.cos(np.pi * t[i]/t_dec)))
#     acc = np.append(acc, acc[-1] * 0.5*(1+np.cos(np.pi * t[i]/t_dec)))
#     i=i+1

# acc = acc[1:]


# plt.plot(acc, label="acceleration")
# plt.plot(factor, label="factor")
# plt.legend()
# plt.show()