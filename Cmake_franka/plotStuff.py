#%%
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
import pandas as pd
#%%
# Plot Acceleration and deceleration factor
period_acc = 4
t = np.linspace(0,period_acc,100)
y_acc = (1 - np.cos(np.pi * t/period_acc))/2
y_dec = (1 + np.cos(np.pi * t/period_acc))/2

plt.plot(t,y_acc, label=r"$\alpha_{acc}$")
plt.plot(t,y_dec, label=r"$\alpha_{dec}$")
plt.grid(False)
plt.title("Acceleration factors")
plt.xlabel("Time [s]")
plt.legend(loc="center right")
# plt.show()
#%%
# Plot Rotation of EE
def plot_quiver(X,Y,axs):
    quiver_angle = np.arctan([Y+1]/X)
    axs.quiver(X,Y,np.cos(quiver_angle), np.sin(quiver_angle))
    axs.quiver(X,Y,np.cos(quiver_angle+np.pi/2), np.sin(quiver_angle+np.pi/2))
def plot_text(X,Y,axs):
    for i in range(len(X)):
        axs.text(X[i]+0.02, Y[i]-0.05, rf"$x_{i}$") # :[{np.round(X[i],2)}, {np.round(Y[i],2)}]

angle = np.pi/4
t = np.linspace(angle*2,angle,100)
r = 1
x = r*np.cos(t)
y = r*np.sin(t)-r

fig, axs = plt.subplots(figsize=(4,4))
axs.plot(x,y)
plot_quiver(x[[0,-1]],y[[0,-1]],axs)
# plot_text(x[[0,-1]],y[[0,-1]],axs)
axs.text(0.04,-r-0.01, rf"$x_0 = x_1$")
axs.plot(0,-r, 'b+')
axs.plot([x[-1],0], [y[-1],-r], 'b--')
axs.text(0.4, -1+0.4-0.05, "r")
y_lim = [-1.1, 0.1]
x_lim = [-0.1, 1.1]
#axs.set_xlim(x_lim)
#axs.set_ylim(y_lim)
axs.grid(True)
axs.set_axis_off()
axs.axis("equal")
# plt.show()

#%%
    # Plot sphere
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
theta_grid = np.linspace(0,np.pi/2, n_grid)[1:]
phi_grid = np.linspace(0,np.pi*2, n_grid)
theta_grid = np.array([np.pi/9, np.pi/8, np.pi/3, np.pi/2])
phi_grid = np.array([np.pi/9, np.pi/8, np.pi/3, np.pi/2, 3*np.pi/2, 3.2*np.pi/2])
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
plt.show()



#%%
    # Not really plotting, only testing meshgrid

n_grid = 5
x_range = np.arange(n_grid)
y_range = np.arange(10, 10+n_grid)
u,v = np.meshgrid(x_range, y_range)
print("u: \n", u, "\nv: \n", v)

for i in range(n_grid):
    print(f"u_test: {u[i]}, v_test: {v[i]}")

# dic = {"theta_0": u_grid[:-1,:-1].flatten(), "theta_1":u_grid[1:,1:].flatten(), "phi_0":v_grid[:-1,:-1].flatten(), "phi_1":v_grid[1:,1:].flatten()}
# df = pd.DataFrame(dic)
# print(df)