#%%
import numpy as np
import matplotlib.pyplot as plt
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
plt.show()
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
plt.show()
