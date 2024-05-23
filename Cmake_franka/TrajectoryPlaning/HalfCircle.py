# %%

import numpy as np
import matplotlib.pyplot as plt

    # Variables
# basic geometric paramaters
r = 1
x = [0] * r
y = [1] * r
alpha = np.pi/12
# Runtime paramters
t = 1   # time
stepsize = 0.1
n_points = 10   # number of intervalls
n = 1   #  counter for the angles
#debug parameters
counter = 1

def plot_xy (x,y) : 
    plt.plot(x,y, '+')
    plt.axis('equal')
    plt.show()

while y[-1]>-0.1 : 
    t = t+1
    print(np.rad2deg(np.arctan(x[-1]/y[-1])), "\n")
    if np.abs(np.arctan(x[-1]/y[-1]))<=np.abs(n*alpha) and (((x[-1]**2+y[-1]**2)*1.01)>=r**2): 
        x = np.append(x, r * np.cos(np.pi/2 + alpha*(n-np.sign(n)) + alpha*np.sign(n) * (t*stepsize)))
        y = np.append(y, r * np.sin(np.pi/2 + alpha*(n-np.sign(n)) + alpha*np.sign(n) * (t*stepsize)))
        if np.abs(np.arctan(x[-1]/y[-1]))>np.abs(n*alpha) :
            t=0
    else :
        x = np.append(x, r*np.cos(np.pi/2 + n*alpha * (1 - t*stepsize)))
        y = np.append(y, y[-1])
        if x[-1]**2+y[-1]**2>=r :
            n = -(n+np.sign(n))
            t=0
#    if (x^2+y^2)<r^2 :



plt.plot(x,y, '+')
plt.axis('equal')
plt.show()
# %%

