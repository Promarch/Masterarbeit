import numpy as np
import matplotlib.pyplot as plt

period_acc = 4
t = np.linspace(0,period_acc,100)
y = (1 - np.cos(np.pi * t/period_acc))/2

plt.plot(t,y)
plt.show()