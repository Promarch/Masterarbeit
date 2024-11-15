#%%
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
import pandas as pd
import time
import glob
import os

def pd_readFile(path):
    list_of_files = glob.glob(path)
    filePath = max(list_of_files, key=os.path.getctime)
    df = pd.read_csv(filePath, header=0, delimiter=";", decimal=",", usecols=['Kraft absolut', 'Kraft', 'Zeit', 'Sollwert Kraft', 'Zyklen absolut', 'Zyklen', 'Zyklusnummer', 'Frequenz absolut', 'Sollwert absolut', 'Geschwindigkeit absolut', 'Normierter Stellwert', 'Stellwert absolut', 'Kolbenposition', 'Weg', 'Sollwert Weg', 'Kolbenposition in %', 'Weg in %']) # nrows=1000
    return df

def pd_readFile_full(path):
    list_of_files = glob.glob(path)
    filePath = max(list_of_files, key=os.path.getctime)
    df = pd.read_csv(filePath, header=0, delimiter=";", decimal=",") # nrows=1000
    return df

def np_readFile(path):
    list_of_files = glob.glob(path)
    filePath = max(list_of_files, key=os.path.getctime)
    array = np.loadtxt(filePath, delimiter=";", skiprows=3)
    return array

np.set_printoptions(precision=3, suppress=True)
#%%
folder_path = "/home/alexandergerard/Documents/"
t_start_full = time.time()
df_orig = pd_readFile(folder_path + 'Probe*')
t_end_full = time.time()
t_full = t_end_full-t_start_full
print(f"This took {t_full} s")
df = df_orig.copy()
# fängt bei Index 554 an
#df = df.drop(columns=['Beschleunigung absolut', 'Temperatur 2 absolut', 'Temperatur 3 absolut', 'Kraft absolut (Aufnehmerwert)', 'Beschleunigung', 'Kraft (Aufnehmerwert)', 'Dehnung absolut', 'Dehnung', 'Sollwert Dehnung', 'Temperatur absolut', 'Spannung absolut', 'Dehnung in %'])
df = df.iloc[555:].reset_index(drop=True)
#%%
# Plot the last 500 entries of the "Kraft" column
colPlot = "Weg"
colSecondary = "Kraft"
nStart = 65
nEnd = 12
x = df["Zeit"].values[-nStart:-nEnd]-df["Zeit"].values[-nStart]

fig, ax1 = plt.subplots(figsize=(10, 6))
line1, = ax1.plot(x, df[colPlot][-nStart:-nEnd].values, label=colPlot)
ax1.set_xlabel("Time [s]")
ax1.set_ylabel(f"{colPlot}")
ax1.grid(True)
ax2 = ax1.twinx()
line2, = ax2.plot(x,df[colSecondary][-nStart:-nEnd].values, color='red', label=colSecondary)
ax2.set_ylabel(f"{colSecondary}")
# ax2.tick_params(axis='y', labelcolor='red')
ax1.legend(handles=[line1, line2], loc='upper left')
maxCol = df[colPlot][-nStart:].values.max()
minCol = df[colPlot][-nStart:].values.max()
rangeCol = maxCol-minCol
# plt.legend()
# Display the plot
plt.show()
