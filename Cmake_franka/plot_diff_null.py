#%%
import glob
import os

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

#%%
    # Get data
folder_path = '/home/alexandergerard/Masterarbeit/Cmake_franka/build/nullspace_test/'
folder_path = 'C:/Users/Alex Benutzer/Documents/Masterarbeit_gesamt/Masterarbeit/Cmake_franka/build/nullspace_test/'
# Position without nullspace
list_of_files_pos_norm = glob.glob(folder_path+'position_data_20240809_170329.txt')
filePath_pos_norm = max(list_of_files_pos_norm, key=os.path.getctime)
pos_norm = np.loadtxt(filePath_pos_norm, delimiter=",")
# Position with nullspace 
list_of_files_pos_null = glob.glob(folder_path+'position_data_20240809_170454.txt')
filePath_pos_null = max(list_of_files_pos_null, key=os.path.getctime)
pos_null_100 = np.loadtxt(filePath_pos_null, delimiter=",")
# Position with nullspace 
list_of_files_pos_null = glob.glob(folder_path+'position_data_20240809_173340.txt')
filePath_pos_null = max(list_of_files_pos_null, key=os.path.getctime)
pos_null_400 = np.loadtxt(filePath_pos_null, delimiter=",")
# Position with nullspace 
list_of_files_pos_null = glob.glob(folder_path+'position_data_20240809_173915.txt')
filePath_pos_null = max(list_of_files_pos_null, key=os.path.getctime)
pos_null_1000 = np.loadtxt(filePath_pos_null, delimiter=",")
# Position with nullspace 
list_of_files_pos_null = glob.glob(folder_path+'position_data_20240809_180014.txt')
filePath_pos_null = max(list_of_files_pos_null, key=os.path.getctime)
pos_null_300 = np.loadtxt(filePath_pos_null, delimiter=",")


    # Calculate absolute position error
distance_norm = np.linalg.norm(pos_norm-pos_norm[0,:], axis=1)
distance_null_100 = np.linalg.norm(pos_null_100-pos_null_100[0,:], axis=1)
distance_null_400 = np.linalg.norm(pos_null_400-pos_null_400[0,:], axis=1)
distance_null_1000 = np.linalg.norm(pos_null_1000-pos_null_1000[0,:], axis=1)
distance_null_300 = np.linalg.norm(pos_null_300-pos_null_300[0,:], axis=1)
x = np.arange(0,len(distance_norm))/1000

fig = plt.figure(figsize=(12,8))
factor_mm = 1000
plt.plot(x, distance_norm*factor_mm, label = "normal")
plt.plot(x, distance_null_100*factor_mm, label = r"$K_n = 100$")
plt.plot(x, distance_null_400*factor_mm, label = r"$K_n = 400$")
plt.plot(np.arange(0,len(distance_null_1000))/1000, distance_null_1000*factor_mm, label = r"$K_n = 1000$")
# plt.plot(distance_null_300, label = r"Ausrichtung")

plt.legend()
plt.title("Absolute positional error")
plt.xlabel("Time [s]")
plt.ylabel("Error [mm]")
plt.show()
