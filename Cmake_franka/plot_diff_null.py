import glob
import os

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt


    # Get data
# Position without nullspace
list_of_files_pos_norm = glob.glob('/home/alexandergerard/Masterarbeit/Cmake_franka/build/nullspace_test/position_data_20240809_170329.txt')
filePath_pos_norm = max(list_of_files_pos_norm, key=os.path.getctime)
pos_norm = np.loadtxt(filePath_pos_norm, delimiter=",")
# Position with nullspace 
list_of_files_pos_null = glob.glob('/home/alexandergerard/Masterarbeit/Cmake_franka/build/nullspace_test/position_data_20240809_170454.txt')
filePath_pos_null = max(list_of_files_pos_null, key=os.path.getctime)
pos_null_100 = np.loadtxt(filePath_pos_null, delimiter=",")
# Position with nullspace 
list_of_files_pos_null = glob.glob('/home/alexandergerard/Masterarbeit/Cmake_franka/build/nullspace_test/position_data_20240809_173340.txt')
filePath_pos_null = max(list_of_files_pos_null, key=os.path.getctime)
pos_null_400 = np.loadtxt(filePath_pos_null, delimiter=",")
# Position with nullspace 
list_of_files_pos_null = glob.glob('/home/alexandergerard/Masterarbeit/Cmake_franka/build/nullspace_test/position_data_20240809_173915.txt')
filePath_pos_null = max(list_of_files_pos_null, key=os.path.getctime)
pos_null_1000 = np.loadtxt(filePath_pos_null, delimiter=",")
# Position with nullspace 
list_of_files_pos_null = glob.glob('/home/alexandergerard/Masterarbeit/Cmake_franka/build/nullspace_test/position_data_20240809_180014.txt')
filePath_pos_null = max(list_of_files_pos_null, key=os.path.getctime)
pos_null_300 = np.loadtxt(filePath_pos_null, delimiter=",")


    # Calculate absolute position error
distance_norm = np.linalg.norm(pos_norm-pos_norm[0,:], axis=1)
distance_null_100 = np.linalg.norm(pos_null_100-pos_null_100[0,:], axis=1)
distance_null_400 = np.linalg.norm(pos_null_400-pos_null_400[0,:], axis=1)
distance_null_1000 = np.linalg.norm(pos_null_1000-pos_null_1000[0,:], axis=1)
distance_null_300 = np.linalg.norm(pos_null_300-pos_null_300[0,:], axis=1)

fig = plt.figure(figsize=(12,8))
plt.plot(distance_norm, label = "normal")
plt.plot(distance_null_100, label = r"$K_n = 100$")
plt.plot(distance_null_400, label = r"$K_n = 400$")
plt.plot(distance_null_1000, label = r"$K_n = 1000$")
plt.plot(distance_null_300, label = r"Ausrichtung")

plt.legend()
plt.show()