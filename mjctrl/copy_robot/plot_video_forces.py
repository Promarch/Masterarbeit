#%%
# Imports
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import matplotlib.animation as animation
import cv2
import glob 
import os 

def lowpassFilter(signal, F_cutoff=10):
    sample_rate = 1000
    signal_filtered = np.array(signal)
    for i in range(np.size(signal, 1)):
        data = signal[:,i]
        fft_sig = np.fft.fft(data)
        cutoff_filter = 1.0 * np.abs(np.fft.fftfreq(fft_sig.size, 1.0/sample_rate)) <= F_cutoff
        data_filter = np.real(np.fft.ifft(fft_sig * cutoff_filter))
        signal_filtered[:,i] = data_filter
    return signal_filtered

#%%
# Load video file
video_path = 'copy_robot/video.mp4'
cap = cv2.VideoCapture(video_path)

# Check if the video was successfully loaded
if not cap.isOpened():
    print("Error: Could not open video.")
    exit()

# Get video properties
fps = cap.get(cv2.CAP_PROP_FPS)
total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

# Extract frames
frame_count = 0
frames = []
while True:
    # Read a frame from the video
    ret, frame = cap.read()
    # If the frame was not read successfully, break the loop
    if not ret:
        break
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    frames.append(frame)
    frame_count += 1

# Release the video capture object
cap.release()
print(f"Extracted {frame_count} frames")

#%% 
    # Get Data
# Force data from sensor
# path =  "/home/alexandergerard/Masterarbeit/Cmake_franka/build/data_output/force_data_20240809_154101.txt"
folder_path = "/home/alexandergerard/Masterarbeit/Cmake_franka/build/data_ball_joint/"
list_of_files_wrench = glob.glob(folder_path + 'F_sensor_tot*')
folder_path = "/home/alexandergerard/Masterarbeit/Cmake_franka/build/data_output_knee/"
list_of_files_wrench = glob.glob(folder_path + 'F_sensor_tot*')
filePath_wrench = max(list_of_files_wrench, key=os.path.getctime)
wrench_orig = np.loadtxt(filePath_wrench, delimiter=",")
wrench = lowpassFilter(wrench_orig, F_cutoff=5)
    # Set up plot
fig, (ax_img, ax_force) = plt.subplots(2,1)
# Plot img
ax_img.axis('off')
ax_img.set_title('Video of Motion')
img = ax_img.imshow(frames[0])

# plot force
x = np.arange(len(wrench))/1000
ax_force.plot(x, wrench[:,3], "r", label=r"Filter $\tau_x$")
ax_force.plot(x, wrench[:,4], "g", label=r"Filter $\tau_y$")
ax_force.plot(x, wrench[:,5], "b", label=r"Filter $\tau_z$")

vert_line = ax_force.axvline(x=0, color = "r")

#pos_tau_x, = ax_force.plot(0,wrench[0,3], 'o')
ax_force.legend(loc = "upper right")

ax_force.set_title('Torque during Motion')
ax_force.set_xlabel('Time [s]')
ax_force.set_ylabel('Torque [Nm]')
ax_force.grid(True)

def update(frame_id):
    # update img
    img.set_data(frames[frame_id])
    # update force
    t = frame_id * len(wrench)/frame_count/1000
    vert_line.set_xdata(t)
    print(f"Current frame: {frame_id}, current time: {np.round(t,3)}")
    return (img, vert_line)

ani = animation.FuncAnimation(fig=fig, func=update, frames=frame_count, interval=25)
ani.save(filename="/home/alexandergerard/Masterarbeit/mjctrl/copy_robot/animation_robot.gif", writer="pillow")
# plt.show()

