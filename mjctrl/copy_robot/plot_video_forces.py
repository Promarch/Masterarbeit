#%%
# Imports
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import cv2
import glob 
import os 

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
# Plot img
fig, (ax_img, ax_force) = plt.subplots(2,1)
# Plot img
ax_img.axis('off')
ax_img.set_title('Video of Motion')

    # Plot force
# Get latest data
list_of_files_wrench = glob.glob('/home/alexandergerard/Masterarbeit/Cmake_franka/build/data_output/force_data*')
filePath_wrench = max(list_of_files_wrench, key=os.path.getctime)
wrench = np.loadtxt(filePath_wrench, delimiter=",")
# plot and annotate
ax_force.plot(wrench[:,3], label=r"$\tau_x$")
ax_force.plot(wrench[:,4], label=r"$\tau_y$")
ax_force.plot(wrench[:,5], label=r"$\tau_z$")
vert_line = ax_force.axvline(x=0, color = "r")

#pos_tau_x, = ax_force.plot(0,wrench[0,3], 'o')
ax_force.legend(loc = "upper right")

ax_force.set_title('Torque during Motion')
ax_force.set_xlabel('Time [s]')
ax_force.set_ylabel('Torque [Nm]')
ax_force.grid(True)

# Slider
ax_slider = plt.axes([0.25, .03, 0.50, 0.02])
t_max = np.shape(wrench)[0]-1
slider = Slider(ax_slider, 'T [ms]', 0, t_max, valinit=0)

def update(val):
    t = round(slider.val)
    frame_id = round((t/t_max) * len(frames))
    print("Frame id: ", frame_id)
    # update img
    ax_img.imshow(frames[frame_id])
    # update force
    vert_line.set_xdata(t)

    # redraw canvas while idle
    fig.canvas.draw_idle()

# call update function on slider value change
slider.on_changed(update)

plt.show()

#%%

TWOPI = 2*np.pi
wrench = np.loadtxt("force_data_20240725_140746.txt", delimiter=",")

fig, ax = plt.subplots()

t = np.arange(0.0, TWOPI, 0.001)
initial_amp = .5
s = np.sin(t)
y_marker = np.sin(initial_amp)
l, = plt.plot(t, s, lw=2)
meh, = plt.plot(initial_amp,y_marker, 'o')
vline = plt.axvline(x=initial_amp, color="r")
ax = plt.axis([0,TWOPI,-1,1])

axamp = plt.axes([0.25, .03, 0.50, 0.02])
# Slider
samp = Slider(axamp, 'Amp', 0, max(t), valinit=initial_amp)

def update(val):
    # amp is the current value of the slider
    amp = samp.val
    # update curve
    meh.set_xdata(amp)
    meh.set_ydata(np.sin(amp))
    vline.set_xdata(amp)
    plt.axvline(x=amp)
    # redraw canvas while idle
    fig.canvas.draw_idle()

# call update function on slider value change
samp.on_changed(update)

# plt.show()
