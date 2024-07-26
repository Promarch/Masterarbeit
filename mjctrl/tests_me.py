#%%
# Imports
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Slider
from matplotlib.widgets import Cursor
import cv2

#%%
# Load video file
video_path = 'video.mp4'
cap = cv2.VideoCapture(video_path)

# Check if the video was successfully loaded
if not cap.isOpened():
    print("Error: Could not open video.")
    exit()

# Get video properties
fps = cap.get(cv2.CAP_PROP_FPS)
total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

#%%
import os
frame_count = 0
frames = []
while True:
    # Read a frame from the video
    ret, frame = cap.read()

    # If the frame was not read successfully, break the loop
    if not ret:
        break

    # Save the frame as an image file
    frame_path = os.path.join("/home/alexandergerard/Masterarbeit/mjctrl/img/", f'frame_{frame_count:04d}.png')
    # cv2.imwrite(frame_path, frame)
    frames.append(frame)
    frame_count += 1

# Release the video capture object
cap.release()
print(f"Extracted {frame_count} frames")
#%% 



#%%
wrench = np.loadtxt("force_data_20240725_140746.txt", delimiter=",")

# Create the figure and the two subplots
fig, (ax_img, ax_force) = plt.subplots(2, 1, figsize=(10, 8))


# Placeholder for the video frame
ret, frame = cap.read()
if not ret:
    print("Error: Could not read the first frame.")
    exit()
# frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
im = ax_img.imshow(frames[0])

ax_img.axis('off')
ax_img.set_title('Video of Motion')

axamp = plt.axes([0.25, .03, 0.50, 0.02])

time = np.arange(0,np.shape(wrench)[0])
force = wrench[:,0]
# Plot the force data
ax_force.plot(time, force)
ax_force.set_title('Force during Motion')
ax_force.set_xlabel('Time (s)')
ax_force.set_ylabel('Force (N)')


plt.tight_layout()
plt.show()

cap.release()
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

plt.show()
