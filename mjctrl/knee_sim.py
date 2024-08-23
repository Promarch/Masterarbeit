import glob
import os 
import time

import numpy as np

import mujoco
import mujoco.viewer

path = "franka_fr3/scene_knee.xml"
model = mujoco.MjModel.from_xml_path(path)
data = mujoco.MjData(model)

# Enable gravity compensation. Set to 0.0 to disable.
# Whether to enable gravity compensation.
gravity_compensation: bool = True
model.body_gravcomp[:] = float(gravity_compensation)

# Body where the point of applied force is located
body_name = "femur"
body_id = model.body(body_name).id
# Site of the point of applied force
site_name = "attachment_site2"
site_pos = model.site(site_name).pos

    # Get forces
folder_path = "/home/alexandergerard/Masterarbeit/Cmake_franka/build/data_output_knee/"
    # Wrench
list_of_files_wrench = glob.glob(folder_path + 'force_data*')
filePath_wrench = max(list_of_files_wrench, key=os.path.getctime)
wrench = np.loadtxt(filePath_wrench, delimiter=",")

# mujoco.viewer.launch(model)

def main() -> None:

    # Time variables
    time_max = (np.shape(wrench)[0]/1000)-0.005
    step_start = 0
    set_mocap_pos = True
    # Debug
    time_debug = 0.1
    time_sample = time_debug
    end_of_file = False
    # Framerate
    time_between_frames = 0.0000125*2
    time_next_frame = time_between_frames

    with mujoco.viewer.launch_passive(
        model=model,
        data=data,
        show_left_ui=True,
        show_right_ui=False,
    ) as viewer:
        # Reset the simulation.
        mujoco.mj_resetData(model, data)

        # Reset the free camera.
        mujoco.mjv_defaultFreeCamera(model, viewer.cam)

        # Enable site frame visualization.
        # viewer.opt.frame = mujoco.mjtFrame.mjFRAME_SITE
        
        t_init = time.time()
        while viewer.is_running():
            time_current = time.time()-t_init

            # Set the control signal and step the simulation.
            if time_current>time_next_frame:
                time_ms = round((time.time()-t_init)*1000)
                if time_ms>np.shape(wrench)[0]:
                    mujoco.mj_applyFT(model, data, wrench[-1, :3], wrench[-1, 3:], site_pos, body_id, data.qfrc_applied)
                    if end_of_file==False:
                        print("End of file reached, time:", time_ms)
                        end_of_file=True
                else: 
                    mujoco.mj_applyFT(model, data, wrench[time_ms-1, :3], wrench[time_ms-1, 3:], site_pos, body_id, data.qfrc_applied)
                    print(f"Current force: {wrench[time_ms-1, :3]}, current Torque: {wrench[time_ms-1, 3:]}")
                mujoco.mj_step(model, data)
                viewer.sync()
                time_next_frame += time_between_frames

if __name__ == "__main__":
    main()

