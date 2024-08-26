#%%
import glob
import os 
import time

import numpy as np

import mujoco
import mujoco.viewer

path = "franka_fr3/scene_knee.xml"
model = mujoco.MjModel.from_xml_path(path)
data = mujoco.MjData(model)

# Body where the point of applied force is located
body_name = "femur"
body_id = model.body(body_name).id
# Site of the point of applied force
site_name = "attachment_site2"
site_pos = model.site(site_name).pos
# Initial joint configuration saved as a keyframe in the XML file.
key_name = "start"
key_id = model.key(key_name).id

    # Get forces
folder_path = "/home/alexandergerard/Masterarbeit/Cmake_franka/build/data_output_knee/"
    # Wrench
list_of_files_wrench = glob.glob(folder_path + 'force_data*')
filePath_wrench = max(list_of_files_wrench, key=os.path.getctime)
wrench = np.loadtxt(filePath_wrench, delimiter=",")
force = wrench[:,:3] * [0, 0, 1]
torque = wrench[:,3:] * [0, 0, 0]

# mujoco.viewer.launch(model)
#%%
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
    time_between_frames = 0.0125*2
    time_next_frame = time_between_frames

    # visualize contact frames and forces, make body transparent
    options = mujoco.MjvOption()
    mujoco.mjv_defaultOption(options)
    options.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = True
    options.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = True
    options.flags[mujoco.mjtVisFlag.mjVIS_TRANSPARENT] = True

    with mujoco.viewer.launch_passive(
        model=model,
        data=data,
        show_left_ui=True,
        show_right_ui=True,
    ) as viewer:
        # Reset the simulation.
        # mujoco.mj_resetData(model, data)
        mujoco.mj_resetDataKeyframe(model, data, key_id)

        # Reset the free camera.
        mujoco.mjv_defaultFreeCamera(model, viewer.cam)

        # Enable site frame visualization.
        # viewer.opt.frame = mujoco.mjtFrame.mjFRAME_SITE
        # viewer.opt.frame = mujoco.mjtVisFlag.mjVIS_PERTOBJ
        # viewer.opt.flags = mujoco.mjtVisFlag.mjVIS_CONTACTPOINT
        
        t_init = time.time()
        while viewer.is_running():
            time_current = np.round(time.time()-t_init,3)

            model.tendon("LCL")._stiffness = data.ctrl[-1]
            mujoco.mj_step(model, data)

            # Set the control signal and step the simulation.
            if time_current>time_next_frame:
                time_ms = round(time_current*1000)
                if time_ms>np.shape(wrench)[0]:
                    # mujoco.mj_applyFT(model, data, force[-1, :], torque[-1, :], site_pos, body_id, data.qfrc_applied)
                    if end_of_file==False:
                        print("End of file reached, time:", time_ms)
                        end_of_file=True
                else: 
                    # mujoco.mj_applyFT(model, data, force[time_ms-1, :]*0.01, torque[time_ms-1, :], site_pos, body_id, data.qfrc_applied)
                    
                    print(f"Time: {time_current}, slider value: {data.ctrl[-1]}, tendon value: {model.tendon('LCL')._stiffness}") # , Current force: {np.round(force[time_ms-1, :],3)}, current Torque: {np.round(torque[time_ms-1, :],3)}
                mujoco.mj_step(model, data)
                viewer.sync()
                time_next_frame += time_between_frames

            # mujoco.mj_step(model, data)
            # viewer.sync()

if __name__ == "__main__":
    main()

