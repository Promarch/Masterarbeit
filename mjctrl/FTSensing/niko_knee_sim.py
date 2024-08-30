#%%
import glob
import os 
import time

import numpy as np

import mujoco
import mujoco.viewer

    # Import model
path = "FTSensing/scene.xml"
model = mujoco.MjModel.from_xml_path(path)
data = mujoco.MjData(model)

    # Set Simulation parameters
# Enable gravity compensation. Set to 0.0 to disable.
model.body_gravcomp[:] = float(False)
# Simulation timestep in seconds.
dt: float = 0.002

    # Get id's of different bodies, sites and tendons
# Body where the point of applied force is located
body_name = "femur"
body_id = model.body(body_name).id
# Site of the point of applied force
site_name = "attachment_site2"
site_id = model.site(site_name).id
site_pos = model.site(site_name).pos
# Initial joint configuration saved as a keyframe in the XML file.
# key_name = "start"
# key_id = model.key(key_name).id
# Tendons
name_tendons = ["ACL", "PCL", "sMCL", "LCL"]
# Force actuators
actuator_name = ["F_x", "F_y", "F_z", "tau_x", "tau_y", "tau_z"]
actuator_ids = np.array([model.actuator(name).id for name in actuator_name])
# general slider
stiffness_slider = "tendon stiffness"
stiffness_slider_id = model.actuator(stiffness_slider).id

    # Get forces
folder_path = "/home/alexandergerard/Masterarbeit/Cmake_franka/build/data_output_knee/"
# Wrench
list_of_files_wrench = glob.glob(folder_path + 'force_data*')
filePath_wrench = max(list_of_files_wrench, key=os.path.getctime)
wrench = np.loadtxt(filePath_wrench, delimiter=",")
force = -wrench[:,:3] * [0, 0, 1]
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
    time_between_frames = 0.025 * 4
    time_next_frame = time_between_frames

    # Pre allocation
    sensor_data = np.zeros(4)

    with mujoco.viewer.launch_passive(
        model=model,
        data=data,
        show_left_ui=True,
        show_right_ui=True,
    ) as viewer:
        # Reset the simulation.
        mujoco.mj_resetData(model, data)
        # mujoco.mj_resetDataKeyframe(model, data, key_id)

        # Reset the free camera.
        mujoco.mjv_defaultFreeCamera(model, viewer.cam)
        
        t_init = time.time()
        while viewer.is_running():
            time_current = np.round(time.time()-t_init,3)
            time_ms = round(time_current*1000)
            step_start = time.time()

            # Set the stiffness of the tendon to the slider value
            # for i, tendon in enumerate(name_tendons):
            #     model.tendon(tendon)._stiffness = data.ctrl[stiffness_slider_id]
                
            # # Current debug
            # model.tendon("ACL")._lengthspring = data.ctrl[model.actuator("L_ACL").id]
            # model.tendon("PCL")._lengthspring = data.ctrl[model.actuator("L_PCL").id]
            # model.tendon("sMCL")._lengthspring = data.ctrl[model.actuator("L_sMCL").id]
            # model.tendon("LCL")._lengthspring = data.ctrl[model.actuator("L_LCL").id]

                # Debug loop
            if time_current>time_debug:
                if time_ms>np.shape(wrench)[0]:
                    mujoco.mj_applyFT(model, data, force[-1, :], torque[-1, :], site_pos, body_id, data.qfrc_applied)
                    if end_of_file==False:
                        print("End of file reached, time:", time_ms)
                        end_of_file=True
                else:
                    mujoco.mj_applyFT(model, data, force[time_ms-1, :], torque[time_ms-1, :], site_pos, body_id, data.qfrc_applied)
                    print(f"Time: {np.round(time_current,1)}, Force: {np.round(force[time_ms-1, :],3)}, Torque: {np.round(torque[time_ms-1, :],3)}")

                time_debug += time_sample

            # Set the control signal and step the simulation.
            # for actuator in name_actuator:
            #     data.ctrl[actuator] = wrench[time_ms, actuator]
            mujoco.mj_step(model, data)

            viewer.sync()
            time_until_next_step = dt - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

if __name__ == "__main__":
    main()

