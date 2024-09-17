#%%
import glob
import os 
import time

import numpy as np

import mujoco
import mujoco.viewer

    # Import model
path = "franka_fr3_sensor/scene.xml"
model = mujoco.MjModel.from_xml_path(path)
data = mujoco.MjData(model)

# Simulation timestep in seconds.
dt: float = 0.002

#     # Get forces
# folder_path = "/home/alexandergerard/Masterarbeit/Cmake_franka/build/data_output_knee/"
# # Wrench
# list_of_files_wrench = glob.glob(folder_path + 'force_data*')
# filePath_wrench = max(list_of_files_wrench, key=os.path.getctime)
# wrench = np.loadtxt(filePath_wrench, delimiter=",")
# force = wrench[:,:3] * [0, 0, 1]
# torque = wrench[:,3:] * [0, 0, 0]

# mujoco.viewer.launch(model)
#%%
def main() -> None:
    
        # Set Simulation parameters
    # Enable gravity compensation. Set to 0.0 to disable.
    gravity_comp = True
    model.body_gravcomp[:] = float(gravity_comp)
    model.opt.timestep = dt

        # Get id's of different bodies, sites and tendons

    # Body where the point of applied force is located
    body_name = "plate_RobotSensor"
    body_id = model.body(body_name).id

    # Site of the point of applied force
    site_name = "sensor_site"
    site_id = model.site(site_name).id
    site_pos = model.site(site_name).pos

    # Initial joint configuration saved as a keyframe in the XML file.
    if gravity_comp==True:
        key_name = "actr_motor"
    else:
        key_name = "actr_pos"
    key_id = model.key(key_name).id
    # Tendons
    name_tendons = ["x_plus", "x_minus", "y_plus", "x_minus"]
    # Force actuators
    joint_names = [
        "joint1",
        "joint2",
        "joint3",
        "joint4",
        "joint5",
        "joint6",
        "joint7",
    ]
    dof_ids = np.array([model.joint(name).id for name in joint_names])
    actuator_ids = np.array([model.actuator(name).id for name in joint_names])
    # general slider
    stiffness_slider = "tendon stiffness"
    stiffness_slider_id = model.actuator(stiffness_slider).id
    # debug_slider = "debug"
    # debug_slider_id = model.actuator(debug_slider).id

    #Time and debug variables
    t_max = 0
    debug:bool = True
    t_debug:float = 1
    t_sample = t_debug

    with mujoco.viewer.launch_passive(
        model=model,
        data=data,
        show_left_ui=True,
        show_right_ui=True,
    ) as viewer:
        # mujoco.mj_resetData(model, data)
        mujoco.mj_resetDataKeyframe(model, data, key_id)

        # Reset the free camera.
        mujoco.mjv_defaultFreeCamera(model, viewer.cam)

        # Enable site frame visualization.
        # if debug==True:
            # viewer.opt.frame = mujoco.mjtFrame.mjFRAME_SITE

        t_init = time.time()
        time_current = 0
        while viewer.is_running() and (time_current<t_max or t_max==0):
            time_current = np.round(time.time()-t_init,3)
            time_ms = round(time_current*1000)
            step_start = time.time()

            # Set the stiffness of the tendon to the slider value
            for i, tendon in enumerate(name_tendons):
                model.tendon(tendon)._stiffness = data.ctrl[stiffness_slider_id]

                # Debug loop
            if (time_current>t_debug) and (debug==True):
                print(f"Time: {np.round(time_current,1)}, L_tendons: {np.round(data.ten_length,3)}")
                t_debug += t_sample
            mujoco.mj_step(model, data)

            viewer.sync()
            time_until_next_step = dt - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

if __name__ == "__main__":
    main()

