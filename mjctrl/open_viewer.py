import mujoco
import mujoco.viewer
import numpy as np
import time

def openViewer(filepath, t_max:float = 0.0, dt:float = 0.002, debug:bool = False, t_debug:float = 0.1):
    # Model variables
    model = mujoco.MjModel.from_xml_path(filepath)
    data = mujoco.MjData(model)
    
    #Time and debug variables
    t_sample = t_debug

    # Initial joint configuration saved as a keyframe in the XML file.
    key_name = "home"
    key_id = model.key(key_name).id

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
        if debug==True:
            viewer.opt.frame = mujoco.mjtFrame.mjFRAME_SITE

        t_init = time.time()
        time_current = 0
        while viewer.is_running() and (time_current<t_max or t_max==0):
            time_current = np.round(time.time()-t_init,3)
            time_ms = round(time_current*1000)
            step_start = time.time()

                # Debug loop
            if (time_current>t_debug) and (debug==True):

                print(f"Time: {np.round(time_current,1)}")
                t_debug += t_sample
            mujoco.mj_step(model, data)

            viewer.sync()
            time_until_next_step = dt - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

filepath = "franka_fr3_sensor/scene.xml"
openViewer(filepath=filepath) # , debug=True, t_debug=1