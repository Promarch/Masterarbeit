#%%
import mujoco
import mujoco.renderer
import mujoco.viewer
import numpy as np
import time
import imageio
from PIL import Image
import glob
import os

# Integration timestep in seconds. This corresponds to the amount of time the joint
# velocities will be integrated for to obtain the desired joint positions.
integration_dt: float = 0.1

# Damping term for the pseudoinverse. This is used to prevent joint velocities from
# becoming too large when the Jacobian is close to singular.
damping: float = 1e-4

# Gains for the twist computation. These should be between 0 and 1. 0 means no
# movement, 1 means move the end-effector to the target in one integration step.
Kpos: float = 0.95
Kori: float = 0.95

# Whether to enable gravity compensation.
gravity_compensation: bool = True

# Simulation timestep in seconds.
dt: float = 0.001

# Nullspace P gain.
Kn = np.asarray([10.0, 10.0, 10.0, 10.0, 5.0, 5.0, 5.0])

# Maximum allowable joint velocity in rad/s.
max_angvel = 0.785

#%%
def main() -> None:
    assert mujoco.__version__ >= "3.1.0", "Please upgrade to mujoco 3.1.0 or later."

    # ------------------------------------------------------------------------
    # -----------             Set Up Mujoco Simulation            ------------
    # ------------------------------------------------------------------------

    # Load the model and data.
    model = mujoco.MjModel.from_xml_path("/home/alexandergerard/Masterarbeit/mjctrl/franka_fr3/scene_joint_actuator.xml")
    data = mujoco.MjData(model)

    # Enable gravity compensation. Set to 0.0 to disable.
    model.body_gravcomp[:] = float(gravity_compensation)
    model.opt.timestep = dt

    # End-effector site we wish to control.
    site_name = "attachment_site"
    site_id = model.site(site_name).id

    # Get the dof and actuator ids for the joints we wish to control. These are copied
    # from the XML file. Feel free to comment out some joints to see the effect on
    # the controller.
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

    # Initial joint configuration saved as a keyframe in the XML file.
    key_name = "home"
    key_id = model.key(key_name).id

    # Mocap body we will control with our mouse.
    mocap_name = "target"
    mocap_id = model.body(mocap_name).mocapid[0]

    # Mujoco options
    options = mujoco.MjvOption()
    options.frame = mujoco.mjtFrame.mjFRAME_SITE

    # ------------------------------------------------------------------------
    # -----------          Calculate desired positions            ------------
    # ------------------------------------------------------------------------

    folder_path = "/home/alexandergerard/Masterarbeit/Cmake_franka/build/data_output_cartesian/"
        # Joint positions        
    list_of_files_q = glob.glob(folder_path + 'joint_posi*')
    filePath_q = max(list_of_files_q, key=os.path.getctime)
    q = np.loadtxt(filePath_q, delimiter=",")
        # Desired rotation with time
    list_of_files_rot_time = glob.glob(folder_path + 'rotation_tim*')
    filePath_rot_time = max(list_of_files_rot_time, key=os.path.getctime)
    rot_time_orig = np.loadtxt(filePath_rot_time, delimiter=",")
    # rearrange the array so that it fits mujoco notation (normal/eigen notation is x,y,z,w; Mjc is w,x,y,z)
    rot_time = np.array(rot_time_orig)
    rot_time[:,0] = rot_time_orig[:,3]
    rot_time[:,1:4] = rot_time_orig[:,0:3]
    n_pos = 1

    desired_angle = np.deg2rad(-20)
    rotation_axis = np.array([1,1,0])

    # Pre-allocation
    rotation_init = np.zeros(4)
    pos_init = np.zeros(3)
    rotation_target_init = np.zeros(9)
    quat_target_init = np.zeros(4)
    desired_quat = np.zeros(4) 
    desired_quat_conj = np.zeros(4)

    # ------------------------------------------------------------------------
    # -----------          Debug and runtime variables            ------------
    # ------------------------------------------------------------------------

    # Time variables
    time_max = (np.shape(q)[0]/1000)-0.005
    step_start = 0
    set_mocap_pos = True

    # Debug
    time_debug = 0.1
    time_sample = time_debug
    end_of_file = False

    # Framerate
    time_between_frames = 0.0125*2
    time_next_frame = time_between_frames
    fps = 1/time_between_frames
    frames = []
    

    # ------------------------------------------------------------------------
    # -----------                                                 ------------
    # -----------                Start Simulation                 ------------
    # -----------                                                 ------------
    # ------------------------------------------------------------------------

    with mujoco.Renderer(model=model, height=224*2, width=320*2) as renderer:

        # Reset the simulation.
        mujoco.mj_resetDataKeyframe(model, data, key_id)
        
        # Set initial position of the robot
        data.qpos = q[0,:]

        t_init = time.time()
        while (time.time()-t_init)<time_max: # time_max
            time_current = time.time()-t_init

            if time_current>0.03 and set_mocap_pos:
                # This line only exists cause I dont know how to run this loop only once
                set_mocap_pos = False   

                    # Calculate desired rotation of mocap body
                # Get initial position
                pos_init = data.site(site_id).xpos
                pos_init = np.array(pos_init)
                model.body("target").pos = pos_init
                # Extract the rotation from the txt file
                model.body("target").quat = rot_time[0,:4]

            # Set new desired rotation according to text file
            if n_pos<len(rot_time):
                if time_current>rot_time[n_pos, -1]:
                    model.body("target").quat = rot_time[n_pos, :4]
                    print(f"Time now is {time_current}")
                    n_pos += 1
            
            # # Loop that gets called every time_debug seconds
            # step_current = round(data.time/model.opt.timestep)
            # if (time.time()-t_init)>time_debug:
            #     time_pc = round(1000*(time.time()-t_init))
            #     time_data = round(1000*data.time)
            #     print(f"Current step: {step_current}; Time wall: {time_pc}ms; Time data:{time_data}")
            #     time_debug += time_sample

            # Set the control signal and step the simulation.
            if time_current>time_next_frame:
                time_now = round((time.time()-t_init)*1000)
                if time_now>np.shape(q)[0]:
                    data.ctrl[actuator_ids] = q[-1, dof_ids]
                    if end_of_file==False:
                        print("End of file reached")
                        end_of_file=True
                else: 
                    data.ctrl[actuator_ids] = q[time_now-1, dof_ids]
                    renderer.update_scene(data, camera="closeup", scene_option=options) #
                    pixels = renderer.render()
                    image = Image.fromarray((pixels).astype(np.uint8))
                    frames.append(image)
                # Step simulation
                mujoco.mj_step(model, data)
                print(f"{time_next_frame}")
                time_next_frame += time_between_frames

        image_arrays = [np.array(img) for img in frames]
        with imageio.get_writer(f'copy_robot/video.mp4', mode='I', fps=fps) as writer:
            for image_array in image_arrays:
                writer.append_data(image_array)

        print(f"Time wall: {time_current}ms; Time data:{time_now}")
        
if __name__ == "__main__":
    main()
