#%%
import mujoco
import mujoco.viewer
import numpy as np
import matplotlib.pyplot as plt
import time

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
dt: float = 0.002

#%%
def main() -> None:
    assert mujoco.__version__ >= "3.1.0", "Please upgrade to mujoco 3.1.0 or later."

    # Load the model and data.
    filePath = "franka_fr3/scene.xml"
    filePath = "franka_fr3/scene_copy_robot.xml"
    # filePath = "kuka_iiwa_14/scene.xml"
    model = mujoco.MjModel.from_xml_path(filePath)
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
    q0 = model.key(key_name).qpos

    # Mocap body we will control with our mouse.
    mocap_name = "target"
    mocap_id = model.body(mocap_name).mocapid[0]

    # Stiffness matrix and stuff
    translation_stiffness = 300
    rotation_stiffness = 100
    K_p_values = np.array([translation_stiffness, translation_stiffness, translation_stiffness, rotation_stiffness, rotation_stiffness, rotation_stiffness])
    K_p = np.diag(K_p_values)
    K_d = np.diag(np.sqrt(K_p_values))
    factor_speed = 100

    # Pre-allocate numpy arrays.
    jac = np.zeros((6, model.nv))
    diag = damping * np.eye(6)
    error = np.zeros(6)
    error_pos = error[:3]
    error_ori = error[3:]
    cart_vel = np.zeros(6)
    cart_vel_pos = cart_vel[:3]
    cart_vel_ori = cart_vel[3:]

    site_quat = np.zeros(4)
    site_quat_conj = np.zeros(4)
    error_quat = np.zeros(4)
    M_inv = np.zeros((model.nv, model.nv))
    Mx = np.zeros((6, 6))

    # Desired position
    pos_init = np.array([0.52, 0.0, 0.2]) # FR3
    pos_init = np.array([0.389, 0.00, 0.355]) # Kuka
    quat_init = [0, 0.7071068, 0.7071068, 0] #[ 0.6830127, -0.6830127, -0.1830127, -0.1830127 ]
    quat_d = np.array([0.1227878, 0.6963642, 0.6963642, 0.1227878]) # FR3
    # quat_d = np.array([-0.1227878, 0.6963642, 0.6963642, -0.1227878])

    # Time variables
    time_acc = 2
    step_start = 0

    # Debugging
    debug:bool = True
    set_mocap_pos:bool = True
    t_debug = 0.5
    t_sample = t_debug

    with mujoco.viewer.launch_passive(
        model=model,
        data=data,
        show_left_ui=True,
        show_right_ui=True,
    ) as viewer:
        # Reset the simulation.
        mujoco.mj_resetDataKeyframe(model, data, key_id)

        # Reset the free camera.
        mujoco.mjv_defaultFreeCamera(model, viewer.cam)

        # Enable site frame visualization.
        viewer.opt.frame = mujoco.mjtFrame.mjFRAME_SITE

        # Time debugging
        t_init = time.time()
        time_current = 0
        while viewer.is_running():
            time_current = np.round(time.time()-t_init,3)
            time_ms = round(time_current*1000)
            step_start = time.time()

            if time_current>0.01 and set_mocap_pos==True:
                set_mocap_pos=False
                pos_pointer = data.site(site_id).xpos.copy()
                pos_init = np.array(pos_pointer)
                model.body("target").pos = pos_init
                model.body("target").quat = quat_d
                data.mocap_pos[mocap_id] = pos_init
                data.mocap_quat[mocap_id] = quat_d

            # Position error.
            error_pos[:] = data.mocap_pos[mocap_id] - data.site(site_id).xpos

            # Orientation error.
            mujoco.mju_mat2Quat(site_quat, data.site(site_id).xmat)
            mujoco.mju_negQuat(site_quat_conj, site_quat)
            mujoco.mju_mulQuat(error_quat, data.mocap_quat[mocap_id], site_quat_conj)
            mujoco.mju_quat2Vel(error_ori, error_quat, 1.0)

            # Jacobian.
            mujoco.mj_jacSite(model, data, jac[:3], jac[3:], site_id)

            # Current velocity
            vel = (jac[:,dof_ids] @ data.qvel[dof_ids])

            # Ensure smooth acceleration
            if (time.time()-t_init)<time_acc:
                factor_rot = (1 - np.cos(np.pi * (time.time()-t_init)/time_acc))/2
            else:
                factor_rot = 1
            factor_filter = np.ones(6)
            factor_filter[3:] = factor_rot

                # Impedance control
            F = K_p @ error - K_d @ vel
            tau = jac.T @ (F * factor_rot) # *factor_filter
            
                # Experiment with velocity based impedance control
            # Calculate the direction of the angular velocity
            factor_speed = 5
            cart_vel[:3] = error[:3]
            cart_vel[3:] = dt * factor_speed * error_ori/np.linalg.norm(error_ori)
            # Ensure smooth acceleration
            if (time.time()-t_init)<time_acc:
                factor_acc = (1 - np.cos(np.pi * (time.time()-t_init)/time_acc))/2
            else:
                factor_acc = 1
            h_c = K_p @ cart_vel - K_d @ (jac @ data.qvel[dof_ids])
            tau = jac.T @ h_c

                # Damped least squares control
            # tau = jac.T @ (np.linalg.solve(jac @ jac.T + diag, error))

            # Set the control signal and step the simulation.
            data.ctrl[actuator_ids] = tau[dof_ids]
            mujoco.mj_step(model, data)

                # Debug loop
            if (time_current>t_debug) and (debug==True):
                np.set_printoptions(suppress=True)
                print(f"Time: {np.round(time_current,1)} \ntau = {np.round(tau.T,3)} \nh_c: {np.transpose(np.round(h_c,3))} \n")
                t_debug += t_sample
            mujoco.mj_step(model, data)

            viewer.sync()
            time_until_next_step = dt - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)


if __name__ == "__main__":
    main()
