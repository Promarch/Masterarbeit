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
    model = mujoco.MjModel.from_xml_path("franka_fr3/scene.xml")
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
    translation_stiffness = 500
    rotation_stiffness = 100
    K_p_values = np.array([translation_stiffness, translation_stiffness, translation_stiffness, rotation_stiffness, rotation_stiffness, rotation_stiffness])
    K_p = np.diag(K_p_values)
    K_d = np.diag(np.sqrt(K_p_values))

    # Pre-allocate numpy arrays.
    jac = np.zeros((6, model.nv))
    diag = damping * np.eye(6)
    error = np.zeros(6)
    error_pos = error[:3]
    error_ori = error[3:]
    site_quat = np.zeros(4)
    site_quat_conj = np.zeros(4)
    error_quat = np.zeros(4)
    M_inv = np.zeros((model.nv, model.nv))
    Mx = np.zeros((6, 6))

    # Desired position
    pos_init = [-0.000124617, 0.55703, 0.06149]
    quat_init = [0, 0.7071068, 0.7071068, 0] #[ 0.6830127, -0.6830127, -0.1830127, -0.1830127 ]

    # Time variables
    time_acc = 5
    step_start = 0

    # Lists to collect force and torque data
    force_data_list = []
    torque_data_list = []
    time_list = []

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

        while viewer.is_running():
            if step_start==0:
                t_init = time.time()

            step_start = time.time()

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
            F = K_p @ error -  K_d @ vel
            tau = jac.T @ (F*factor_filter)

                # Damped least squares control
            # tau = jac.T @ (np.linalg.solve(jac @ jac.T + diag, error))

                # Operational space
            # Compute the task-space inertia matrix.
            mujoco.mj_solveM(model, data, M_inv, np.eye(model.nv))
            Mx_inv = jac @ M_inv @ jac.T
            if abs(np.linalg.det(Mx_inv)) >= 1e-2:
                Mx = np.linalg.inv(Mx_inv)
            else:
                Mx = np.linalg.pinv(Mx_inv, rcond=1e-2)
            # Compute generalized forces.
            # tau = jac.T @ Mx @ ((K_p @ error - K_d @ (jac[:,dof_ids] @ data.qvel[dof_ids]))*factor_filter)

            # Set the control signal and step the simulation.
            data.ctrl[actuator_ids] = tau[dof_ids]
            mujoco.mj_step(model, data)

            # Get sensor data
            torque_data_list.append(data.sensor("TorqueSensor").data.copy())
            force_data_list.append(data.sensor("ForceSensor").data.copy())
            time_list.append(data.time)

            viewer.sync()
            time_until_next_step = dt - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)
        force_data = np.vstack(force_data_list)
        torque_data = np.vstack(torque_data_list)
        time_data = np.array(time_list)

        # # Plot force and torque data
        # plt.figure(figsize=(12, 6))

        # plt.subplot(2, 1, 1)
        # plt.plot(time_data, force_data)
        # plt.title("Force at Attachment Site")
        # plt.xlabel("Time (s)")
        # plt.ylabel("Force (N)")
        # plt.grid(0.25)
        # plt.legend(['Fx', 'Fy', 'Fz'])

        # plt.subplot(2, 1, 2)
        # plt.plot(time_data, torque_data)
        # plt.title("Torque at Attachment Site")
        # plt.xlabel("Time (s)")
        # plt.ylabel("Torque (Nm)")
        # plt.grid(0.25)
        # plt.legend(['Tx', 'Ty', 'Tz'])

        # plt.tight_layout()
        # plt.show()
        
        print("Last line")


if __name__ == "__main__":
    main()
