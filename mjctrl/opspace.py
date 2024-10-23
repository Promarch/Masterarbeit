import mujoco
import mujoco.viewer
import numpy as np
import time

# Cartesian impedance control gains.
impedance_pos = np.asarray([100.0, 100.0, 100.0])  # [N/m]
impedance_ori = np.asarray([50.0, 50.0, 50.0])  # [Nm/rad]

# Joint impedance control gains.
Kp_null = np.asarray([75.0, 75.0, 50.0, 50.0, 40.0, 25.0, 25.0])

# Damping ratio for both Cartesian and joint impedance control.
damping_ratio = 1.0

# Gains for the twist computation. These should be between 0 and 1. 0 means no
# movement, 1 means move the end-effector to the target in one integration step.
Kpos: float = 1

# Gain for the orientation component of the twist computation. This should be
# between 0 and 1. 0 means no movement, 1 means move the end-effector to the target
# orientation in one integration step.
Kori: float = 1

# Integration timestep in seconds.
integration_dt: float = 1.0

# Whether to enable gravity compensation.
gravity_compensation: bool = True

# Simulation timestep in seconds.
dt: float = 0.002


def main() -> None:
    assert mujoco.__version__ >= "3.1.0", "Please upgrade to mujoco 3.1.0 or later."

    # Load the model and data.
    robot = "kuka"
    robot = "FR3"
    if robot == "FR3":
        filePath = "franka_fr3/scene_copy_robot.xml"
        # Desired position
        pos_init = np.array([0.52, 0.0, 0.2]) # FR3
        quat_d = np.array([0.1227878, 0.6963642, 0.6963642, 0.1227878]) # FR3
    elif robot == "kuka":
        filePath = "kuka_iiwa_14/scene.xml"
        # Desired position
        pos_init = np.array([0.5, 0.00, 0.355]) # Kuka
        quat_d = np.array([-0.1227878, 0.6963642, 0.6963642, -0.1227878]) # Kuka
        # quat_d = np.array([0.1227878, 0.6963642, 0.6963642, 0.1227878]) # FR3

    model = mujoco.MjModel.from_xml_path(filePath)
    data = mujoco.MjData(model)

    model.body_gravcomp[:] = float(gravity_compensation)
    model.opt.timestep = dt

    # Compute damping and stiffness matrices.
    damping_pos = damping_ratio * 2 * np.sqrt(impedance_pos)
    damping_ori = damping_ratio * 2 * np.sqrt(impedance_ori)
    Kp = np.concatenate([impedance_pos, impedance_ori], axis=0)
    Kd = np.concatenate([damping_pos, damping_ori], axis=0)
    Kd_null = damping_ratio * 2 * np.sqrt(Kp_null)

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

    quat_init = [0, 0.7071068, 0.7071068, 0] 

    # Pre-allocate numpy arrays.
    jac = np.zeros((6, model.nv))
    twist = np.zeros(6)
    site_quat = np.zeros(4)
    site_quat_conj = np.zeros(4)
    error_quat = np.zeros(4)
    M_inv = np.zeros((model.nv, model.nv))
    Mx = np.zeros((6, 6))

    # Debugging
    debug:bool = True
    set_mocap_pos:bool = True
    t_debug = 0.05
    t_sample = t_debug
    time_acc = 5

    # Trying out velocity based impedance control
    cart_vel = np.zeros(6)

    with mujoco.viewer.launch_passive(
        model=model,
        data=data,
        show_left_ui=False,
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

            # Spatial velocity (aka twist).
            dx = data.mocap_pos[mocap_id] - data.site(site_id).xpos
            twist[:3] = Kpos * dx / integration_dt
            mujoco.mju_mat2Quat(site_quat, data.site(site_id).xmat)
            mujoco.mju_negQuat(site_quat_conj, site_quat)
            mujoco.mju_mulQuat(error_quat, data.mocap_quat[mocap_id], site_quat_conj)
            mujoco.mju_quat2Vel(twist[3:], error_quat, 1.0)
            twist[3:] *= Kori / integration_dt
            # twist[:3] = 0

            # Jacobian.
            mujoco.mj_jacSite(model, data, jac[:3], jac[3:], site_id)

            # Compute the task-space inertia matrix.
            mujoco.mj_solveM(model, data, M_inv, np.eye(model.nv))
            Mx_inv = jac @ M_inv @ jac.T
            if abs(np.linalg.det(Mx_inv)) >= 1e-2:
                Mx = np.linalg.inv(Mx_inv)
            else:
                Mx = np.linalg.pinv(Mx_inv, rcond=1e-2)

            # Compute generalized forces.
            tau = jac.T @ Mx @ (Kp * twist - Kd * (jac @ data.qvel[dof_ids]))

                # Experiment with velocity based impedance control
            # Calculate the direction of the angular velocity
            factor_speed = 60
            cart_vel[:3] = twist[:3]
            cart_vel[3:] = dt * factor_speed * twist[3:]/np.linalg.norm(twist[3:])
            # Ensure smooth acceleration
            if (time.time()-t_init)<time_acc:
                factor_acc = (1 - np.cos(np.pi * (time.time()-t_init)/time_acc))/2
            else:
                factor_acc = 1
            tau = jac.T @ Mx @ (Kp * cart_vel - Kd * (jac @ data.qvel[dof_ids]))


            # # Add joint task in nullspace.
            # Jbar = M_inv @ jac.T @ Mx
            # ddq = Kp_null * (q0 - data.qpos[dof_ids]) - Kd_null * data.qvel[dof_ids]
            # tau += (np.eye(model.nv) - jac.T @ Jbar.T) @ ddq

            # # Add gravity compensation.
            # if gravity_compensation:
            #     tau += data.qfrc_bias[dof_ids]

                # Debug loop
            if (time_current>t_debug) and (debug==True):
                np.set_printoptions(suppress=True)
                print(f"Time: {np.round(time_current,1)}, \ntau = {np.round(tau.T,3)} \nq = {np.round(data.qpos.copy(),3)}\n")
                t_debug += t_sample
            mujoco.mj_step(model, data)

            # Set the control signal and step the simulation.
            np.clip(tau, *model.actuator_ctrlrange.T, out=tau)
            data.ctrl[actuator_ids] = tau[actuator_ids]
            mujoco.mj_step(model, data)

            viewer.sync()
            time_until_next_step = dt - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)


if __name__ == "__main__":
    main()
