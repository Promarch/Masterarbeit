# %%
"""
Quick documentationn on the available Programs:
control_states = [0,1,2,3,5,14,19,23,24,25,26,27];
(0):  Stop
(1):  TrackingCalibration
(2):  FTSCalibration
(3):  ConnectSpecimen
(5):  Return Home
(14): Handmove
(19): Flexion
(23): Pivot Shift
(24): Varus-Valgus Loading
(25): Internal-External Loading
(26): Lachmann Loading
(27): Lachmann Loading v2
"""
import mujoco
import json
import mujoco.viewer
import pystache
import torch
import yaml
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib
from IPython.display import HTML
import numpy as np
import mujoco_viewer
from dm_control.rl.control import PhysicsError
import random
from torch.utils.data import Dataset
from config import EnvSettings, OverallSettings
from PIL import Image
import imageio
from serial_example import BotaSerialSensor


TIME_LABEL = 'time in s'
SCENE = '../scene.xml'
LEGEND_LOC = 'lower right'



def find_between(s: str, first: str, last: str):
    """helper for string preformatting"""
    try:
        start = s.index(first) + len(first)
        start_pos = s.index(first)
        end = s.index(last, start)
        return s[start:end].replace('"', ''), start_pos, end
    except ValueError:
        return "", "", ""


def recursive_loading(xml_path, path_ext='../', st_s='<include file=', end_s='/>', template_mode=False):
    """recursively load subfiles"""

    with open(xml_path, "r") as stream:
        xml_string = stream.read()

    xml_string = xml_string.replace("./", path_ext)
    extra_file, start_p, end_p = find_between(xml_string, st_s, end_s)

    if template_mode:
        filename = extra_file.split('/')[-1].split('.')[0]
        extra_file = f'{path_ext}{filename}_template.xml'

    if len(extra_file) > 0:
        extra_string = recursive_loading(
            extra_file, path_ext=path_ext)

        spos = extra_string.index('<mujoco model=')
        end = extra_string.index('>', spos)
        extra_string = extra_string[:spos] + extra_string[end:]
        extra_string = extra_string.replace('</mujoco>', '')

        xml_string = xml_string[:start_p] + extra_string + xml_string[end_p:]

    return xml_string


def display_video(frames, framerate=60, dpi=600):
    height, width, _ = frames[0].shape
    orig_backend = matplotlib.get_backend()
    # Switch to headless 'Agg' to inhibit figure rendering.
    matplotlib.use('Agg')
    fig, ax = plt.subplots(1, 1, figsize=(width / dpi, height / dpi), dpi=dpi)
    matplotlib.use(orig_backend)  # Switch back to the original backend.
    ax.set_axis_off()
    ax.set_aspect('equal')
    ax.set_position([0, 0, 1, 1])
    im = ax.imshow(frames[0])

    def update(frame):
        im.set_data(frame)
        return [im]

    interval = 1000 / framerate
    anim = animation.FuncAnimation(fig=fig, func=update, frames=frames,
                                   interval=interval, blit=True, repeat=False)
    return HTML(anim.to_html5_video())



def filter_array(arr, t=0.8, decimal_places=4):
    """Filter the array using the given threshold t and round the filtered values to the specified number of decimal places"""
    filtered_arr = []
    loc_value = arr[0]
    filtered_arr.append(round(loc_value, decimal_places))

    for value in arr[1:]:
        loc_value = t * loc_value + (1 - t) * value
        filtered_arr.append(round(loc_value, decimal_places))

    for i in range(len(arr)-1, 0, -1):
        loc_value = t * loc_value + (1 - t) * filtered_arr[i]
        filtered_arr[i] = round(loc_value, decimal_places)

    return filtered_arr


def get_error_knee(qpos, seg_data):
    err1 = ((seg_data['fq'] - qpos[:, 3:])**2).sum()
    err2 = ((seg_data['fp'] - qpos[:, :3])**2).sum()
    err = err1 + err2
    return err


class ParameterHandler:
    def __init__(self, parameters_file, param_path="../params"):
        self.parameters_file = f"{param_path}/{parameters_file}"

    def load_parameters(self, override=None):
        with open(self.parameters_file, "r") as stream:
            try:
                self.parameters = yaml.safe_load(stream)
                if override is not None:
                    self.parameters.update(override)
            except yaml.YAMLError as exc:
                print(exc)
                exit(0)


class ModelRenderer(ParameterHandler):
    def __init__(self, template_file, parameters_file, param_path="../params", path_ext="../"):
        super().__init__(parameters_file, param_path)
        self.template_file = template_file
        self.path_ext = path_ext
        self.render_template()

    def render_template(self, override=None):
        self.load_parameters(override=override)
        self.raw_model_str = recursive_loading(
            self.template_file, template_mode=True, path_ext=self.path_ext)
        self.update_model(self.parameters)

    def update_model(self, parameters):
        self.model = pystache.render(self.raw_model_str, parameters)
        self.physics = mujoco.MjModel.from_xml_string(self.model)


class BaseModel(ModelRenderer):
    """the base model to inherit from"""

    def __init__(self, cfg: EnvSettings, dataset=None):
        """load model and raw xml string"""
        super().__init__(cfg.scene_path, cfg.param_file, cfg.param_path, cfg.path_ext)

        self.crit_speed = cfg.crit_speed
        self.freq = cfg.freq
        self.dt = 1 / cfg.freq
        self.done = False
        self.epoch_count = 0
        self.t_step = 0
        self.mse = torch.nn.MSELoss()
        self.lam_reward = cfg.lam_reward
        self.mse_told = 0
        self.apply_ft_body_id = cfg.apply_ft_body_id
        self.data = mujoco.MjData(self.physics)
        self.pos_len = len(self.data.qpos)
        self.vel_len = len(self.data.qvel)

    def new_epoch(self):
        self.dset.reshuffle()
        self.iterset = iter(self.dset)
        self.episode_count = 0
        self.mse_told = 0

    def get_new_data(self):
        self.seg_data = next(self.iterset)
        self.episode_count += 1
        self.ft_ext = self.seg_data['ft']

        if self.episode_count >= self.dset.len - 1:
            self.new_epoch()
            self.epoch_count += 1

    def apply_force(self, ft_in):
        self.physics.data.xfrc_applied[self.apply_ft_body_id, :] = ft_in

    def step(self, ft_in=None):
        """apply one timestep in the simulation"""

        if ft_in is None:
            ft_in = self.ft_ext[self.t_step, :]

        tinit = self.physics.data.time
        self.t_step += 1

        if self.t_step >= self.max_sim_len:
            self.done = True

        while (self.physics.data.time - tinit) < self.dt:
            try:
                # integrator step for model simulation
                self.apply_force(ft_in)
                self.physics.step()
                cur_state_max = np.max(
                    np.abs(np.array(self.state())))

                # more stable: early braking method
                if cur_state_max > self.crit_speed:
                    self.done = True
                    return self.return_values(early_break=True)

            except PhysicsError:
                self.done = True
                return self.return_values(early_break=True)

        return self.return_values()

    def get_ft(self):
        return self.ft_ext[self.t_step, :]

    def simulate(self, ft_ext, freq=100, tend=1):
        """simulate for given time determined by length of ft_ext"""
        # init the trajectory
        sim_len = int(round(freq*tend))
        qpos = np.zeros((sim_len, self.pos_len))
        tpos = 0
        qpos[tpos, :] = self.physics.data.qpos

        self.apply_force(ft_ext[tpos, :])
        keep_simulating = True
        tinit = self.physics.data.time

        # simulate
        while tpos < sim_len - 1 and keep_simulating:
            # set the external force
            self.apply_force(ft_ext[tpos, :])

            # integrator step:
            self.physics.step()
            cur_state_max = np.max(
                np.abs(np.array(self.state())))

            # more stable: early braking method
            if cur_state_max > 3 * self.crit_speed:
                keep_simulating = False

            # sample by frequency
            if (self.physics.data.time - tinit) > self.dt:
                tpos += 1
                tinit = self.physics.data.time
                qpos[tpos, :] = self.physics.data.qpos

        return qpos

    def simulate_dq(self, ft_ext, freq=100, tend=1):
        """simulate for given time determined by length of ft_ext"""
        # init the trajectory
        sim_len = int(round(freq*tend))
        qpos = np.zeros((sim_len, self.vel_len))
        tpos = 0
        qpos[tpos, :] = self.physics.data.qvel

        self.apply_force(ft_ext[tpos, :])
        keep_simulating = True
        tinit = self.physics.data.time

        # simulate
        while tpos < sim_len - 1 and keep_simulating:
            # set the external force
            self.apply_force(ft_ext[tpos, :])

            # integrator step:
            self.physics.step()
            cur_state_max = np.max(
                np.abs(np.array(self.state())))

            # more stable: early braking method
            if cur_state_max > 3 * self.crit_speed:
                keep_simulating = False

            # sample by frequency
            if (self.physics.data.time - tinit) > self.dt:
                tpos += 1
                tinit = self.physics.data.time
                qpos[tpos, :] = self.physics.data.qvel

        return qpos

    def get_reward(self, early_break=False):
        """general reward function"""
        self.cur_pos = torch.tensor(self.pos())
        s_mea_t1 = self.get_measure_state()

        # the general distance between measurement and state
        mse_cur = -self.mse(s_mea_t1, self.cur_pos).detach().cpu().numpy()

        if early_break: # negative incentive
            reward = self.lam_reward * mse_cur - 100
        else: # positive incentive
            reward = self.lam_reward * mse_cur # + 0.1
        return reward

    def pos(self):
        return self.physics.data.qpos

    def vel(self):
        pos = self.physics.data.qpos
        pos_len = len(pos)
        state = self.physics.state()
        vel = state[pos_len:]
        return vel

    def state(self):
        return self.physics.state()

    def complete_state(self, device='cpu'):
        state = torch.tensor(self.state())
        ft_ext = torch.tensor(self.ft_ext[self.t_step, :])
        complete_state = torch.cat([state, ft_ext], dim=0).to(device).float()
        return complete_state

    def return_values(self, early_break=False):
        """
        -> state, reward, done (dset active)
        -> state, pos, done (dset inactive)
        """
        self.check_max_time()

        if self.dset_active:
            return self.complete_state(), self.get_reward(early_break), self.done

        return self.state(), self.pos(), self.done

    def check_max_time(self):
        """check if max time reached -> done is True"""
        if self.t_step >= (self.max_sim_len - 1):
            self.done = True

    def get_pixels(self):
        pixels = self.physics.render(height=1024, width=1024)
        return pixels

    def run(self, seg_data=None, parameters=None, freq=100):
        """interactively run the model"""

        bota_sensor_1 = BotaSerialSensor('/dev/ttyUSB0')

        # create the viewer object
        mjc_model = mujoco.MjModel.from_xml_string(self.model)
        data = mujoco.MjData(mjc_model)
        #viewer = mujoco_viewer.MujocoViewer(mjc_model, data)
        tpos = 0
        tend = 100
        sim_len = int(round(freq*tend))

        if seg_data is not None:
            tend, ft = self.reset(
                seg_data=seg_data, parameters=parameters, freq=freq)
            sim_len = int(round(freq*tend))
            mjc_model = mujoco.MjModel.from_xml_string(self.model)
            data = mujoco.MjData(mjc_model)
            viewer = mujoco_viewer.MujocoViewer(mjc_model, data)

        # simulate and render
        mjc_model = mujoco.MjModel.from_xml_path('scene.xml')
        mjc_data = mujoco.MjData(mjc_model)
        

        with mujoco.viewer.launch_passive(mjc_model, mjc_data) as viewer:
            while tpos < sim_len - 1:
                # apply external force if applied
                if seg_data is not None:
                    self.apply_force(ft[tpos, :])
                # simulate forward
                mujoco.mj_step(mjc_model, data)
                viewer.sync()

                if tpos < data.time * freq:
                    tpos += 1

        # close



    def perform_run(self, seg_data, freq=100, parameters=None, video=False, verbose=False, err=False):
        """perform a complete run by setting inital positions, applying ft and rec all data"""

        tend, ft = self.reset(
            seg_data=seg_data, parameters=parameters)

        # get the simulation state vector
        qpos = self.simulate(ft, freq=freq, tend=tend)
        loc_error = self.get_error(qpos, seg_data)

        if verbose:
            self.gernerate_video(qpos)
            if err:
                return self.plot_results(qpos, seg_data), loc_error
            return self.plot_results(qpos, seg_data)
        return qpos

    def gernerate_video(self, qpos):
        model = mujoco.MjModel.from_xml_string(self.model)
        data = mujoco.MjData(model)
        renderer = mujoco.Renderer(model, 1024, 1024)
        scene_option = mujoco.MjvOption()
        scene_option.flags[mujoco.mjtVisFlag.mjVIS_JOINT] = True

        frames = []

        # Loop through the timesteps, skipping every 10 steps
        for t in range(0, qpos.shape[0], 10):
            # Update the joint positions
            data.qpos[:qpos.shape[1]] = qpos[t, :]

            mujoco.mj_step(model, data)
            renderer.update_scene(
                data, scene_option=scene_option, camera="camera0")
            pixels = renderer.render()
            image = Image.fromarray((pixels).astype(np.uint8))
            frames.append(image)

        image_arrays = [np.array(img) for img in frames]
        self.epoch_count += 1
        with imageio.get_writer(f'./res/training_progress/{self.epoch_count}.mp4', mode='I', fps=20) as writer:
            for image_array in image_arrays:
                writer.append_data(image_array)

    def simulate_real_trajectory(self, seg_data, freq=100):
        frames = []
        len_traj = len(seg_data['fp'])
        for ind in range(len_traj):
            tend, ft = self.reset(seg_data=seg_data, t_sel=ind)
            frame = self.get_pixels()
            frames.append(frame)
        return display_video(frames, framerate=freq)


class KneeModel(BaseModel):
    """Contains the general knee model for simulation runs"""

    def __init__(self, cfg: EnvSettings, dataset=None):
        """load model and raw xml string"""
        super().__init__(cfg, dataset)

    def set_init_position(self, body_pos, body_quat, parameters, body_name='femur'):
        """update the body position"""
        find_str = f'<body name="{body_name}" '

        # get the position of body
        _, start_p, end_p = find_between(
            self.raw_model_str, find_str, '>')
        start_p = start_p + len(find_str)

        # also learn delta tibia-femur
        if body_name == 'tibia':
            body_pos[0] += float(parameters['tibia']['dx'])
            body_pos[1] += float(parameters['tibia']['dy'])
            body_pos[2] += float(parameters['tibia']['dz'])

            body_quat[0] += float(parameters['tibia']['dqw'])
            body_quat[1] += float(parameters['tibia']['dqx'])
            body_quat[2] += float(parameters['tibia']['dqy'])
            body_quat[3] += float(parameters['tibia']['dqz'])

        # prepare new string
        pos_str = f'pos="{body_pos[0]} {body_pos[1]} {body_pos[2]}"'
        quat_str = f'quat="{body_quat[0]} {body_quat[1]} {body_quat[2]} {body_quat[3]}"'
        replace_str = f'{pos_str} {quat_str}'

        # overwrite model str
        self.raw_model_str = self.raw_model_str[:start_p] + \
            replace_str + self.raw_model_str[end_p:]

    def get_measure_state(self, device='cpu'):
        pos = torch.tensor(self.seg_data['tp'][self.t_step, :])
        quat = torch.tensor(self.seg_data['tq'][self.t_step, :])
        s_t_mea = torch.zeros(self.pos_len, dtype=torch.float64).to(device)
        s_t_mea[:3] = pos
        s_t_mea[3:] = quat
        return s_t_mea

    def get_error(self, qpos, seg_data):
        err1 = ((seg_data['fq'] - qpos[:, 3:])**2).sum()
        err2 = ((seg_data['fp'] - qpos[:, :3])**2).sum()
        err = err1 + err2
        return err

    def init_pos_and_model(self, seg_data, parameters=None, t_sel=0):
        if parameters is None:
            parameters = self.parameters

        # set start pos
        fp0 = seg_data['fp'][t_sel, :]
        fq0 = seg_data['fq'][t_sel, :]
        self.set_init_position(fp0, fq0, parameters, body_name='femur')
        tp0 = seg_data['tp'][t_sel, :]
        tq0 = seg_data['tq'][t_sel, :]

        # apply updated model
        self.update_model(parameters)

    def reset_new(self, parameters, t_sel=0):
        self.get_new_data()
        self.done = False
        self.t_step = 0

        self.init_pos_and_model(self.seg_data, parameters, t_sel)
        tend = int(
            round((len(self.seg_data['fp'][:, 0]) * 100) / self.freq)) / 100
        self.tend = tend
        self.max_sim_len = len(self.seg_data['fp'][:, 0])

    def reset(self, seg_data, parameters=None, freq=100, t_sel=0):
        """initialize the model in the current state"""
        self.done = False
        self.t_step = 0

        self.init_pos_and_model(seg_data, parameters, t_sel)

        # get t_end
        tend = int(round((len(seg_data['fp'][:, 0]) * 100) / freq)) / 100
        self.tend = tend
        self.max_sim_len = len(seg_data['fp'][:, 0])

        # reset old mse
        self.mse_told = 0

        return tend, seg_data['ft']


# %%
if __name__ == '__main__':
    cfg = EnvSettings(
        model_type='knee',
        path_ext='./',
        scene_path='./scene.xml',
        param_path='./params',
        param_file='parameters_init.yaml',
        param_data_gen_file='',
        data_path='',
        crit_speed=20,
        max_episode_len=500,
        ft_input_len=6,
        freq=100,
        apply_ft_body_id=3,
        lam_reward=20,
    )

    t1 = 0000
    t2 = t1 + 5000
    model = KneeModel(cfg)
    model.run()

# %%