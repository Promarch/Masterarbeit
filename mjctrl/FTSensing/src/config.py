# %% file to map the configs in yaml strucutre
from dataclasses import dataclass
from typing import List


@dataclass
class EnvSettings:
    model_type: str
    path_ext: str
    scene_path: str
    param_path: str
    param_file: str
    param_data_gen_file: str
    crit_speed: float
    max_episode_len: int
    ft_input_len: int
    freq: int
    apply_ft_body_id: int
    lam_reward: float
    data_path: str


@dataclass
class MeasureSettings:
    train_percentage: int
    shuffle: bool
    measure_path: str
    filter_oscillations: bool
    filter_apps: List[int]
    filter_tste: List[int]
    use_start_only: bool
    start_pos: int

@dataclass
class OverallSettings:
    env: EnvSettings
    measure_dataset: MeasureSettings



# %%
