from pathlib import Path
import yaml


class Config:
    def __init__(self, file_path) -> None:
        file_path = Path(file_path).resolve()
        with open(file_path, "r") as f:
            config = yaml.safe_load(f)

        self.data = config
        self.policy_path = (file_path.parent / config["policy_path"]).resolve()
        self.control_dt = config["control_dt"]
        self.history_length = config["history_length"]
        self.obs_term_dims = config["obs_term_dims"]
        self.obs_joint_names = config["obs_joint_names"]
        self.action_joint_names = config["action_joint_names"]
        self.default_joint_pos = config["default_joint_pos"]
        self.joint_areas = config["joint_areas"]
        self.ang_vel_scale = config["ang_vel_scale"]
        self.dof_vel_scale = config["dof_vel_scale"]
        self.action_scale = config["action_scale"]
        self.obs_clip = config["obs_clip"]
        self.action_clip = config["action_clip"]
        self.action_delta_limit = config["action_delta_limit"]
        self.rl_gains = config["rl_gains"]
