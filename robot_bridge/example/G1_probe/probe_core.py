"""ROS-free part of the G1 probe client: build the observation, run the policy, compute targets.

Uses the deploy bundle's own tools/reference_obs.py (Deploy, PackPlayer, build_obs), so the
input layout has a single source. Tested offline against the bundle's parity recordings
(test_offline.py) before anything goes to the robot.
"""
import json
import sys
from pathlib import Path

import numpy as np
import onnxruntime as ort


def load_reference_obs(bundle_dir):
    tools = str(Path(bundle_dir) / "tools")
    if tools not in sys.path:
        sys.path.insert(0, tools)
    import reference_obs
    return reference_obs


class ProbeCore:
    def __init__(self, bundle_dir):
        self.bundle_dir = Path(bundle_dir)
        ref = load_reference_obs(self.bundle_dir)
        self._build_obs = ref.build_obs
        self.deploy = ref.Deploy(self.bundle_dir)
        self.player = ref.PackPlayer(self.deploy)
        # One thread, no spin-waiting: the default pool keeps every core busy between calls, which
        # on the robot starves the bridge's 1 kHz loop. The MLP takes ~0.1 ms on one core.
        opts = ort.SessionOptions()
        opts.intra_op_num_threads = 1
        opts.inter_op_num_threads = 1
        opts.add_session_config_entry("session.intra_op.allow_spinning", "0")
        opts.add_session_config_entry("session.inter_op.allow_spinning", "0")
        self.session = ort.InferenceSession(str(self.bundle_dir / "policy" / "policy.onnx"),
                                            sess_options=opts, providers=["CPUExecutionProvider"])
        self.input_name = self.session.get_inputs()[0].name
        self.manifest = json.loads((self.bundle_dir / "motions" / "manifest.json").read_text())
        self.motion_ids = [m["motion_id"] for m in self.manifest]
        self.trained = {m["motion_id"]: bool(m["trained_in_TI_B_inclHO"]) for m in self.manifest}
        self.joint_names = self.deploy.cfg["joints"]["bridge_motor_names"]
        self.reset()

    def reset(self):
        """Policy (re)start: previous action 0, reference = held stand row 0."""
        self.prev_action = np.zeros(29, dtype=np.float32)
        self.player.start_held()

    def start_motion(self, motion_id):
        self.player.start_motion(motion_id)

    @property
    def ref_name(self):
        return self.player.name

    @property
    def ref_k(self):
        return self.player.k

    def step(self, quat_wxyz, gyro, q, dq):
        """One 20 ms tick. Returns (obs, raw_action, target_q); advances the reference row."""
        obs = self._build_obs(self.deploy, self.player, quat_wxyz, gyro, q, dq, self.prev_action)
        raw = self.session.run(None, {self.input_name: obs[None, :]})[0][0].astype(np.float32)
        self.prev_action = raw
        self.player.advance()
        return obs, raw, self.deploy.target(raw)


def tilt_rad(quat_wxyz):
    """Angle between the pelvis z axis and world up."""
    w, x, y, z = (float(v) for v in quat_wxyz)
    cos_tilt = 1.0 - 2.0 * (x * x + y * y)
    return float(np.arccos(np.clip(cos_tilt, -1.0, 1.0)))
