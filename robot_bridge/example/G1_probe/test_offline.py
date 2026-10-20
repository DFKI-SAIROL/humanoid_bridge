"""Offline check of the client's step path against the bundle's parity recordings (no ROS, no robot).

Feeds each recorded (quat, gyro, q, dq) through ProbeCore.step and compares the observation
and raw action with what the policy produced in simulation. Also checks the hand-over to the
held stand after the motion's last row.

    python test_offline.py --bundle <bundle dir>
"""
import argparse
from pathlib import Path

import numpy as np

from probe_core import ProbeCore

OBS_TOL = 1e-5
ACT_TOL = 1e-4


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bundle", required=True)
    args = parser.parse_args()

    core = ProbeCore(args.bundle)
    worst_obs, worst_act = 0.0, 0.0
    for path in sorted((Path(args.bundle) / "parity").glob("*.npz")):
        rec = np.load(path)
        core.reset()
        core.start_motion(path.stem)
        n = len(rec["obs"])
        obs_err, act_err = 0.0, 0.0
        for t in range(n):
            obs, raw, _ = core.step(rec["pelvis_quat_wxyz"][t], rec["gyro"][t], rec["q"][t], rec["dq"][t])
            obs_err = max(obs_err, float(np.abs(obs - rec["obs"][t]).max()))
            act_err = max(act_err, float(np.abs(raw - rec["action"][t]).max()))
            # The recording feeds back the sim's own action; keep the same history.
            core.prev_action = rec["action"][t].astype(np.float32)
        # After the last motion row the reference must switch to held stand row 0.
        row = core.player.row()
        assert core.ref_name == "held_stand" and np.array_equal(row, core.deploy.held_rows[0]), path.stem
        ok = obs_err < OBS_TOL and act_err < ACT_TOL
        print(f"{path.stem:36s} steps {n:4d}  obs {obs_err:.2e}  action {act_err:.2e}  {'OK' if ok else 'FAIL'}")
        worst_obs, worst_act = max(worst_obs, obs_err), max(worst_act, act_err)

    passed = worst_obs < OBS_TOL and worst_act < ACT_TOL
    print(f"worst obs {worst_obs:.2e}, worst action {worst_act:.2e} -> {'PASS' if passed else 'FAIL'}")
    raise SystemExit(0 if passed else 1)


if __name__ == "__main__":
    main()
