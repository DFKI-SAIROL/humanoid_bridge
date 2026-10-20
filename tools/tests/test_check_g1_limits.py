"""tools/check-g1-limits.py against small synthetic configs and URDFs."""

import subprocess
import sys
from pathlib import Path

import yaml

SCRIPT = Path(__file__).resolve().parent.parent / "check-g1-limits.py"

LIMITS = {
    "L_LEG_HIP_PITCH": (-2.5, 2.8, 32.0, 88.0),
    "L_LEG_HIP_ROLL": (-0.5, 2.9, 20.0, 88.0),
    "WAIST_YAW": (-2.6, 2.6, 32.0, 88.0),
}
URDF_NAMES = {
    "L_LEG_HIP_PITCH": "left_hip_pitch_joint",
    "L_LEG_HIP_ROLL": "left_hip_roll_joint",
    "WAIST_YAW": "waist_yaw_joint",
}


def write_config(tmp_path, overrides=None):
    params = {"joint_names": list(LIMITS)}
    for name, (lo, hi, dq, tau) in LIMITS.items():
        params[name] = {"q_min": lo, "q_max": hi, "dq_limit": dq, "tau_limit": tau}
    for (name, field), value in (overrides or {}).items():
        params[name][field] = value
    path = tmp_path / "config.yaml"
    path.write_text(yaml.safe_dump({"robot_bridge": {"ros__parameters": params}}))
    return path


def write_urdf(tmp_path, overrides=None, skip=(), velocity_effort=True):
    joints = []
    for name, (lo, hi, dq, tau) in LIMITS.items():
        urdf = URDF_NAMES[name]
        if urdf in skip:
            continue
        dq = 32.0 if name == "L_LEG_HIP_ROLL" else dq  # the original URDF value
        attrs = {"lower": lo, "upper": hi, "velocity": dq, "effort": tau}
        if not velocity_effort:
            attrs.update(velocity=0, effort=0)
        attrs.update((overrides or {}).get(urdf, {}))
        text = " ".join(f'{k}="{v}"' for k, v in attrs.items())
        joints.append(f'<joint name="{urdf}" type="revolute"><limit {text}/></joint>')
    path = tmp_path / "robot.urdf"
    path.write_text(f'<robot name="g1">{"".join(joints)}</robot>')
    return path


def run(*args):
    result = subprocess.run(
        [sys.executable, str(SCRIPT), *map(str, args)], capture_output=True, text=True
    )
    return result.returncode, result.stdout


def test_matching_urdf_passes(tmp_path):
    rc, out = run("--config", write_config(tmp_path), "--urdf", write_urdf(tmp_path))
    assert rc == 0
    assert out == f"ok   {tmp_path / 'robot.urdf'}: 3 joints, 12 fields checked\n"


def test_mismatch_fails_with_one_line_per_field(tmp_path):
    urdf = write_urdf(tmp_path, {"waist_yaw_joint": {"velocity": 30, "upper": 2.0}})
    rc, out = run("--config", write_config(tmp_path), "--urdf", urdf)
    assert rc == 1
    assert out.splitlines() == [
        f"FAIL {urdf}:",
        "  WAIST_YAW q_max: config 2.6 urdf 2",
        "  WAIST_YAW dq_limit: config 32 urdf 30",
    ]


def test_within_tolerance_passes(tmp_path):
    urdf = write_urdf(tmp_path, {"waist_yaw_joint": {"upper": 2.6005}})
    assert run("--config", write_config(tmp_path), "--urdf", urdf)[0] == 0


def test_zero_velocity_and_effort_are_skipped(tmp_path):
    rc, out = run(
        "--config", write_config(tmp_path), "--urdf", write_urdf(tmp_path, velocity_effort=False)
    )
    assert rc == 0
    assert "3 joints, 6 fields checked" in out


def test_intended_difference_passes_with_the_intended_value(tmp_path):
    # URDF says 32 for L_LEG_HIP_ROLL dq_limit, config holds the intended 20.
    assert run("--config", write_config(tmp_path), "--urdf", write_urdf(tmp_path))[0] == 0


def test_intended_difference_with_another_value_fails(tmp_path):
    config = write_config(tmp_path, {("L_LEG_HIP_ROLL", "dq_limit"): 32.0})
    rc, out = run("--config", config, "--urdf", write_urdf(tmp_path))
    assert rc == 1
    assert "  L_LEG_HIP_ROLL dq_limit: config 32 intended 20" in out.splitlines()


def test_missing_joint_fails(tmp_path):
    urdf = write_urdf(tmp_path, skip=("waist_yaw_joint",))
    rc, out = run("--config", write_config(tmp_path), "--urdf", urdf)
    assert rc == 1
    assert "  WAIST_YAW: no joint waist_yaw_joint in the urdf" in out.splitlines()


def test_one_failing_urdf_of_two_fails(tmp_path):
    good = write_urdf(tmp_path)
    bad = good.with_name("bad.urdf")
    bad.write_text(good.read_text().replace('effort="88.0"', 'effort="50"', 1))
    rc, out = run("--config", write_config(tmp_path), "--urdf", good, "--urdf", bad)
    assert rc == 1
    assert out.splitlines()[0].startswith("ok   ")
    assert out.splitlines()[1] == f"FAIL {bad}:"


def test_unreadable_file_is_a_usage_error(tmp_path):
    assert run("--config", write_config(tmp_path), "--urdf", tmp_path / "none.urdf")[0] == 2
    assert run("--config", tmp_path / "none.yaml", "--urdf", write_urdf(tmp_path))[0] == 2
    assert run("--config", write_config(tmp_path))[0] == 2
