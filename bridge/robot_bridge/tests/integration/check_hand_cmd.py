"""Asserts the recorded /dex3/*/cmd matches what the driver asked for. Exit 1 on failure.

Offline analysis: must not import rclpy, so it runs anywhere the recording does.
"""

import argparse
import json
import sys

from drive_hand_cmd import CONTROL_HZ, NUM_HAND_JOINTS, OVER_LIMIT, fingerprint

HAND_HZ = 100.0  # hand_control_dt: 0.01 in G1_config.yaml
RAMP_SECONDS = 2.0  # `duration` in G1_config.yaml
SIDES = ("left", "right")

# The hand watchdog releases `duration + 0.2 s` after the last command. An early
# release has to be separated from the end-of-run one by more than that; the rest
# is margin, not a claim about how big the gap should be.
MIN_EARLY_RELEASE_GAP = 0.3

# Frames checked while the NaN commands arrive: from the first one's effect to
# just past the last, still inside the last good command's watchdog.
NAN_WINDOW = (0.02, 0.05)

# hand_kp_max / hand_kd_max in G1_config.yaml.
KP_MAX = 10.0
KD_MAX = 2.0

# GEAR's baked-in hand gains, which the adapter reproduces.
WANT_KP = (2.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0)
WANT_KD = (0.5, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2)

# (q_min, q_max) per motor, per side, in DDS order, from the Dex3 URDFs. The two
# hands are mirrored, so a bound check that only looked at |q| would pass a sign
# flip -- which is the mistake these ranges exist to catch.
Q_RANGE = {
    "left": (
        (-1.04719755, 1.04719755),
        (-0.72431163, 0.920),
        (0.0, 1.74532925),
        (-1.57079632, 0.0),
        (-1.74532925, 0.0),
        (-1.57079632, 0.0),
        (-1.74532925, 0.0),
    ),
    "right": (
        (-1.04719755, 1.04719755),
        (-0.920, 0.72431163),
        (-1.74532925, 0.0),
        (0.0, 1.57079632),
        (0.0, 1.74532925),
        (0.0, 1.57079632),
        (0.0, 1.74532925),
    ),
}


def in_range(side, q, tol=1e-6):
    return all(Q_RANGE[side][i][0] - tol <= v <= Q_RANGE[side][i][1] + tol for i, v in enumerate(q))


def expected_mode(i):
    """The Dex3 bitfield the bridge must pack: id, status enable, timeout enable."""
    return (i & 0x0F) | (0x01 << 4) | (0x01 << 7)


def is_limp(sample):
    """A released frame: zero gains and zero feedforward is what makes it slack."""
    return all(v == 0.0 for v in sample["kp"]) and all(v == 0.0 for v in sample["kd"])


def close(a, b, tol=1e-3):
    return all(abs(x - y) < tol for x, y in zip(a, b))


def last_live(samples):
    """The last commanded frame. Every run ends limp -- the driver stops streaming
    a second before the fake dumps, so the watchdog releases both hands -- and
    reading samples[-1] would assert against that release instead."""
    live = [s for s in samples if not is_limp(s)]
    return live[-1] if live else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("samples")
    parser.add_argument(
        "--expect-nan-dropped", metavar="PHASES", help="drive_hand_cmd.py --phases output"
    )
    parser.add_argument("--expect-clamped", action="store_true")
    parser.add_argument(
        "--expect-left-released", action="store_true", help="fake_dex3 --stale-after"
    )
    parser.add_argument("--expect-left-hot", action="store_true", help="fake_dex3 --hot-after")
    args = parser.parse_args()

    with open(args.samples) as handle:
        recorded = json.load(handle)

    failures = []

    def check(name, ok, detail=""):
        print("{:<34} {}{}".format(name, "PASS" if ok else "FAIL", "  " + detail if detail else ""))
        if not ok:
            failures.append(name)

    for side in SIDES:
        samples = recorded.get(side, [])
        check(f"{side}: received /dex3/cmd", len(samples) > 100, f"{len(samples)} samples")
        if not samples:
            continue

        # A bridge that forgot to resize the unbounded MotorCmd[] sends nothing
        # useful; this is the check that catches it.
        check(f"{side}: seven motors per frame", all(s["n"] == NUM_HAND_JOINTS for s in samples))

        span = samples[-1]["t"] - samples[0]["t"]
        rate = len(samples) / span if span else 0.0
        check(
            f"{side}: publishes at ~{HAND_HZ:.0f} Hz",
            0.7 * HAND_HZ < rate < 1.3 * HAND_HZ,
            f"{rate:.0f} Hz",
        )

        last = last_live(samples) or samples[-1]
        check(
            f"{side}: bridge packed the mode bits",
            all(m == expected_mode(i) for i, m in enumerate(last["mode"])),
            "mode={}".format(last["mode"]),
        )

        # The ramp to hand_ready_q, which is zeros: sampled just after it ends.
        t0 = samples[0]["t"]
        ramped = min(samples, key=lambda s: abs((s["t"] - t0) - (RAMP_SECONDS + 0.3)))
        check(
            f"{side}: start_hand_control ramped",
            all(abs(v) < 1e-2 for v in ramped["q"]),
            "q[0]={:.4f} wanted 0".format(ramped["q"][0]),
        )

        check(
            f"{side}: gains inside the config range",
            all(0.0 <= v <= KP_MAX for v in last["kp"])
            and all(0.0 <= v <= KD_MAX for v in last["kd"]),
        )

        check(
            f"{side}: q inside every joint range",
            in_range(side, last["q"]),
            "q={}".format([round(v, 3) for v in last["q"]]),
        )

    # The load-bearing one: the two hands are separate topics carrying separate,
    # mirrored fingerprints. A crossed wire or a shared buffer shows here.
    settled = {}
    for side in SIDES:
        samples = recorded.get(side, [])
        if not samples:
            continue
        # Live frames only. A released hand republishes its last q, so the limp
        # tail matches the fingerprint too and would drag a zero-gain frame in
        # here -- which is what the gain check below would then read.
        want = fingerprint(side)
        matching = [
            s
            for s in samples
            if not is_limp(s) and all(abs(a - b) < 1e-3 for a, b in zip(s["q"], want))
        ]
        settled[side] = matching
        check(
            f"{side}: motor i lands on motor i",
            len(matching) > 0.2 * CONTROL_HZ,
            f"{len(matching)} frames matched {[round(v, 3) for v in want[:3]]}",
        )

    if len(settled) == 2:
        check(
            "the two hands are not crossed",
            not any(
                all(abs(a - b) < 1e-3 for a, b in zip(s["q"], fingerprint("right")))
                for s in settled["left"]
            ),
        )

    if settled.get("left"):
        check(
            "left: adapter gains reached the wire",
            all(abs(a - b) < 1e-3 for a, b in zip(settled["left"][-1]["kp"], WANT_KP))
            and all(abs(a - b) < 1e-3 for a, b in zip(settled["left"][-1]["kd"], WANT_KD)),
        )

    if args.expect_clamped:
        # Not the last live frame: drive_hand_cmd --check-estop streams after the over-limit one.
        live = [s for s in recorded["right"] if not is_limp(s)]
        clamped = [hi for _, hi in Q_RANGE["right"]]
        at_max = [s for s in live if close(s["q"], clamped)]
        check(
            "an out-of-range q was clamped, not dropped",
            bool(at_max) and all(in_range("right", s["q"]) for s in live),
            f"{len(at_max)} live frames at q_max={[round(v, 3) for v in clamped]}",
        )

    if args.expect_nan_dropped:
        right = recorded["right"]
        check("no NaN reached the wire", all(all(v == v for v in s["q"]) for s in right))
        # The over-limit command, clamped, is the last good one before the NaN commands.
        with open(args.expect_nan_dropped) as handle:
            start, end = json.load(handle)["nan"]
        clamped = [hi for _, hi in Q_RANGE["right"]]
        _, kp, kd = OVER_LIMIT
        frames = [
            s
            for s in right
            if start + NAN_WINDOW[0] <= recorded["t0"] + s["t"] <= end + NAN_WINDOW[1]
        ]
        kept = all(
            close(s["q"], clamped)
            and close(s["kp"], [kp] * NUM_HAND_JOINTS)
            and close(s["kd"], [kd] * NUM_HAND_JOINTS)
            for s in frames
        )
        check(
            "NaN commands left the last good target",
            len(frames) >= 0.05 * CONTROL_HZ and kept,
            "{} frames while they arrived, {}".format(
                len(frames), "all at the clamped target" if kept else "changed"
            ),
        )

    for wanted, cause in ((args.expect_left_released, "stale"), (args.expect_left_hot, "hot")):
        if not wanted:
            continue
        # Both hands end limp on every run, so "left went limp" proves nothing on
        # its own. What per-side release means is that the left stopped being
        # commanded well before the right did.
        left, right = recorded["left"], recorded["right"]
        left_live, right_live = last_live(left), last_live(right)
        check(
            f"a {cause} left hand released early",
            left_live is not None
            and right_live is not None
            and right_live["t"] - left_live["t"] > MIN_EARLY_RELEASE_GAP,
            "left last commanded at {}, right at {}".format(
                left_live["t"] if left_live else "never",
                right_live["t"] if right_live else "never",
            ),
        )
        check(
            "the right hand kept going",
            right_live is not None and any(v > 0.0 for v in right_live["kp"]),
            f"per-side release: a {cause} left hand must not drop the right",
        )
        check(
            "the released left hand went slack",
            all(is_limp(s) for s in left[-5:]),
        )
        if cause == "hot":
            # Its state still arrives, so it gets limp frames, not silence, while the right works.
            after = [s for s in left if left_live is not None and s["t"] > left_live["t"]]
            check(
                "the hot left hand stays limp, not silent",
                bool(after)
                and all(is_limp(s) for s in after)
                and right_live is not None
                and after[-1]["t"] >= right_live["t"],
                "{} limp frames, the last at {}".format(
                    len(after), after[-1]["t"] if after else "never"
                ),
            )

    if failures:
        print("\n{} check(s) failed: {}".format(len(failures), ", ".join(failures)))
        sys.exit(1)
    print("\nall checks passed")


if __name__ == "__main__":
    main()
