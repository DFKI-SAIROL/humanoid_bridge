"""Fake Dex3 hands: publish /dex3/{left,right}/state and record /dex3/*/cmd.

Never publishes /dex3/*/cmd -- the bridge owns that topic. Tracks commands
perfectly, like fake_g1 does for the body.

Two switches exist so the guards can be exercised:
  --stale-after S   stop publishing left-hand state S seconds after the first
                    left-hand command arrives
  --hot-after S     report the left hand over temperature, same clock
  --silent SIDE     never publish that hand's state: a hand that is not attached

Both clocks start at the first `/dex3/left/cmd`, which is `start_hand_control`,
not at process start. The runner sleeps about 6 s before the driver arms the
hands, so a process-start clock silences the left hand before it has ever
tracked and the run reports "left last commanded at never".
"""

import argparse
import json
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from unitree_hg.msg import HandCmd, HandState, MotorState

NUM_MOTORS = 7
SIDES = ("left", "right")
STATE_PERIOD = 0.01  # 100 Hz; the bridge aborts on state older than 0.2 s
HOT = 99  # over hand_temperature_limit: 80.0 in G1_config.yaml


class FakeDex3(Node):
    def __init__(self, out_path, stale_after=None, hot_after=None, silent=None):
        super().__init__("fake_dex3")
        self.out_path = out_path
        self.stale_after = stale_after
        self.hot_after = hot_after
        self.silent = silent

        self.publishers_by_side = {}
        self.q = {}
        self.samples = {side: [] for side in SIDES}

        for side in SIDES:
            self.publishers_by_side[side] = self.create_publisher(
                HandState, f"/dex3/{side}/state", 10
            )
            self.q[side] = [0.0] * NUM_MOTORS
            self.create_subscription(HandCmd, f"/dex3/{side}/cmd", self._make_handler(side), 10)

        self.t0 = time.monotonic()
        # Set on the first /dex3/left/cmd. The --stale-after and --hot-after
        # clocks run from here; sample timestamps still run from t0.
        self.first_cmd_t = None
        self.create_timer(STATE_PERIOD, self._tick)

    def _make_handler(self, side):
        def handler(msg):
            if side == "left" and self.first_cmd_t is None:
                self.first_cmd_t = time.monotonic()
            # An unbounded MotorCmd[] on the wire: a bridge that forgot to resize
            # sends nothing here, which is the failure this records.
            n = min(len(msg.motor_cmd), NUM_MOTORS)
            self.q[side] = [msg.motor_cmd[i].q for i in range(n)]
            self.samples[side].append(
                {
                    "t": round(time.monotonic() - self.t0, 4),
                    "n": len(msg.motor_cmd),
                    "mode": [int(msg.motor_cmd[i].mode) for i in range(n)],
                    "q": [round(msg.motor_cmd[i].q, 6) for i in range(n)],
                    "dq": [round(msg.motor_cmd[i].dq, 6) for i in range(n)],
                    "tau": [round(msg.motor_cmd[i].tau, 6) for i in range(n)],
                    "kp": [round(msg.motor_cmd[i].kp, 4) for i in range(n)],
                    "kd": [round(msg.motor_cmd[i].kd, 4) for i in range(n)],
                }
            )

        return handler

    def _tick(self):
        # None until the bridge commands the left hand, so neither switch can
        # fire during the ~6 s the runner spends starting the fakes and bridge.
        since_cmd = None if self.first_cmd_t is None else time.monotonic() - self.first_cmd_t
        armed = since_cmd is not None

        for side in SIDES:
            if side == self.silent:
                continue
            if (side == "left" and self.stale_after is not None
                    and armed and since_cmd > self.stale_after):
                continue  # go silent: exercises the per-side freshness guard

            hot = (side == "left" and self.hot_after is not None
                   and armed and since_cmd > self.hot_after)

            state = HandState()
            state.motor_state = [MotorState() for _ in range(NUM_MOTORS)]
            for i in range(NUM_MOTORS):
                state.motor_state[i].q = self.q[side][i]
                state.motor_state[i].dq = 0.0
                state.motor_state[i].tau_est = 0.0
                state.motor_state[i].temperature = [HOT if hot else 30, 30]
            state.power_v = 24.0
            state.power_a = 0.5
            self.publishers_by_side[side].publish(state)

    def dump(self):
        with open(self.out_path, "w") as handle:
            # t0: time.monotonic() at sample t 0, to line samples up with a driver's phases.
            json.dump({"t0": self.t0, **self.samples}, handle)
        counts = ", ".join(f"{side}={len(self.samples[side])}" for side in SIDES)
        print(f"fake_dex3: wrote /dex3/*/cmd samples ({counts})", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("out", help="where to write the recorded /dex3/*/cmd samples")
    parser.add_argument("--stale-after", type=float, default=None)
    parser.add_argument("--hot-after", type=float, default=None)
    parser.add_argument("--silent", choices=SIDES, default=None)
    args = parser.parse_args()

    rclpy.init()
    node = FakeDex3(args.out, stale_after=args.stale_after, hot_after=args.hot_after,
                    silent=args.silent)
    signal.signal(signal.SIGTERM, lambda *_: (node.dump(), sys.exit(0)))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.dump()


if __name__ == "__main__":
    main()
