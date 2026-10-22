"""Drives the e-stop of a running bridge against fake_g1.py and checks /lowcmd live.

Exit 1 on failure. run_fake_estop_test.sh starts the fake, then this driver, then the bridge.
"""

import argparse
import bisect
import sys
import time

import rclpy
import yaml
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32, String, UInt16
from std_srvs.srv import Trigger
from unitree_hg.msg import LowCmd

from robot_bridge.cmd_client import RobotCmdClient, pack_robot_cmd

NUM_MOTORS = 29
KEY_R1, KEY_L1, KEY_R2 = 1 << 0, 1 << 1, 1 << 4
BRIDGE_HZ = 1000.0  # control_dt: 0.001
RAMP_SECONDS = 2.5  # `duration` 2.0 plus margin
GRACE = 0.05  # /lowcmd and /bridge_state are separate topics: order at a change is not kept
HOLDING = "driver: holding /lowcmd"


def load_params(path):
    with open(path) as handle:
        params = yaml.safe_load(handle)["robot_bridge"]["ros__parameters"]
    names = params["joint_names"]
    return {
        "kp": [float(params[n]["kp"]) for n in names],
        "kd": [float(params[n]["kd"]) for n in names],
        "damp_kd": float(params["estop"]["damp_kd"]),
        "ready_q": [float(v) for v in params["ready_q"]],
        "dq_limit": [float(params[n]["dq_limit"]) for n in names],
        "parallel": [bool(params[n].get("if_parallel_joint", False)) for n in names],
    }


class Driver:
    def __init__(self, node, params, bridge_log):
        self.node = node
        self.params = params
        self.bridge_log = bridge_log
        self.client = RobotCmdClient(node, num_dof=NUM_MOTORS, control_frequency=50.0)
        self.ready = node.create_client(Trigger, "/ready_position_control")
        self.zero = node.create_client(Trigger, "/zero_position_control")
        self.keys = node.create_publisher(UInt16, "/fake_g1/keys", 10)
        self.roll = node.create_publisher(Float32, "/fake_g1/roll", 10)
        self.remote = node.create_publisher(Bool, "/fake_g1/remote", 10)
        self.pause = node.create_publisher(Bool, "/fake_g1/pause", 10)
        self.nan_q = node.create_publisher(Bool, "/fake_g1/nan_q", 10)
        self.dq = node.create_publisher(Float32, "/fake_g1/dq", 10)
        self.nan_imu = node.create_publisher(Bool, "/fake_g1/nan_imu", 10)
        # Up before the bridge starts: the bridge waits until it is gone.
        self.blocker = node.create_publisher(LowCmd, "/lowcmd", 10)
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        node.create_subscription(String, "/bridge_state", self._on_state, latched)
        node.create_subscription(LowCmd, "/lowcmd", self._on_cmd, 10)
        self.state = None
        self.last = None
        self.count = 0
        self.failures = []
        self.changes = []  # (t, state)
        self.frames = []  # (t, kp, q, dq, tau)

    def _on_state(self, msg):
        self.state = msg.data
        self.changes.append((time.monotonic(), msg.data))

    def _on_cmd(self, msg):
        self.last = msg
        self.count += 1
        motors = list(msg.motor_cmd)[:NUM_MOTORS]
        self.frames.append(
            (
                time.monotonic(),
                [m.kp for m in motors],
                [m.q for m in motors],
                [m.dq for m in motors],
                [m.tau for m in motors],
            )
        )

    # --- helpers ---------------------------------------------------------

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.005)

    def wait_inputs(self, timeout=5.0):
        # A press published before discovery matched is lost (Fast DDS).
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            inputs = (self.keys, self.roll, self.remote, self.pause)
            inputs += (self.nan_q, self.dq, self.nan_imu)
            if all(p.get_subscription_count() for p in inputs):
                return True
            rclpy.spin_once(self.node, timeout_sec=0.01)
        return False

    def wait_state(self, wanted, timeout=2.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if self.state == wanted:
                return True
            rclpy.spin_once(self.node, timeout_sec=0.005)
        return False

    def drain(self, quiet=0.1, timeout=2.0):
        """Spin until no /lowcmd frame for `quiet` s; False if frames go on for `timeout` s."""
        end = time.monotonic() + timeout
        seen, last = self.count, time.monotonic()
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.005)
            now = time.monotonic()
            if self.count != seen:
                seen, last = self.count, now
            elif now - last >= quiet:
                return True
        return False

    def wait_log(self, text, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            with open(self.bridge_log) as handle:
                if text in handle.read():
                    return True
            self.spin(0.1)
        return False

    def check(self, name, ok, detail=""):
        print(
            "{:<44} {}{}".format(name, "PASS" if ok else "FAIL", "  " + detail if detail else ""),
            flush=True,
        )
        if not ok:
            self.failures.append(name)

    def check_state(self, name, wanted):
        self.check(name, self.wait_state(wanted), f"state '{self.state}'")

    def motors(self):
        return list(self.last.motor_cmd)[:NUM_MOTORS] if self.last else []

    def frame(self, kp, kd, q=None):
        motors = self.motors()
        if not motors:
            return False, "no /lowcmd"
        ok = all(abs(m.kp - k) < 1e-3 for m, k in zip(motors, kp))
        ok = ok and all(abs(m.kd - k) < 1e-3 for m, k in zip(motors, kd))
        if q is not None:
            ok = ok and all(abs(m.q - q) < 1e-2 for m in motors)
        return ok, f"q[0]={motors[0].q:.4f} kp[0]={motors[0].kp:.1f} kd[0]={motors[0].kd:.1f}"

    def at(self, q):
        motors = self.motors()
        if not motors:
            return False, "no /lowcmd"
        worst = max(abs(m.q - w) for m, w in zip(motors, q))
        return worst < 1e-2, f"worst |q - wanted| {worst:.4f}"

    def feedforward(self, dq, tau, frames):
        """Every frame sends this dq and tau. Parallel joints: dq only, the bridge converts their torque."""
        parallel = self.params["parallel"]
        bad = sum(
            any(
                abs(f_dq[i] - dq) > 1e-6 or (not parallel[i] and abs(f_tau[i] - tau) > 1e-6)
                for i in range(len(parallel))
            )
            for _, _, _, f_dq, f_tau in frames
        )
        return bool(frames) and bad == 0, f"{bad} of {len(frames)} frames off"

    def rate(self, seconds=1.0):
        start = self.count
        self.spin(seconds)
        return (self.count - start) / seconds

    def press(self, keys, seconds=0.2):
        self.keys.publish(UInt16(data=keys))
        self.spin(seconds)

    def send(self, q, hold_position, duration=0.1, dq=0.0, tau=0.0):
        n = NUM_MOTORS
        self.client.send_cmd(
            q_target_pos=[q] * n,
            q_target_vel=[dq] * n,
            target_tau=[tau] * n,
            target_kp=self.params["kp"],
            target_kd=self.params["kd"],
            duration=duration,
            hold_position=hold_position,
        )

    def start(self, q):
        return self.client.start_control(default_position=[q] * NUM_MOTORS, timeout_sec=30.0)

    # --- the run ---------------------------------------------------------

    def run(self):
        kp, kd = self.params["kp"], self.params["kd"]
        damp_kp, damp_kd = [0.0] * NUM_MOTORS, [self.params["damp_kd"]] * NUM_MOTORS

        # A call stored while the bridge waits would arm once it spins: no arming service yet.
        up = self.client.estop_reset_client.wait_for_service(timeout_sec=30.0)
        self.check("bridge up, waiting on our /lowcmd", up)
        if not up:
            return
        seen = self.wait_log("Detected 1 publishers on /lowcmd", timeout=5.0)
        self.check(
            "bridge saw our /lowcmd publisher",
            seen,
            "" if seen else "not in the bridge log: discovery missed it, so the bridge never waited",
        )
        start = self.client.start_control_client
        self.check("no /start_control while the bridge waits", not start.wait_for_service(timeout_sec=1.0))
        self.node.destroy_publisher(self.blocker)
        self.check("/start_control once /lowcmd is free", start.wait_for_service(timeout_sec=10.0))

        self.check("fake_g1 inputs matched", self.wait_inputs())
        self.check_state("starts DISARMED", "DISARMED")

        # The bridge reads its parameters once; g1_web_commander reads the live ones.
        get = self.node.create_client(GetParameters, "/robot_bridge/get_parameters")
        put = self.node.create_client(SetParameters, "/robot_bridge/set_parameters")
        names = self.client._call(get, GetParameters.Request(names=["joint_names"]), 10.0)
        key = names.values[0].string_array_value[0] + ".kp"
        value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=999.0)
        request = SetParameters.Request(parameters=[Parameter(name=key, value=value)])
        result = self.client._call(put, request, 10.0).results[0]
        self.check(
            f"param set {key} refused",
            not result.successful and "read-only" in result.reason,
            result.reason,
        )
        now = self.client._call(get, GetParameters.Request(names=[key]), 10.0).values[0]
        kept = now.double_value == self.params["kp"][0]
        self.check(f"{key} unchanged", kept, str(now.double_value))

        # Arming needs the remote (estop.require_remote); fake_g1 sends its header by default.
        count = self.count
        self.remote.publish(Bool(data=False))
        self.spin(1.0)
        reply = self.start(0.123)
        self.check(
            "start_control refused without the remote",
            not reply.success and "remote" in reply.message,
            reply.message,
        )
        self.remote.publish(Bool(data=True))
        self.spin(0.3)

        # Gate 9: a wrong length moves nothing.
        reply = self.client.start_control(default_position=[0.1] * 14, timeout_sec=30.0)
        self.check("start_control with 14 values refused", not reply.success, reply.message)
        reply = self.client.start_control(default_position=[5.0] * NUM_MOTORS, timeout_sec=30.0)
        self.check("start_control out of range refused", not reply.success, reply.message)
        # Gains were dropped silently: the bridge arms with its config gains.
        request = self.client.start_control_client.srv_type.Request()
        request.default_position = [0.123] * NUM_MOTORS
        request.default_kp = [50.0] * NUM_MOTORS
        reply = self.client._call(self.client.start_control_client, request, 30.0)
        self.check(
            "start_control with default_kp refused",
            not reply.success and "default_kp" in reply.message,
            reply.message,
        )

        # Another /lowcmd writer (a restarted robot's motion service) blocks arming.
        other = self.node.create_publisher(LowCmd, "/lowcmd", 10)
        self.spin(1.0)
        reply = self.start(0.123)
        self.check(
            "start_control refused with another /lowcmd publisher",
            not reply.success and "/lowcmd" in reply.message,
            reply.message,
        )
        self.node.destroy_publisher(other)
        self.spin(1.0)

        # A stalled /lowstate refuses arming and names the cause. 0.3 s: past state_loss
        # (0.2 s). checkState_ runs before the remote check, so the reply names state_loss.
        self.pause.publish(Bool(data=True))
        self.spin(0.3)
        reply = self.start(0.123)
        self.check(
            "start_control refused on a stalled /lowstate",
            not reply.success and "state_loss" in reply.message,
            reply.message,
        )
        self.pause.publish(Bool(data=False))
        self.spin(0.5)
        self.check_state("still DISARMED", "DISARMED")
        self.check("refusals published nothing", self.count == count, f"{self.count - count} frames")

        # Remote damp while disarmed: latches, publishes nothing.
        self.press(KEY_L1 | KEY_R1)
        self.press(0)
        self.check_state("L1+R1 while disarmed latches silent", "LATCHED damp remote_damp silent")
        count = self.count
        self.spin(0.5)
        self.check(
            "silent latch publishes nothing", self.count == count, f"{self.count - count} frames"
        )
        self.check("start_control refused while latched", not self.start(0.123).success)
        self.check("estop_reset", self.client.estop_reset().success)
        self.check_state("reset from silent -> DISARMED", "DISARMED")

        # Arm.
        self.check("start_control", self.start(0.123).success)
        self.spin(RAMP_SECONDS)
        self.check_state("ARMED", "ARMED")
        self.check("ramp reached 0.123", *self.frame(kp, kd, q=0.123))

        # An order outside [0, 1] used to throw on the executor thread and end the bridge.
        n = NUM_MOTORS
        bad = pack_robot_cmd([0.3] * n, [0.0] * n, [0.0] * n, kp, kd, interpolation_order=2.0, duration=0.1)
        self.client.cmd_publisher.publish(bad)
        self.spin(0.3)
        hz = self.rate(0.5)
        self.check("bad interpolation_order dropped, bridge alive", hz > 500, f"{hz:.0f} Hz")
        self.check("ARMED, still at 0.123", *self.frame(kp, kd, q=0.123))

        # Feedforward from the last client: the next arm must drop it, its hold has no timeout.
        self.send(0.123, hold_position=True, dq=0.05, tau=0.5)
        self.spin(0.3)
        self.check("dq and tau reach /lowcmd", *self.feedforward(0.05, 0.5, self.frames[-1:]))

        # Operator damp.
        self.client.estop_damp()
        self.check_state("estop_damp latches", "LATCHED damp service_damp")
        self.spin(0.2)
        self.check("damp frame: kp 0, kd damp_kd", *self.frame(damp_kp, damp_kd, q=0.123))
        hz = self.rate()
        self.check("damp at ~1 kHz", 0.8 * BRIDGE_HZ < hz < 1.2 * BRIDGE_HZ, f"{hz:.0f} Hz")
        self.send(0.3, hold_position=True)
        self.spin(0.3)
        self.check("/robot_cmd dropped while latched", *self.frame(damp_kp, damp_kd, q=0.123))
        self.check("start_control refused while latched", not self.start(0.2).success)

        # Operator hold over damp, by remote.
        self.press(KEY_R1 | KEY_R2)
        self.check_state("R1+R2 re-stiffens to hold", "LATCHED hold remote_hold")
        self.check("hold frame: config gains, q kept", *self.frame(kp, kd, q=0.123))
        self.check("estop_reset refused while combo held", not self.client.estop_reset().success)
        self.press(0)
        self.check("estop_reset", self.client.estop_reset().success)
        self.check_state("reset -> STOPPED, still holding", "STOPPED hold remote_hold")
        self.spin(0.2)
        self.check("STOPPED keeps the hold frame", *self.frame(kp, kd, q=0.123))

        # Re-arm, then /stop_control: hold, no latch.
        rearm = time.monotonic()
        self.check("start_control after reset", self.start(0.2).success)
        self.spin(RAMP_SECONDS)
        self.check("ramp reached 0.2", *self.frame(kp, kd, q=0.2))
        rearmed = [f for f in self.frames if f[0] >= rearm]
        self.check("re-arm sends dq 0 and tau 0", *self.feedforward(0.0, 0.0, rearmed))
        self.client.stop_control()
        self.check_state("stop_control holds without latch", "STOPPED hold stop")
        reply = self.client.start_control(default_position=[], timeout_sec=30.0)
        self.check("start_control [] without reset", reply.success, reply.message)
        self.spin(RAMP_SECONDS)
        self.check_state("ARMED again", "ARMED")
        self.check("[] ramped to ready_q", *self.at(self.params["ready_q"]))

        # Another /lowcmd sender while armed (motion service, a script): two senders fight.
        other = self.node.create_publisher(LowCmd, "/lowcmd", 10)  # never publishes
        appeared = time.monotonic()
        latched = self.wait_state("LATCHED damp other_lowcmd", timeout=3.0)
        took = time.monotonic() - appeared
        self.check(
            "another /lowcmd publisher latches damp",
            latched and took <= 1.5,
            f"{took:.2f} s, state '{self.state}'",
        )
        self.spin(0.2)
        self.check("other_lowcmd frame is damp", *self.frame(damp_kp, damp_kd))
        reply = self.client.estop_reset()
        self.check(
            "estop_reset refused while it is there",
            not reply.success and "other_lowcmd" in reply.message,
            reply.message,
        )
        self.node.destroy_publisher(other)
        self.spin(1.0)
        self.check("estop_reset once it is gone", self.client.estop_reset().success)
        self.check_state("reset -> STOPPED damp", "STOPPED damp other_lowcmd")

        # Ready and zero move every joint: armed, they still need the remote.
        # Armed at 0.2 (not ready_q), so an accepted ready must move every joint.
        self.client.stop_control()
        self.check("start_control at 0.2", self.start(0.2).success)
        self.spin(RAMP_SECONDS)
        self.check("ramp reached 0.2", *self.frame(kp, kd, q=0.2))
        self.remote.publish(Bool(data=False))
        self.spin(1.0)
        for name, client in (("ready", self.ready), ("zero", self.zero)):
            reply = self.client._call(client, Trigger.Request(), 30.0)
            self.check(
                f"{name} refused while armed without the remote",
                not reply.success and "remote" in reply.message,
                reply.message,
            )
        self.check("refused ready/zero kept 0.2", *self.frame(kp, kd, q=0.2))
        self.remote.publish(Bool(data=True))
        self.spin(0.3)
        reply = self.client._call(self.ready, Trigger.Request(), 30.0)
        self.check("ready accepted while armed with the remote", reply.success, reply.message)
        self.spin(RAMP_SECONDS)
        self.check_state("ARMED after ready", "ARMED")
        self.check("ready moved 0.2 -> ready_q", *self.at(self.params["ready_q"]))

        # Command timeout: hold, latched.
        self.send(0.25, hold_position=False, duration=0.1)
        self.spin(0.8)
        self.check_state("timeout latches hold", "LATCHED hold cmd_timeout")
        self.check("timeout holds the last pose", *self.frame(kp, kd, q=0.25))

        # IMU fault escalates hold to damp; reset refused while tilted.
        self.roll.publish(Float32(data=1.5))
        self.check_state("IMU fault escalates to damp", "LATCHED damp imu")
        self.spin(0.2)
        self.check("escalated frame is damp", *self.frame(damp_kp, damp_kd))
        self.check("estop_reset refused while tilted", not self.client.estop_reset().success)

        # Operator hold while the fault lasts: it wins and stays (faults fire at onset).
        self.press(KEY_R1 | KEY_R2)
        self.press(0)
        self.check_state("R1+R2 during a lasting fault -> hold", "LATCHED hold remote_hold")
        self.spin(0.5)
        self.check(
            "hold stays while the fault lasts",
            self.state == "LATCHED hold remote_hold",
            f"state '{self.state}'",
        )
        self.check("hold frame during the fault", *self.frame(kp, kd, q=0.25))
        self.roll.publish(Float32(data=0.0))
        self.spin(0.3)
        self.check("estop_reset after level", self.client.estop_reset().success)
        self.check_state("reset -> STOPPED hold", "STOPPED hold remote_hold")

        # Both combos at once: damp wins.
        self.press(KEY_L1 | KEY_R1 | KEY_R2)
        self.press(0)
        self.check_state("L1+R1+R2 -> damp", "LATCHED damp remote_damp")
        self.spin(0.3)

        # /lowstate gone: damp frames for 1 s more, then silent until reset.
        self.pause.publish(Bool(data=True))
        self.spin(0.6)
        count = self.count
        self.spin(0.3)
        self.check("frames go on right after the loss", self.count > count, f"{self.count - count}")
        self.check_state("silent 1.2 s after the loss", "LATCHED damp remote_damp silent")
        # /bridge_state is taken first: frames sent before the latch can still be queued.
        before = self.count
        quiet = self.drain()
        count = self.count
        self.spin(0.5)
        detail = f"{self.count - count} after {count - before} drained"
        detail += "" if quiet else ", no 0.1 s gap in 2 s"
        self.check("silent latch publishes nothing", quiet and self.count == count, detail)
        self.pause.publish(Bool(data=False))
        self.spin(0.5)
        self.check("estop_reset once /lowstate is back", self.client.estop_reset().success)
        self.check_state("reset from silent -> DISARMED", "DISARMED")
        self.bad_state(damp_kp, damp_kd)
        self.check_recording()

    def bad_state(self, damp_kp, damp_kd):
        """Bad robot data while ARMED latches damp state_invalid; reset once it is gone."""
        limit = self.params["dq_limit"][0]
        self.check("start_control at 0.2", self.start(0.2).success)
        self.spin(RAMP_SECONDS)
        self.check_state("ARMED", "ARMED")
        self.dq.publish(Float32(data=0.9 * limit))
        self.spin(0.3)
        under = self.state == "ARMED"
        self.check("joint 0 at 0.9 dq_limit stays ARMED", under, f"state '{self.state}'")
        self.dq.publish(Float32(data=0.0))
        faults = (
            ("NaN q", self.nan_q, Bool(data=True), Bool(data=False)),
            ("joint 0 at 1.1 dq_limit", self.dq, Float32(data=1.1 * limit), Float32(data=0.0)),
            ("NaN IMU roll", self.nan_imu, Bool(data=True), Bool(data=False)),
        )
        for i, (name, publisher, on, off) in enumerate(faults):
            if i:
                self.check(f"{name}: re-arm", self.start(0.2).success)
                self.spin(RAMP_SECONDS)
                self.check_state(f"{name}: ARMED", "ARMED")
            publisher.publish(on)
            self.check_state(f"{name} latches damp state_invalid", "LATCHED damp state_invalid")
            self.spin(0.2)
            self.check(f"{name}: damp frame", *self.frame(damp_kp, damp_kd))
            refused = not self.client.estop_reset().success
            self.check(f"{name}: reset refused while it lasts", refused)
            publisher.publish(off)
            self.spin(0.3)
            self.check(f"{name}: reset once it is gone", self.client.estop_reset().success)
            self.check_state(f"{name}: reset -> STOPPED damp", "STOPPED damp state_invalid")

    def check_recording(self):
        """Every frame, not only the last: a damp latch has kp 0, a hold never moves."""
        times = [t for t, _ in self.changes]
        damp_bad = hold_bad = silent_bad = checked = 0
        hold_q = {}  # change index -> first q in that interval
        for t, kp, q, _, _ in self.frames:
            i = bisect.bisect_right(times, t) - 1
            if i < 0:
                continue
            near = [c for c in times[max(i - 1, 0) : i + 2] if abs(c - t) < GRACE]
            if near:
                continue
            state = self.changes[i][1]
            checked += 1
            if state.endswith("silent"):
                silent_bad += 1
            elif state.startswith(("LATCHED damp", "STOPPED damp")):
                damp_bad += any(abs(k) > 1e-6 for k in kp)
            elif state.startswith(("LATCHED hold", "STOPPED hold")):
                first = hold_q.setdefault(i, q)
                hold_bad += any(abs(a - b) > 1e-3 for a, b in zip(q, first))
        self.check("recording: frames checked", checked > 10000, f"{checked} of {len(self.frames)}")
        self.check("recording: no frame while latched silent", silent_bad == 0, f"{silent_bad}")
        self.check("recording: every damp frame has kp 0", damp_bad == 0, f"{damp_bad} bad")
        self.check("recording: no hold frame moves", hold_bad == 0, f"{hold_bad} bad")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("params", help="the G1_config.yaml the bridge runs with")
    parser.add_argument("bridge_log", help="the bridge's log, read for its /lowcmd check")
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("estop_driver")
    driver = Driver(node, load_params(args.params), args.bridge_log)
    print(HOLDING, flush=True)  # run_fake_estop_test.sh starts the bridge on this line
    try:
        driver.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if driver.failures:
        print("\n{} check(s) failed: {}".format(len(driver.failures), ", ".join(driver.failures)))
        sys.exit(1)
    print("\nall checks passed")


if __name__ == "__main__":
    main()
