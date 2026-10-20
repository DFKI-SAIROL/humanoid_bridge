"""G1 client for the probing motion controller (g1_raps deploy bundle) via humanoid_bridge.

Every 20 ms: read /lowstate, build the 338-col observation (bundle's reference_obs.py), run
policy.onnx, send target = default + scale * action with the policy's kp/kd to /robot_cmd
(interpolation_order=0, duration=0.02, hold_position=False -> the bridge stops on its own if
this client goes quiet for 0.22 s).

Modes:
  IDLE     nothing is published; the bridge holds whatever it was told (e.g. HOME).
  POLICY   policy running; reference = held stand, or a motion pack, then back to held stand.
  DAMPING  kp ramps to ~0 over 1 s (last target held), then pure damping around the current q.

--shadow runs the policy on live state but never publishes or calls a bridge service.

Web commander: http://<web-host>:<web-port>/ (default 127.0.0.1:8080) has the same commands.

Terminal commands (type + Enter):
  home        /start_control with the bundle's HOME pose (bridge moves there in 2 s with its own gains)
  go          start the policy on the held-stand reference (robot must be near HOME)
  ls          list motions (index, trained flag, seconds)
  m <id|idx>  play a motion (only from held stand; untrained P2 needs --allow-untrained)
  e           damping (soft e-stop)
  stop        damping if running, then /stop_control; the client keeps running
  q           like stop, then save the log and exit
Remote: L2 + B = damping. The bridge's own L2 + UP + LEFT still stops the bridge.

Recording (run_store.py, under --log-dir): every motion is saved as its own file in runs/, from
its first row to its end; if it is damped, through the 1 s damping ramp. Everything while the
policy runs also goes to sessions/ (chunks of 5 min). The web commander's Data tab lists,
downloads and deletes them.
"""
import argparse
import collections
import hashlib
import queue
import signal
import sys
import threading
import time
from datetime import datetime

import numpy as np
import rclpy
from bridge_interface.msg import MotorCmd, RobotCmd
from bridge_interface.srv import SetDefaultPosition
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from robot_bridge_py.robot_client import KeyMap, RemoteController
from std_srvs.srv import Trigger
from unitree_hg.msg import LowState

import web_commander
from probe_core import ProbeCore, tilt_rad
from run_store import RunStore

CONTROL_DT = 0.02
NUM_DOF = 29
DAMPING_RAMP_S = 1.0
# kp never reaches exactly 0: the bridge's torque clamp divides by kp.
KP_FLOOR = 1e-2
STATE_TIMEOUT_S = 0.1
HOME_SETTLE_S = 2.5          # bridge interpolates to HOME in 2.0 s (G1_config.yaml duration)
SESSION_CHUNK_TICKS = 15000  # 5 min at 50 Hz

IDLE, POLICY, DAMPING = "IDLE", "POLICY", "DAMPING"
MODE_CODE = {IDLE: 0, POLICY: 1, DAMPING: 2}


class ProbeClient:
    def __init__(self, node, core: ProbeCore, args):
        self.node = node
        self.core = core
        self.args = args
        self.log = node.get_logger()
        d = core.deploy
        self.kp = d.kp.astype(np.float32)
        self.kd = d.kd.astype(np.float32)
        self.home = d.home.astype(np.float32)

        self.mode = IDLE
        self.home_requested_at = None
        self.damping_t0 = None
        self.damping_kp0 = None
        self.last_target = None

        self.state_time = None
        self.state_raw = None       # newest /lowstate, still serialized
        self.state_raw_time = None
        self.state_tick = 0
        self.quat = np.array([1.0, 0.0, 0.0, 0.0])
        self.gyro = np.zeros(3)
        self.q = np.zeros(NUM_DOF)
        self.dq = np.zeros(NUM_DOF)
        self.tau_est = np.zeros(NUM_DOF)
        self.remote = RemoteController()

        self.cmd = RobotCmd()
        self.cmd.motor_cmd = [MotorCmd() for _ in range(NUM_DOF)]
        self.cmd.interpolation_order = 0.0
        self.cmd.hold_position = False
        self.cmd.duration = CONTROL_DT

        # Raw: rclpy (Foxy, ARM) cannot convert 1000 LowState/s to Python objects and still run the
        # policy on time; only the newest message is converted, once per tick.
        node.create_subscription(LowState, "/lowstate", self._on_lowstate, qos_profile_sensor_data, raw=True)
        self.pub = None if args.shadow else node.create_publisher(RobotCmd, "/robot_cmd", 1)
        self.start_cli = node.create_client(SetDefaultPosition, "/start_control")
        self.stop_cli = node.create_client(Trigger, "/stop_control")

        self.commands = queue.Queue()
        self.events = collections.deque(maxlen=200)
        self.stop_requested = False
        self.quit_requested = False
        self.done = False
        self.records = []
        self.store = RunStore(args.log_dir)
        self.run = None
        self.last_kp = self.kp
        self.provenance = {
            "bundle": str(core.bundle_dir),
            "policy_sha256": hashlib.sha256((core.bundle_dir / "policy" / "policy.onnx").read_bytes()).hexdigest(),
            "checkpoint_sha256": d.cfg["provenance"]["checkpoint_sha256"],
            "joint_names": list(core.joint_names),
            "control_dt": CONTROL_DT,
            "tilt_limit": args.tilt_limit,
            "shadow": args.shadow,
        }
        self.last_print = 0.0
        self.bridge_ready = False
        node.create_timer(CONTROL_DT, self._tick)
        node.create_timer(0.5, self._check_bridge)
        threading.Thread(target=self._read_stdin, daemon=True).start()

    def _say(self, level, msg):
        # rclpy wants one severity per call site, so no dict dispatch here.
        if level == "ERROR":
            self.log.error(msg)
        elif level == "WARN":
            self.log.warn(msg)
        else:
            self.log.info(msg)
        self.events.append({"time": datetime.now().strftime("%H:%M:%S"), "level": level, "msg": msg})

    # ---------------------------------------------------------------- inputs
    def _on_lowstate(self, raw):
        self.state_raw = raw
        self.state_raw_time = time.monotonic()

    def _update_state(self):
        """Convert the newest /lowstate (if any arrived since the last tick)."""
        if self.state_raw is None:
            return
        raw, self.state_raw = self.state_raw, None
        msg = deserialize_message(raw, LowState)
        self.state_time = self.state_raw_time
        self.state_tick = msg.tick
        self.quat = np.array(msg.imu_state.quaternion, dtype=np.float64)
        self.gyro = np.array(msg.imu_state.gyroscope, dtype=np.float64)
        for i in range(NUM_DOF):
            m = msg.motor_state[i]
            self.q[i], self.dq[i], self.tau_est[i] = m.q, m.dq, m.tau_est
        self.remote.set(bytes(msg.wireless_remote))
        if self.remote.new_event and self.remote.is_exact_combo(self.remote.button, [KeyMap.L2, KeyMap.B]):
            self.commands.put("e")

    def _read_stdin(self):
        for line in sys.stdin:
            if line.strip():
                self.commands.put(line.strip())

    def _check_bridge(self):
        """The bridge registers /start_control before it leaves its debug-mode wait, and a request
        sent then would run later. Ready = the bridge is the only /lowcmd publisher."""
        pubs = self.node.get_publishers_info_by_topic("/lowcmd")
        ready = (not self.args.shadow and self.start_cli.service_is_ready()
                 and len(pubs) == 1 and pubs[0].node_name == "robot_bridge")
        if ready != self.bridge_ready:
            self._say("INFO" if ready else "WARN",
                      "bridge ready (debug mode, publishing /lowcmd)" if ready else
                      "bridge not ready (not running, or robot not in debug mode)")
        self.bridge_ready = ready

    def _state_fresh(self):
        return self.state_time is not None and time.monotonic() - self.state_time < STATE_TIMEOUT_S

    def _home_ready(self):
        return self.home_requested_at is not None and time.monotonic() - self.home_requested_at >= HOME_SETTLE_S

    def _home_moving(self):
        return self.home_requested_at is not None and not self._home_ready()

    def _damping_done(self):
        return self.mode == DAMPING and time.monotonic() - self.damping_t0 >= DAMPING_RAMP_S

    # ---------------------------------------------------------------- commands
    def _handle(self, line):
        parts = line.split()
        name = parts[0].lower()
        if name == "ls":
            for i, m in enumerate(self.core.manifest):
                flag = "trained" if m["trained_in_TI_B_inclHO"] else "UNTRAINED"
                print(f"  {i:2d} {m['motion_id']:40s} {flag:9s} {m['seconds']:5.1f}s  sim falls {m['sim_nominal_falls']}")
        elif name == "home":
            self._cmd_home()
        elif name == "go":
            self._cmd_go()
        elif name == "m" and len(parts) == 2:
            self._cmd_motion(parts[1])
        elif name == "e":
            self._enter_damping("operator")
        elif name in ("stop", "q"):
            self.stop_requested = True
            self.quit_requested |= name == "q"
            if self.mode == POLICY:
                self._enter_damping(name)
        else:
            print(__doc__.split("Terminal commands")[1])

    def _cmd_home(self):
        if self.args.shadow:
            self._say("WARN", "shadow mode: not calling /start_control")
            return
        if self.mode == POLICY or (self.mode == DAMPING and not self._damping_done()):
            self._say("WARN", f"'home' only from IDLE or after the damping ramp (now {self.mode})")
            return
        if not self.bridge_ready:
            self._say("ERROR", "bridge not ready (is G1_bridge running and the robot in debug mode, L2+R2?)")
            return
        if not self._state_fresh():
            self._say("ERROR", "no fresh /lowstate (link to the robot down?)")
            return
        if self._home_moving():
            self._say("WARN", "HOME already in progress")
            return
        req = SetDefaultPosition.Request()
        req.default_position = [float(v) for v in self.home]
        self.start_cli.call_async(req)
        self.mode = IDLE
        self.home_requested_at = time.monotonic()
        self._say("INFO", "start_control sent: bridge moves to HOME in 2 s with its own gains")

    def _cmd_go(self):
        if self.mode != IDLE:
            self._say("WARN", f"'go' only from IDLE (now {self.mode}); after damping, run 'home' first")
            return
        if not self._state_fresh():
            self._say("ERROR", "no fresh /lowstate")
            return
        err = np.abs(self.q - self.home)
        if err.max() > self.args.home_tol:
            j = int(err.argmax())
            msg = f"robot not at HOME: {self.core.joint_names[j]} off by {err[j]:.3f} rad (tol {self.args.home_tol})"
            if not self.args.shadow:
                self._say("ERROR", msg + "; run 'home' first")
                return
            self._say("WARN", msg + " (shadow: running anyway)")
        if not self.args.shadow and not self._home_ready():
            self._say("ERROR", "run 'home' first and wait for the bridge to reach HOME")
            return
        self.core.reset()
        self.last_target = None
        self.mode = POLICY
        self._say("INFO", "policy started on held stand" + (" (shadow, not publishing)" if self.args.shadow else ""))

    def _cmd_motion(self, key):
        if self.mode != POLICY or self.core.ref_name != "held_stand":
            self._say("WARN", "motions start only from the held stand while the policy runs")
            return
        ids = self.core.motion_ids
        motion_id = ids[int(key)] if key.isdigit() and int(key) < len(ids) else key
        if motion_id not in self.core.trained:
            self._say("ERROR", f"unknown motion '{key}' (see 'ls')")
            return
        if not self.core.trained[motion_id] and not self.args.allow_untrained:
            self._say("ERROR", f"{motion_id} was not trained; rerun with --allow-untrained to play it")
            return
        self.core.start_motion(motion_id)
        self.run = {
            "motion_id": motion_id,
            "q_ref": np.load(self.core.bundle_dir / "motions" / f"{motion_id}.npz")["q_ref"].astype(np.float32),
            "ticks": [],
            "start": self.store.now(),
            "reason": None,
        }
        self._say("INFO", f"motion {motion_id} ({len(self.core.player.rows)} rows), recording")

    def _enter_damping(self, reason):
        if self.mode == DAMPING:
            return
        if self.mode == IDLE:
            if self.args.shadow or self.home_requested_at is None:
                self._say("WARN", "damping requested while IDLE: nothing is commanded by this client")
                return
            # The bridge is holding HOME with its own gains: take over and ramp down from there.
            self.last_target = self.home.copy()
            reason += ", taking over the bridge's HOME hold"
        self._say("WARN", f"DAMPING ({reason})")
        if self.run is not None:
            self.run["reason"] = reason
        self.mode = DAMPING
        self.damping_t0 = time.monotonic()
        self.damping_kp0 = self.kp.copy()
        if self.last_target is None:
            self.last_target = self.q.astype(np.float32).copy()

    def _stop_bridge(self):
        """Called once the policy is not running (IDLE, or damping ramp finished)."""
        self.stop_requested = False
        if not self.args.shadow and self.stop_cli.service_is_ready():
            self.stop_cli.call_async(Trigger.Request())
            self._say("INFO", "stop_control sent")
        elif not self.args.shadow:
            self._say("WARN", "/stop_control not available")
        self.mode = IDLE
        self.home_requested_at = None
        self.save_log()
        if self.quit_requested:
            self.done = True
            raise SystemExit(0)

    # ---------------------------------------------------------------- loop
    def _tick(self):
        self._update_state()
        while not self.commands.empty():
            self._handle(self.commands.get())

        now = time.monotonic()
        obs = raw = None
        motion_done = False
        if self.mode == POLICY:
            if not self._state_fresh():
                self._enter_damping("stale /lowstate")
            else:
                tilt = tilt_rad(self.quat)
                obs, raw, target = self.core.step(self.quat, self.gyro, self.q, self.dq)
                if not (np.all(np.isfinite(obs)) and np.all(np.isfinite(raw))):
                    self._enter_damping("NaN in obs/action")
                elif tilt > self.args.tilt_limit and not self.args.shadow:
                    self._enter_damping(f"tilt {tilt:.2f} rad > {self.args.tilt_limit}")
                else:
                    prev_ref = self.core.ref_name
                    self.last_target = target.astype(np.float32)
                    self._publish(self.last_target, self.kp, self.kd)
                    if prev_ref != "held_stand" and self.core.player.k >= len(self.core.player.rows):
                        self._say("INFO", f"motion {prev_ref} finished, back to held stand")
                        motion_done = True

        if self.mode == DAMPING:
            s = min(1.0, (now - self.damping_t0) / DAMPING_RAMP_S)
            kp = np.maximum(self.damping_kp0 * (1.0 - s), KP_FLOOR).astype(np.float32)
            target = self.last_target if s < 1.0 else self.q.astype(np.float32)
            self._publish(target, kp, self.kd)

        self._record(now, obs, raw)
        if motion_done:
            self._finish_run("completed")
        elif self.run is not None and self._damping_done():
            self._finish_run("damped")
        self._print_status(now, obs)

        if self.stop_requested and (self.mode == IDLE or self._damping_done()):
            self._stop_bridge()

    def _publish(self, q_target, kp, kd):
        self.last_kp = kp
        if self.pub is None:
            return
        for i in range(NUM_DOF):
            c = self.cmd.motor_cmd[i]
            c.q, c.dq, c.tau = float(q_target[i]), 0.0, 0.0
            c.kp, c.kd = float(kp[i]), float(kd[i])
        self.pub.publish(self.cmd)

    def _record(self, now, obs, raw):
        """One tick for the session log and, during a motion, its run. ref_row = reference row
        fed to the policy this tick (-1 when the policy did not step)."""
        if self.mode == IDLE and obs is None:
            return
        tick = {
            "t": now,
            "wall": self.store.now(),
            "mode": MODE_CODE[self.mode],
            "ref_name": self.core.ref_name,
            "ref_row": self.core.ref_k - 1 if obs is not None else -1,
            "obs": np.zeros(338, np.float32) if obs is None else obs,
            "action": np.full(NUM_DOF, np.nan, np.float32) if raw is None else raw,
            "target": np.full(NUM_DOF, np.nan, np.float32) if self.last_target is None else self.last_target.copy(),
            "kp": np.asarray(self.last_kp, np.float32).copy(),
            "q": self.q.astype(np.float32),
            "dq": self.dq.astype(np.float32),
            "tau_est": self.tau_est.astype(np.float32),
            "quat_wxyz": self.quat.astype(np.float32),
            "gyro": self.gyro.astype(np.float32),
            "state_age": np.float32(now - self.state_time) if self.state_time is not None else np.float32(np.nan),
            "lowstate_tick": np.uint32(self.state_tick),
        }
        self.records.append(tick)
        if self.run is not None:
            q_ref = self.run["q_ref"]
            row = tick["ref_row"] if tick["ref_row"] >= 0 else (self.run["ticks"][-1]["ref_row"] if self.run["ticks"] else 0)
            run_tick = {k: v for k, v in tick.items() if k != "ref_name"}
            run_tick["q_ref"] = q_ref[min(max(row, 0), len(q_ref) - 1)]
            self.run["ticks"].append(run_tick)
        if len(self.records) >= SESSION_CHUNK_TICKS:
            self.save_log()

    def _finish_run(self, outcome, sync=False):
        run, self.run = self.run, None
        if run is None or not run["ticks"]:
            return
        ticks = run["ticks"]
        rows = [t["ref_row"] for t in ticks if t["ref_row"] >= 0]
        end = self.store.now()
        meta = dict(
            self.provenance,
            motion_id=run["motion_id"],
            outcome=outcome if run["reason"] is None else f"{outcome} ({run['reason']})",
            reason=run["reason"],
            start_iso=self.store.iso(run["start"]),
            end_iso=self.store.iso(end),
            duration_s=round(end - run["start"], 3),
            n_ticks=len(ticks),
            rows_total=len(run["q_ref"]),
            rows_done=(max(rows) + 1) if rows else 0,
            max_tilt=round(max(tilt_rad(t["quat_wxyz"]) for t in ticks), 4),
            manifest=next(m for m in self.core.manifest if m["motion_id"] == run["motion_id"]),
        )
        name = self.store.save("runs", run["motion_id"], ticks, meta, sync=sync)
        self._say("INFO", f"run saved: {name} [{meta['outcome']}, {len(ticks)} ticks]")

    def _print_status(self, now, obs):
        if now - self.last_print < 1.0:
            return
        self.last_print = now
        if not self._state_fresh():
            print(f"[{self.mode}] waiting for /lowstate")
            return
        g = obs[0:3] if obs is not None else None
        line = (f"[{self.mode}] ref {self.core.ref_name}[{self.core.ref_k}]  tilt {tilt_rad(self.quat):.3f}  "
                f"|q-home|max {np.abs(self.q - self.home).max():.3f}  |dq|max {np.abs(self.dq).max():.2f}")
        if g is not None:
            line += f"  grav [{g[0]:+.2f} {g[1]:+.2f} {g[2]:+.2f}]  |a|max {np.abs(self.core.prev_action).max():.2f}"
        print(line)

    def status(self):
        """Snapshot for the web commander (read from the HTTP thread)."""
        fresh = self._state_fresh()
        err = np.abs(self.q - self.home)
        rows = self.core.player.rows
        return {
            "mode": self.mode,
            "shadow": self.args.shadow,
            "state_fresh": fresh,
            "tilt": tilt_rad(self.quat) if fresh else None,
            "tilt_limit": self.args.tilt_limit,
            "q_home_err": float(err.max()),
            "q_home_joint": self.core.joint_names[int(err.argmax())],
            "dq_max": float(np.abs(self.dq).max()),
            "action_max": float(np.abs(self.core.prev_action).max()) if self.mode == POLICY else None,
            "home_ready": self._home_ready(),
            "home_age": None if self.home_requested_at is None else time.monotonic() - self.home_requested_at,
            "ref_name": self.core.ref_name,
            "ref_k": self.core.ref_k,
            "ref_rows": None if rows is None else len(rows),
            "allow_untrained": self.args.allow_untrained,
            "bridge_ready": self.bridge_ready,
            "can": {
                "home": (self.bridge_ready and fresh and not self._home_moving()
                         and (self.mode == IDLE or self._damping_done())),
                "go": self.mode == IDLE and fresh and (self.args.shadow or self._home_ready()),
                "motion": self.mode == POLICY and self.core.ref_name == "held_stand",
                "damp": self.mode == POLICY or (self.mode == IDLE and not self.args.shadow
                                                 and self.home_requested_at is not None),
                "stop": not self.args.shadow and (self.mode != IDLE or self.home_requested_at is not None),
            },
            "motions": [{"id": m["motion_id"], "trained": bool(m["trained_in_TI_B_inclHO"]),
                         "seconds": m["seconds"], "priority": m["priority"], "falls": m["sim_nominal_falls"]}
                        for m in self.core.manifest],
            "events": list(self.events)[-50:],
            "recording": None if self.run is None else {"motion_id": self.run["motion_id"], "ticks": len(self.run["ticks"])},
            "clock_set": self.store.clock_set,
        }

    def save_log(self, sync=False):
        records, self.records = self.records, []
        if not records:
            return
        meta = dict(self.provenance, start_iso=self.store.iso(records[0]["wall"]),
                    end_iso=self.store.iso(records[-1]["wall"]), n_ticks=len(records),
                    duration_s=round(records[-1]["t"] - records[0]["t"], 3),
                    motions=sorted({r["ref_name"] for r in records} - {"held_stand"}))
        name = self.store.save("sessions", "session_shadow" if self.args.shadow else "session", records, meta, sync=sync)
        self._say("INFO", f"session log saved: {name} ({len(records)} ticks)")

    def shutdown(self):
        """Process exit: write whatever is in memory before returning."""
        if self.run is not None:
            self._finish_run("client exit", sync=True)
        self.save_log(sync=True)
        self.store.flush()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--bundle", required=True, help="g1_raps deploy bundle dir (has deploy.yaml)")
    parser.add_argument("--shadow", action="store_true", help="run the policy on live state, publish nothing")
    parser.add_argument("--allow-untrained", action="store_true", help="allow the 16 untrained P2 motions")
    parser.add_argument("--tilt-limit", type=float, default=0.6, help="rad; damping above this (sim max 0.29, train fall 0.8)")
    parser.add_argument("--home-tol", type=float, default=0.15, help="rad; max |q - HOME| to allow 'go'")
    parser.add_argument("--log-dir", default="probe_logs")
    parser.add_argument("--web-host", default="127.0.0.1", help="0.0.0.0 to reach the page from another device")
    parser.add_argument("--web-port", type=int, default=8080, help="0 disables the web commander")
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("g1_probe_client")
    client = ProbeClient(node, ProbeCore(args.bundle), args)

    # Set explicitly: a process started with '&' from a script inherits SIGINT ignored.
    # First SIGINT/SIGTERM = 'q' (damp if running, stop the bridge, save, exit); second = exit now.
    def on_signal(signum, frame):
        if client.quit_requested:
            raise KeyboardInterrupt
        client.commands.put("q")

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    if args.web_port:
        web_commander.serve(client, args.web_host, args.web_port)
        print(f"Web commander: http://{args.web_host}:{args.web_port}/")
    print(__doc__.split("Terminal commands")[1])
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        client.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
