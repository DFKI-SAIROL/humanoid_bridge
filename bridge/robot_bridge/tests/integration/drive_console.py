"""Drives a running robot_console against the real G1_bridge and fake_g1.py.

HTTP + WebSocket like a browser tab; /lowcmd is watched over ROS to prove what
the console's buttons did to the wire. Exit 1 on failure. run_fake_console_test.sh
starts everything first.
"""

import argparse
import base64
import http.client
import json
import os
import socket
import sys
import threading
import time

import rclpy
import yaml
from bridge_interface.msg import RobotCmd
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import UInt16
from unitree_hg.msg import LowCmd

KEY_R1, KEY_L1 = 1 << 0, 1 << 1
ELBOW = 18  # L_ELBOW in the body command vector
COOKIE = "robot_console_token"


def read_message(stream):
    head = stream.read(2)
    if len(head) < 2:
        return None
    length = head[1] & 0x7F
    if length == 126:
        length = int.from_bytes(stream.read(2), "big")
    elif length == 127:
        length = int.from_bytes(stream.read(8), "big")
    return json.loads(stream.read(length))


class Tab:
    """One browser tab: the stream socket, pings while open, POSTs with its client id."""

    def __init__(self, address, token):
        self.address, self.token = address, token
        self.sock = socket.create_connection(address, timeout=5)
        key = base64.b64encode(os.urandom(16)).decode()
        self.sock.sendall(
            (
                f"GET /api/stream HTTP/1.1\r\nHost: {address[0]}:{address[1]}\r\n"
                f"Cookie: {COOKIE}={token}\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
                f"Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n"
            ).encode()
        )
        header = b""
        while b"\r\n\r\n" not in header:
            header += self.sock.recv(1)
        if b" 101 " not in header.split(b"\r\n")[0]:
            raise RuntimeError(f"no websocket: {header[:80]!r}")
        self.stream = self.sock.makefile("rb")
        self.client = read_message(self.stream)["client"]
        self.snap = None
        self.alive = True
        threading.Thread(target=self._pump, daemon=True).start()

    def _pump(self):
        mask = b"\x07\x08\x09\x0a"
        ping = bytes(b ^ mask[i % 4] for i, b in enumerate(b'{"type":"ping"}'))
        last = 0.0
        while self.alive:
            try:
                msg = read_message(self.stream)
                if msg is None:
                    return
                self.snap = msg
                if time.monotonic() - last > 0.2:
                    self.sock.sendall(bytes([0x81, 0x80 | len(ping)]) + mask + ping)
                    last = time.monotonic()
            except (OSError, ValueError):
                return

    def post(self, path, **body):
        conn = http.client.HTTPConnection(*self.address, timeout=20)
        conn.request(
            "POST",
            path,
            body=json.dumps({**body, "client": self.client}),
            headers={"Cookie": f"{COOKIE}={self.token}", "Content-Type": "application/json"},
        )
        reply = conn.getresponse()
        data = json.loads(reply.read())
        conn.close()
        return reply.status, data

    def close(self):
        self.alive = False
        self.stream.close()
        self.sock.close()


class Wire:
    """The latest /lowcmd and its rate, spun on a thread."""

    def __init__(self, node):
        self.last = None
        self.count = 0
        node.create_subscription(LowCmd, "/lowcmd", self._on, 10)
        self.robot_cmds = 0
        node.create_subscription(RobotCmd, "/robot_cmd", self._on_robot_cmd, 10)
        self.keys = node.create_publisher(UInt16, "/fake_g1/keys", 10)

    def _on_robot_cmd(self, msg):
        self.robot_cmds += 1

    def _on(self, msg):
        self.last = msg
        self.count += 1


class Driver:
    def __init__(self, address, token, kp):
        self.address, self.token, self.kp = address, token, kp
        self.failures = []

    def check(self, name, ok, detail=""):
        print(
            "{:<46} {}{}".format(name, "PASS" if ok else "FAIL", "  " + detail if detail else ""),
            flush=True,
        )
        if not ok:
            self.failures.append(name)

    @staticmethod
    def wait(cond, timeout=5.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            try:
                if cond():
                    return True
            except (TypeError, KeyError, AttributeError, StopIteration):
                pass
            time.sleep(0.02)
        return False

    def fresh(self, tab, cond, timeout=5.0):
        """`cond` on a snapshot taken after this call: an older one predates the action."""
        since = time.time()
        return self.wait(lambda: tab.snap["t"] > since and cond(), timeout)

    def state_is(self, tab, wanted, timeout=5.0):
        ok = self.fresh(tab, lambda: tab.snap["bridge"]["state"] == wanted, timeout)
        return ok, f"state '{tab.snap and tab.snap['bridge']['state']}'"

    def proc(self, tab, name):
        return next(p for p in tab.snap["processes"] if p["name"] == name)

    def run(self, wire):
        tab = Tab(self.address, self.token)
        viewer = Tab(self.address, self.token)
        try:
            self.check("starts DISARMED", *self.state_is(tab, "DISARMED"))
            self.check("arm needs control", tab.post("/api/arm", confirm=True)[0] == 409)
            self.check("take control", tab.post("/api/control/take")[1]["ok"])
            self.check("arm needs confirm", tab.post("/api/arm", confirm=False)[0] == 409)
            self.check("arm", tab.post("/api/arm", confirm=True)[1]["ok"])
            self.check("ARMED", *self.state_is(tab, "ARMED"))
            self.check(
                "/lowcmd at config gains",
                self.wait(lambda: abs(wire.last.motor_cmd[0].kp - self.kp[0]) < 1e-3),
            )

            # Processes and their gates.
            def marker_ok():
                requires = self.proc(tab, "streamer")["requires"]
                return next(r for r in requires if r["cond"] == "marker.ready")["ok"]

            self.check("streamer waits for marker.ready", self.wait(lambda: not marker_ok()))
            self.check("start marker", tab.post("/api/process/start", name="marker")[1]["ok"])
            self.check(
                "marker ready (log regex)",
                self.wait(lambda: self.proc(tab, "marker")["state"] == "ready"),
            )
            self.check(
                "start streamer (armed + marker.ready)",
                tab.post("/api/process/start", name="streamer", params={"mode": "slow"})[1]["ok"],
            )
            self.check(
                "arm refused while streamer runs", tab.post("/api/arm", confirm=True)[0] == 409
            )
            self.check("jog refused while streamer runs", tab.post("/api/jog/start")[0] == 409)
            self.check(
                "a viewer may stop a process",
                viewer.post("/api/process/stop", name="streamer")[1]["ok"],
            )
            self.check(
                "streamer stopped",
                self.fresh(tab, lambda: self.proc(tab, "streamer")["state"] == "stopped"),
            )

            # Jog one joint.
            status, reply = tab.post("/api/jog/start")
            self.check("jog start", reply.get("ok") is True, f"{status} {reply.get('message', '')}")
            self.check(
                "jog streams",
                self.wait(lambda: tab.snap["jog"]["active"] and tab.snap["jog"]["alive"]),
            )
            q0 = wire.last.motor_cmd[ELBOW].q
            tab.post("/api/jog/target", name="L_ELBOW", q=q0 + 0.2)
            self.check(
                "the elbow moves on /lowcmd",
                self.wait(lambda: abs(wire.last.motor_cmd[ELBOW].q - (q0 + 0.2)) < 0.02),
                f"q0 {q0:.3f}",
            )

            # Web e-stop.
            self.check("viewer E-STOP DAMP", viewer.post("/api/estop", mode="damp")[1]["ok"])
            self.check("LATCHED damp", *self.state_is(tab, "LATCHED damp service_damp"))
            self.check("damp on the wire", self.wait(lambda: wire.last.motor_cmd[0].kp == 0.0))
            self.check("jog ended", self.fresh(tab, lambda: not tab.snap["jog"]["active"]))
            status, reply = tab.post("/api/arm", confirm=True)
            self.check(
                "arm refused while latched", reply.get("ok") is False, reply.get("message", "")
            )
            self.check("reset", tab.post("/api/reset")[1]["ok"])
            self.check("STOPPED damp", *self.state_is(tab, "STOPPED damp service_damp"))

            # The jog deadman: the controlling tab vanishes mid-jog.
            self.check("re-arm after reset", tab.post("/api/arm", confirm=True)[1]["ok"])
            self.check("ARMED again", *self.state_is(tab, "ARMED"))
            time.sleep(2.2)  # the arming ramp
            status, reply = tab.post("/api/jog/start")
            self.check("jog start again", reply.get("ok") is True, reply.get("message", ""))
            self.check("jog streams again", self.wait(lambda: tab.snap["jog"]["alive"]))
            before = wire.robot_cmds
            time.sleep(0.5)
            flowed = wire.robot_cmds - before
            # Paced, not flooded: motor_probe's loop once sent ~200 here.
            self.check("/robot_cmd paced at <= 50 Hz", 10 <= flowed <= 32, f"{flowed} in 0.5 s")
            tab.close()
            self.check(
                "tab closed mid-jog: cmd_timeout hold",
                *self.state_is(viewer, "LATCHED hold cmd_timeout"),
            )
            self.check("control freed", self.wait(lambda: not viewer.snap["control"]["held"]))

            # The remote, seen through the console.
            viewer.post("/api/control/take")
            viewer.post("/api/reset")
            wire.keys.publish(UInt16(data=KEY_L1 | KEY_R1))
            time.sleep(0.2)
            wire.keys.publish(UInt16(data=0))
            self.check(
                "remote L1+R1 shows on the page", *self.state_is(viewer, "LATCHED damp remote_damp")
            )
        finally:
            tab.close()
            viewer.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", type=int)
    parser.add_argument("token_file")
    parser.add_argument("params")
    args = parser.parse_args()

    with open(args.params) as handle:
        params = yaml.safe_load(handle)["robot_bridge"]["ros__parameters"]
    kp = [float(params[n]["kp"]) for n in params["joint_names"]]
    token = open(args.token_file).read().strip()

    rclpy.init()
    node = rclpy.create_node("console_driver")
    wire = Wire(node)
    # Its own executor: rclpy.spin on a thread segfaults when the node is destroyed under it.
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    while wire.keys.get_subscription_count() == 0:
        time.sleep(0.05)

    driver = Driver(("127.0.0.1", args.port), token, kp)
    try:
        driver.run(wire)
    finally:
        executor.shutdown()
        spinner.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()
    if driver.failures:
        print("\n{} check(s) failed: {}".format(len(driver.failures), ", ".join(driver.failures)))
        sys.exit(1)
    print("\nall checks passed")


if __name__ == "__main__":
    main()
