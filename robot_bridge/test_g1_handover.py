#!/usr/bin/env python3
"""Offline regression: python3 test_g1_handover.py /path/to/G1_bridge.

Uses a separate ROS domain and loopback only; never connects to the robot.
"""
import os
import pathlib
import signal
import subprocess
import sys
import tempfile
import time

os.environ.update(ROS_DOMAIN_ID="181", ROS_LOCALHOST_ONLY="0",
                  RMW_IMPLEMENTATION="rmw_cyclonedds_cpp",
                  CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>'
                  '<NetworkInterface name="lo"/></Interfaces></General></Domain></CycloneDDS>')

import rclpy
from unitree_api.msg import Request, Response
from unitree_hg.msg import LowCmd, LowState

rclpy.init()
node = rclpy.create_node("fake_g1")
responses = node.create_publisher(Response, "/api/motion_switcher/response", 10)
commands = node.create_publisher(LowCmd, "/lowcmd", 10)
states = node.create_publisher(LowState, "/lowstate", 10)
reply = {"data": '{"name":"ai"}', "code": 0, "wrong_id": False}

def respond(request):
    assert request.header.identity.api_id == 1001, "Bridge must not change robot mode"
    if reply["data"] is None:
        return
    response = Response()
    response.header.identity = request.header.identity
    if reply["wrong_id"]:
        response.header.identity.id += 1
    response.header.status.code = reply["code"]
    response.data = reply["data"]
    responses.publish(response)

subscription = node.create_subscription(Request, "/api/motion_switcher/request", respond, 10)
config = pathlib.Path(__file__).parent / "params/G1_config.yaml"
marker = "startup handover verified"

def pump(seconds, traffic=False):
    until = time.monotonic() + seconds
    while time.monotonic() < until:
        rclpy.spin_once(node, timeout_sec=0.02)
        if traffic:
            commands.publish(LowCmd())

def output(log):
    log.seek(0)
    return log.read().decode(errors="replace")

def launch(log):
    return subprocess.Popen([sys.argv[1], "--ros-args", "--params-file", str(config)],
                            stdout=log, stderr=subprocess.STDOUT)

def interrupt(process, log):
    process.send_signal(signal.SIGINT)
    assert process.wait(timeout=5) == 0, output(log)
    assert "Aborted" not in output(log) and "terminate called" not in output(log), output(log)

process = None
try:
    with tempfile.TemporaryFile() as log:
        process = launch(log)
        for data, code, wrong_id in [('{"name":"ai"}', 0, False),
                                     ('invalid json', 0, False),
                                     ('{}', 0, False),
                                     ('{"name":""}', 1, False),
                                     ('{"name":""}', 0, True)]:
            reply.update(data=data, code=code, wrong_id=wrong_id)
            pump(3.5)
            assert process.poll() is None and marker not in output(log), output(log)
        reply.update(data='{"name":""}', code=0, wrong_id=False)
        pump(4, traffic=True)
        assert marker not in output(log), output(log)
        # Keep the publisher registered, but stop samples: the original bug.
        pump(4)
        assert marker in output(log), output(log)
        interrupt(process, log)
        print("PASS: active/invalid/failed/unmatched modes and command traffic block; silent writer passes")
    reply["data"] = None
    with tempfile.TemporaryFile() as log:
        process = launch(log)
        pump(4)
        assert marker not in output(log), output(log)
        interrupt(process, log)
        print("PASS: missing response blocks and Ctrl+C exits cleanly during handover")
    node.destroy_publisher(states)
    reply["data"] = '{"name":""}'
    with tempfile.TemporaryFile() as log:
        process = launch(log)
        pump(4)
        assert marker in output(log), output(log)
        interrupt(process, log)
        print("PASS: Ctrl+C exits cleanly while waiting for /lowstate")
finally:
    if process is not None and process.poll() is None:
        process.kill()
        process.wait()
    node.destroy_node()
    rclpy.shutdown()
