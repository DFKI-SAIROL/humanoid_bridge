"""Exercise the real X2 executable against a loopback-only fake robot.

Run after sourcing Humble and the X2 installation:
  python3 robot_bridge/tests/test_x2_rearm.py /path/to/X2_bridge
No hardware discovery: a dedicated domain and UDP loopback whitelist are forced.
"""
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest

import yaml

EXECUTABLE = os.environ.get("X2_BRIDGE_TEST_EXECUTABLE")


def configure_loopback():
    profile_dir = tempfile.TemporaryDirectory(prefix="x2-rearm-test-")
    profile = Path(profile_dir.name) / "dds.xml"
    profile.write_text('''<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <transport_descriptors><transport_descriptor><transport_id>loopback</transport_id>
    <type>UDPv4</type><interfaceWhiteList><address>127.0.0.1</address></interfaceWhiteList>
    </transport_descriptor></transport_descriptors>
    <participant profile_name="test" is_default_profile="true"><rtps>
    <userTransports><transport_id>loopback</transport_id></userTransports>
    <useBuiltinTransports>false</useBuiltinTransports></rtps></participant></profiles>''')
    for key in ("ROS_DISCOVERY_SERVER", "FASTDDS_DEFAULT_PROFILES_FILE", "RMW_FASTRTPS_USE_QOS_FROM_XML"):
        os.environ.pop(key, None)
    os.environ.update(ROS_DOMAIN_ID="198", ROS_LOCALHOST_ONLY="1",
                      RMW_IMPLEMENTATION="rmw_fastrtps_cpp",
                      FASTRTPS_DEFAULT_PROFILES_FILE=str(profile))
    return profile_dir

import rclpy
from rclpy.qos import qos_profile_sensor_data
from aimdk_msgs.msg import JointState, JointStateArray, JointCommandArray
from aimdk_msgs.srv import GetSystemState
from bridge_interface.srv import SetDefaultPosition
from bridge_interface.msg import RobotCmd, MotorCmd
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger


class RearmTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not EXECUTABLE:
            raise unittest.SkipTest("Pass an X2 bridge executable to run the loopback integration test")
        cls.saved_env = os.environ.copy()
        cls.profile_dir = configure_loopback()

    @classmethod
    def tearDownClass(cls):
        os.environ.clear()
        os.environ.update(cls.saved_env)
        cls.profile_dir.cleanup()

    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node("fake_x2")
        self.config_path = Path(__file__).resolve().parents[1] / "params/X2_config.yaml"
        self.config = yaml.safe_load(self.config_path.read_text())["robot_bridge"]["ros__parameters"]
        self.q = list(self.config["ready_q"])
        self.feedback = True
        self.commands = []
        self.groups = {}
        for group, start, end in (("leg", 0, 12), ("waist", 12, 15), ("arm", 15, 29), ("head", 29, 30)):
            pub = self.node.create_publisher(JointStateArray, f"/aima/hal/joint/{group}/state", qos_profile_sensor_data)
            self.groups[group] = (pub, start, end)
        self.imu_pub = self.node.create_publisher(Imu, "/aima/hal/imu/torso/state", qos_profile_sensor_data)
        self.sub = self.node.create_subscription(JointCommandArray, "/aima/hal/joint/leg/command",
                                                 self.commands.append, qos_profile_sensor_data)
        self.server = self.node.create_service(GetSystemState, "/aimdk_5Fmsgs/srv/GetSystemState", self.mode)
        self.timer = self.node.create_timer(0.01, self.publish)
        self.start = self.node.create_client(SetDefaultPosition, "/start_control")
        self.stop = self.node.create_client(Trigger, "/stop_control")
        self.log = tempfile.TemporaryFile(mode="w+")
        self.process = subprocess.Popen([EXECUTABLE, "--ros-args", "--params-file", str(self.config_path)],
                                        stdout=self.log, stderr=subprocess.STDOUT)
        self.addCleanup(self.cleanup_bridge)
        self.spin_until(lambda: self.node.count_publishers("/aima/hal/joint/leg/command") > 0, 8)
        self.spin(0.3)

    def cleanup_bridge(self):
        self.process.send_signal(signal.SIGINT)
        try:
            self.process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait()
        self.log.seek(0)
        print(self.log.read()[-4000:])
        self.log.close()
        self.node.destroy_node()
        rclpy.shutdown()

    def mode(self, request, response):
        response.header.status.value = 1
        response.cur_state = "Develop_MC"
        response.curr_status.value = 1
        return response

    def publish(self):
        if not self.feedback:
            return
        for group, (pub, start, end) in self.groups.items():
            msg = JointStateArray()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.joints = [JointState(name=self.config["joint_names"][i], position=self.q[i]) for i in range(start, end)]
            if group == "head":
                msg.joints.append(JointState(name="head_pitch_joint"))
            pub.publish(msg)
        imu = Imu()
        imu.orientation.w = 1.0
        imu.header.stamp = self.node.get_clock().now().to_msg()
        self.imu_pub.publish(imu)

    def spin_until(self, predicate, timeout=3):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.01)
        self.assertTrue(predicate(), "Timed out waiting for fake-robot interaction")

    def spin(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def call(self, client, request):
        future = client.call_async(request)
        self.spin_until(future.done)
        return future.result()

    def test_stop_then_explicit_start(self):
        request = SetDefaultPosition.Request(default_position=self.q)
        self.assertTrue(self.call(self.start, request).success)
        self.spin(0.1)
        self.assertTrue(self.call(self.stop, Trigger.Request()).success)
        self.spin(0.1)
        self.assertTrue(self.commands)
        self.assertTrue(all(j.stiffness == 0 for j in self.commands[-1].joints))
        # Reproduce the reported failure on the same running bridge process.
        reply = self.call(self.start, request)
        self.assertTrue(reply.success, reply.message)
        self.spin(0.1)
        self.assertTrue(any(j.stiffness > 0 for j in self.commands[-1].joints))

    def test_rearm_uses_measured_pose_and_discards_old_commands(self):
        self.assertTrue(self.call(self.start, SetDefaultPosition.Request(default_position=self.q)).success)
        self.spin(0.1)
        self.assertTrue(self.call(self.stop, Trigger.Request()).success)
        self.q[0] = -0.3  # Robot moved under damping.
        pub = self.node.create_publisher(RobotCmd, "/robot_cmd", 1)
        self.spin(0.2)
        cmd = RobotCmd(duration=0.02, interpolation_order=0.0, hold_position=True)
        cmd.motor_cmd = [MotorCmd(q=0.1, kp=20.0, kd=1.0, dq=0.5, tau=1.0) for _ in self.q]
        for _ in range(5):
            pub.publish(cmd)
            self.spin(0.02)
        self.assertTrue(all(j.stiffness == 0 for j in self.commands[-1].joints))
        self.commands.clear()
        target = list(self.config["ready_q"])
        reply = self.call(self.start, SetDefaultPosition.Request(default_position=target))
        self.assertTrue(reply.success, reply.message)
        self.spin_until(lambda: any(m.joints[0].stiffness > 0 for m in self.commands))
        first = next(m.joints[0] for m in self.commands if m.joints[0].stiffness > 0)
        self.assertAlmostEqual(first.position, self.q[0], delta=0.03)
        self.assertEqual(first.velocity, 0)
        self.assertEqual(first.effort, 0)
        for _ in range(3):
            self.assertTrue(self.call(self.stop, Trigger.Request()).success)
            self.assertTrue(self.call(self.start, SetDefaultPosition.Request(default_position=target)).success)

    def test_rejected_rearm_keeps_damping(self):
        self.assertTrue(self.call(self.stop, Trigger.Request()).success)
        bad_targets = ([0.0], [float("nan")] * 30, [100.0] * 30)
        for target in bad_targets:
            reply = self.call(self.start, SetDefaultPosition.Request(default_position=target))
            self.assertFalse(reply.success)
        reply = self.call(self.start, SetDefaultPosition.Request(default_position=self.q, default_kp=[999.0] * 30))
        self.assertFalse(reply.success)
        self.feedback = False
        self.spin(0.3)
        self.assertFalse(self.call(self.start, SetDefaultPosition.Request(default_position=self.q)).success)
        self.spin(0.05)
        self.assertTrue(all(j.stiffness == 0 for j in self.commands[-1].joints))
        self.feedback = True
        self.spin(0.1)
        # Fresh feedback alone cannot restart motion.
        self.assertTrue(all(j.stiffness == 0 for j in self.commands[-1].joints))
        self.assertTrue(self.call(self.start, SetDefaultPosition.Request(default_position=self.q)).success)

    def test_watchdog_stop_requires_new_start(self):
        self.assertTrue(self.call(self.start, SetDefaultPosition.Request(default_position=self.q)).success)
        self.feedback = False
        self.spin(0.4)
        self.assertTrue(all(j.stiffness == 0 for j in self.commands[-1].joints))
        self.feedback = True
        self.spin(0.1)
        self.assertTrue(all(j.stiffness == 0 for j in self.commands[-1].joints))
        self.assertTrue(self.call(self.start, SetDefaultPosition.Request(default_position=self.q)).success)

    def test_conflicting_hal_publisher_cannot_rearm(self):
        self.assertTrue(self.call(self.stop, Trigger.Request()).success)
        other = self.node.create_publisher(JointCommandArray, "/aima/hal/joint/leg/command", qos_profile_sensor_data)
        self.spin(0.3)
        self.assertFalse(self.call(self.start, SetDefaultPosition.Request(default_position=self.q)).success)
        self.node.destroy_publisher(other)


if __name__ == "__main__":
    if len(sys.argv) > 1 and not sys.argv[1].startswith("-"):
        EXECUTABLE = sys.argv.pop(1)
    unittest.main()
