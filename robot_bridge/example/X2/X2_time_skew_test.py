import argparse
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from aimdk_msgs.msg import JointStateArray
import yaml


class TimeSkewTest:
    def __init__(self, node, config):
        self.node = node
        self.samples = []
        self.latest = {}
        self.skipped = dict(missing=0, invalid=0, stale=0)
        self.subscriptions = []
        self.groups = ['leg', 'waist', 'arm']
        if config['enable_head']:
            self.groups.append('head')
        if config['include_imu_in_time_check']:
            self.groups.append('imu')
        self.timeouts = {group: config[f'{group}_state_timeout'] for group in self.groups}
        qos = type(qos_profile_sensor_data)(depth=1, reliability=qos_profile_sensor_data.reliability,
                                          durability=qos_profile_sensor_data.durability)
        for group in self.groups:
            topic = '/aima/hal/imu/torso/state' if group == 'imu' else f'/aima/hal/joint/{group}/state'
            message_type = Imu if group == 'imu' else JointStateArray
            self.subscriptions.append(node.create_subscription(
                message_type, topic, lambda msg, group=group: self.receive(group, msg), qos))
        self.timer = node.create_timer(config['control_dt'], self.sample)
        print('Read-only header.stamp comparison; no robot commands or mode changes.', flush=True)
        print(f'Groups: {self.groups}; sampling period: {config["control_dt"]} s', flush=True)
        print('Clock comparability and message payload validity are not established by this test.', flush=True)

    def receive(self, group, msg):
        stamp = msg.header.stamp
        valid = stamp.sec >= 0 and 0 <= stamp.nanosec < 1000000000 and (stamp.sec != 0 or stamp.nanosec != 0)
        self.latest[group] = (stamp.sec * 1000000000 + stamp.nanosec, time.monotonic(), valid)

    def sample(self):
        if any(group not in self.latest for group in self.groups):
            self.skipped['missing'] += 1
            return
        values = [self.latest[group] for group in self.groups]
        if not all(value[2] for value in values):
            self.skipped['invalid'] += 1
            return
        now = time.monotonic()
        if any(now - self.latest[group][1] > self.timeouts[group] for group in self.groups):
            self.skipped['stale'] += 1
            return
        stamps = [value[0] for value in values]
        self.samples.append(max(stamps) - min(stamps))

    def report(self):
        print(f'\nValid comparison samples: {len(self.samples)}; skipped ticks: {self.skipped}')
        missing = [group for group in self.groups if group not in self.latest]
        if missing:
            print(f'No messages received from: {missing}')
        if not self.samples:
            print('No usable comparisons. Do not choose a threshold from this run.')
            return
        ns = np.asarray(self.samples, dtype=np.int64)
        ms = ns / 1000000.0
        print(f'Skew (ms): min={ms.min():.3f}, mean={ms.mean():.3f}, '
              f'P95={np.percentile(ms, 95):.3f}, P99={np.percentile(ms, 99):.3f}, max={ms.max():.3f}')
        for threshold in (5, 10, 20, 50):
            count = int(np.count_nonzero(ns > threshold * 1000000))
            print(f'Above {threshold:2d} ms: {count}/{len(ns)} ({100 * count / len(ns):.3f}%)')
        print('These are sampled latest-message timestamp spreads, not network latency or packet-loss rates.')
        print('A candidate threshold is not a validated safety limit; repeat under normal workload.')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Read-only X2 component timestamp skew measurement.')
    parser.add_argument('--duration', type=float, default=60.0, help='Measurement duration in seconds (default: 60).')
    parser.add_argument('--config', type=Path, default=Path(__file__).resolve().parents[2] / 'params/X2_config.yaml')
    args = parser.parse_args()
    if not np.isfinite(args.duration) or args.duration <= 0:
        parser.error('--duration must be finite and positive')
    with open(args.config) as file:
        config = yaml.safe_load(file)['robot_bridge']['ros__parameters']
    rclpy.init()
    node = rclpy.create_node('x2_time_skew_test')
    test = TimeSkewTest(node, config)
    deadline = time.monotonic() + args.duration
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
    except KeyboardInterrupt:
        pass
    finally:
        test.report()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
