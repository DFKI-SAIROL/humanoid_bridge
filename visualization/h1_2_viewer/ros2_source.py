"""Optional ROS 2 transport. Imported only for live mode; subscriptions only."""
import sys
import time
from threading import Lock, Thread
from contextlib import nullcontext
import rclpy
from rclpy.node import Node

from .pointcloud import cloud_xyz
from .telemetry import unitree_body, named_body


class DashboardNode(Node):
    def __init__(self, dashboard, profile):
        super().__init__('h1_2_observation_dashboard', start_parameter_services=False, enable_rosout=False)
        self.dashboard = dashboard
        self.profile = profile
        self.pending = {}
        self.data_lock = Lock()
        self.subscriptions_ = []
        source = profile.settings['source']
        if source['type'] == 'unitree':
            try:
                from unitree_hg.msg import LowState
            except ImportError as exc:
                raise RuntimeError('Unitree live mode requires built/sourced unitree_hg messages. Offline demo does not need ROS; standard JointState mode does not need Unitree body messages.') from exc
            body_type = LowState
        else:
            from sensor_msgs.msg import JointState
            body_type = JointState
        self.subscribe(source['state_topic'], body_type, 'body')
        if source['lidar_topic']:
            from sensor_msgs.msg import PointCloud2
            self.subscribe(source['lidar_topic'], PointCloud2, 'lidar')
        if source['hand_topic']:
            try:
                from unitree_go.msg import MotorStates
            except ImportError as exc:
                raise RuntimeError('Inspire feedback requires unitree_go messages; set source.hand_topic: null to disable this optional stream.') from exc
            self.subscribe(source['hand_topic'], MotorStates, 'hands')
        from .spatial_ros import attach
        attach(self, dashboard, profile)
        from .head_camera_ros import attach as attach_head
        attach_head(self, dashboard, profile)
        from .native_video import attach as attach_video
        attach_video(self, profile)
        from .client_observer import attach
        attach(self, profile)

    def subscribe(self, topic, msg_type, stream):
        def receive(msg):
            with self.data_lock:
                self.pending[stream] = msg
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        self.subscriptions_.append(self.create_subscription(
            msg_type, topic, receive,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)))

    def render_pending(self):
        with self.data_lock:
            pending, self.pending = self.pending, {}
        self.client_pending = {}
        for stream, msg in pending.items():
            if stream in ('body_target', 'hand_target'):
                self.client_pending[stream] = msg
                continue
            try:
                if stream == 'body':
                    decode = unitree_body if self.profile.settings['source']['type'] == 'unitree' else named_body
                    self.dashboard.body(*decode(msg, self.profile.joints))
                    if self.profile.settings['source']['type'] == 'unitree' and self.profile.settings['attitude']['enabled']:
                        from .attitude import unitree_attitude
                        try:
                            self.dashboard.attitude(*unitree_attitude(msg))
                            self.dashboard.errors.pop('imu', None)
                        except (ValueError, TypeError) as exc:
                            self.dashboard.errors['imu'] = str(exc)
                elif stream == 'lidar':
                    self.dashboard.cloud(cloud_xyz(msg), msg.header.frame_id)
                elif stream == 'head_rgb':
                    self.dashboard.head_rgb(msg)
                    self.dashboard.errors.pop('head_video', None)
                elif stream == 'head_video_error':
                    self.dashboard.errors['head_video'] = msg
                else:
                    self.dashboard.hands([m.q for m in msg.states])
            except (ValueError, TypeError, BufferError) as exc:
                self.dashboard.errors[stream] = str(exc)
                print(f'Rejected {stream}: {exc}', file=sys.stderr)
        from .spatial_ros import drain
        drain(self, self.dashboard, self.profile)
        from .head_camera_ros import drain as drain_head
        drain_head(self, self.dashboard, self.profile)
        from .client_observer import drain as drain_client
        drain_client(self, self.dashboard, self.profile)


def run(dashboard, profile, ros_args):
    node = None
    worker = None
    failures = []
    rclpy.init(args=ros_args)
    try:
        node = DashboardNode(dashboard, profile)
        def receive():
            try:
                rclpy.spin(node)
            except rclpy.executors.ExternalShutdownException:
                pass
            except Exception as exc:
                # Signal-driven shutdown may invalidate the wait set before spin returns.
                if rclpy.ok():
                    failures.append(exc)
        # DDS reception continues replacing latest samples even when Rerun applies backpressure.
        worker = Thread(target=receive, name='h1-2-read-only-dds', daemon=True)
        worker.start()
        interval = 1 / profile.settings['runtime']['hz']
        next_status = 0
        while rclpy.ok() and not failures:
            started = time.monotonic()
            batch = getattr(getattr(dashboard, 'spatial', None), 'batch', nullcontext)
            with batch():
                node.render_pending()
            if started >= next_status:
                dashboard.status()
                next_status = started + 1
            time.sleep(max(0, interval - (time.monotonic() - started)))
        if failures:
            raise failures[0]
    finally:
        rclpy.try_shutdown()
        if worker is not None:
            worker.join(timeout=3)
        if node is not None:
            node.destroy_node()
