"""What the Unitree robots (G1, Go2) share in sim: the remote's bytes, the motion switcher's ROS side."""

from robot_sim.motion import BOOT_MODE, MotionSwitcher

REMOTE_HEADER = (0x55, 0x51)  # wireless_remote bytes 0-1 with a remote; all 0 without one
# Key bits in wireless_remote bytes 2-3, as the bridge's estop.hpp names them.
KEYS = ("R1", "L1", "START", "SELECT", "R2", "L2", "F1", "F2",
        "A", "B", "X", "Y", "UP", "RIGHT", "DOWN", "LEFT")  # fmt: skip


def key_mask(names):
    """Button names -> the key bits; ValueError on an unknown name."""
    mask = 0
    for name in names:
        if name.upper() not in KEYS:
            raise ValueError(f"button {name!r}: expected one of {KEYS}")
        mask |= 1 << KEYS.index(name.upper())
    return mask


def set_remote(wireless_remote, connected):
    """Header bytes on, or every byte 0 as the robot sends without a remote."""
    wireless_remote[0], wireless_remote[1] = REMOTE_HEADER if connected else (0, 0)


def set_keys(wireless_remote, mask):
    wireless_remote[2] = mask & 0xFF  # little-endian
    wireless_remote[3] = mask >> 8


class MotionService:
    """The fake motion service: answers /api/motion_switcher/*; while a mode holds, it owns the topic.

    `frame`: the LowCmd it sends, every motor mode 0. Lock held by the caller for `boot` and `send`.
    """

    PERIOD_S = 0.01  # its /lowcmd, 100 Hz

    def __init__(self, node, lowcmd_type, topic, frame, lock):
        from unitree_api.msg import Request, Response

        self.node = node
        self.lowcmd_type = lowcmd_type
        self.topic = topic
        self.frame = frame
        self.lock = lock
        self.switcher = MotionSwitcher()
        self.publisher = None  # guarded by lock: the physics thread publishes
        self.due = 0.0
        self.Response = Response
        node.create_subscription(Request, "/api/motion_switcher/request", self._on_request, 10)
        self.response = node.create_publisher(Response, "/api/motion_switcher/response", 10)
        node.create_timer(0.1, self.sync)  # the publisher follows the mode, on the executor thread

    def _on_request(self, msg):
        with self.lock:
            code, data = self.switcher.call(msg.header.identity.api_id, msg.parameter)
        response = self.Response()
        response.header.identity = msg.header.identity
        response.header.status.code = code
        response.data = data
        self.response.publish(response)
        self.node.get_logger().info(f"motion switcher: api {msg.header.identity.api_id} -> {code} {data}")
        self.sync()

    def sync(self):
        """A publisher while a mode holds the topic, none once released."""
        with self.lock:
            if self.switcher.mode and self.publisher is None:
                self.publisher = self.node.create_publisher(self.lowcmd_type, self.topic, 10)
                self.node.get_logger().warn(f"motion service '{self.switcher.mode}' holds {self.topic}")
            elif not self.switcher.mode and self.publisher is not None:
                self.node.destroy_publisher(self.publisher)
                self.publisher = None
                self.node.get_logger().info(f"motion service released {self.topic}")

    def boot(self):
        """A rebooted robot: its motion service holds the topic again."""
        self.switcher.mode = BOOT_MODE

    def send(self, now):
        """Its frame while it holds the topic, at PERIOD_S."""
        if self.publisher is not None and now >= self.due:
            self.due = now + self.PERIOD_S
            self.publisher.publish(self.frame)
