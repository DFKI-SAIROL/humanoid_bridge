"""Read-only Unitree front video framing and H.264 decoding, independent of ROS/Rerun."""
import struct
import time


def video_payload(serialized, height=360):
    """CDR1: observed H1 firmware (timestamp,height,bytes), or public three-array IDL.

    The current firmware advertises Go2FrontVideoData but does not match its public
    three-array schema. Consume raw bytes, never deserialize it with that schema.
    """
    data = memoryview(serialized)
    if len(data) < 20 or bytes(data[:2]) not in (b'\x00\x00', b'\x00\x01'):
        raise ValueError('Unsupported/truncated front video CDR')
    endian = '<' if data[1] == 1 else '>'
    observed_height, size = struct.unpack_from(endian + 'II', data, 12)
    if observed_height in (180, 360, 720) and 20 + size <= len(data) <= 23 + size:
        return bytes(data[20:20 + size]) if observed_height == height else None
    # Public Go2FrontVideoData: timestamp + video720p/360p/180p byte sequences.
    offset, selected = 12, None
    for stream_height in (720, 360, 180):
        offset = 4 + ((offset - 4 + 3) // 4) * 4
        if offset + 4 > len(data):
            raise ValueError('Truncated front video sequence header')
        size = struct.unpack_from(endian + 'I', data, offset)[0]
        offset += 4
        if offset + size > len(data):
            raise ValueError('Truncated front video sequence')
        if stream_height == height and size:
            selected = bytes(data[offset:offset + size])
        offset += size
    return selected


class FrontVideoDecoder:
    def __init__(self, *, height=360, hz=5., max_width=640):
        try:
            import av
        except ImportError as exc:
            raise RuntimeError('Native head video needs PyAV: install the viewer with pip install -e ".[video]"') from exc
        self.av = av
        self.codec = av.CodecContext.create('h264', 'r')
        self.codec.thread_count = 1
        self.height, self.hz, self.max_width = height, hz, max_width
        self.last_output = -float('inf')
        self.frames = 0
        self.decode_errors = 0
        self.keyframes = 0
        self.corrupt_frames = 0

    def receive(self, serialized):
        payload = video_payload(serialized, self.height)
        if not payload:
            return None
        latest = None
        try:
            packets = self.codec.parse(payload)
        except self.av.FFmpegError:
            self.decode_errors += 1
            return None
        for packet in packets:
            try:
                # Decode every packet; dropping P-frames before decoding breaks H.264.
                frames = self.codec.decode(packet)
            except self.av.FFmpegError:
                # Joining mid-GOP can fail until the next SPS/keyframe arrives.
                self.decode_errors += 1
                continue
            for image in frames:
                if image.is_corrupt:
                    self.corrupt_frames += 1
                    continue
                self.frames += 1
                self.keyframes += int(image.key_frame)
                now = time.monotonic()
                if now - self.last_output < 1 / self.hz:
                    continue
                if image.width > self.max_width:
                    width = self.max_width
                    image = image.reformat(width=width, height=max(1, round(image.height * width / image.width)))
                latest = image.to_ndarray(format='rgb24')
                self.last_output = now
        return latest


def attach(node, profile):
    settings = profile.settings['head_camera']
    if not settings['video_topic']:
        return
    from unitree_go.msg import Go2FrontVideoData
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    decoder = FrontVideoDecoder(height=settings['video_height'], hz=settings['rgb_hz'],
                                max_width=settings['max_rgb_width'])
    node.front_video_decoder = decoder
    def receive(serialized):
        try:
            rgb = decoder.receive(serialized)
            if rgb is not None:
                with node.data_lock:
                    node.pending['head_rgb'] = rgb
        except ValueError as exc:
            with node.data_lock:
                node.pending['head_video_error'] = str(exc)
    node.subscriptions_.append(node.create_subscription(
        Go2FrontVideoData, settings['video_topic'], receive,
        QoSProfile(depth=30, reliability=ReliabilityPolicy.BEST_EFFORT), raw=True))
