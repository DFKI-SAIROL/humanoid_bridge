"""Bounded timestamp pairing for already registered RGB/depth messages; no ROS imports."""


class RgbdPairer:
    def __init__(self, tolerance_ms=0.0, capacity=4):
        self.tolerance_ns = round(tolerance_ms * 1_000_000)
        self.capacity = capacity
        self.queues = {'rgb': [], 'depth': []}
        self.consumed = {'rgb': -1, 'depth': -1}
        self.matched = 0
        self.last_delta_ns = None

    def add(self, kind, message):
        stamp = message.header.stamp
        timestamp = stamp.sec * 1_000_000_000 + stamp.nanosec
        if timestamp <= self.consumed[kind]:
            return None
        queue = self.queues[kind]
        queue.append((timestamp, message))
        queue.sort(key=lambda item: item[0])
        del queue[:-self.capacity]
        # Prefer the newest complete pair; never reuse an already consumed image.
        for depth_stamp, depth in reversed(self.queues['depth']):
            candidates = [(abs(rgb_stamp-depth_stamp), rgb_stamp, rgb)
                          for rgb_stamp, rgb in self.queues['rgb']
                          if rgb.header.frame_id == depth.header.frame_id
                          and (rgb.width, rgb.height) == (depth.width, depth.height)
                          and abs(rgb_stamp-depth_stamp) <= self.tolerance_ns]
            if not candidates:
                continue
            delta, rgb_stamp, rgb = min(candidates, key=lambda item: (item[0], -item[1]))
            for stream, consumed in (('rgb', rgb_stamp), ('depth', depth_stamp)):
                self.consumed[stream] = consumed
                self.queues[stream] = [(s, m) for s, m in self.queues[stream] if s > consumed]
            self.matched += 1
            self.last_delta_ns = delta
            return depth, rgb
        return None
