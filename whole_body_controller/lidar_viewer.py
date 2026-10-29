"""Display /livox/lidar in Rerun; run with the bridge's ROS2 Python."""

import argparse
from pathlib import Path
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
import rerun as rr


from h1_2_viewer.pointcloud import cloud_xyz


class LidarViewer(Node):
    def __init__(self, topic):
        super().__init__("h1_2_lidar_viewer")
        self.frames = 0
        self.subscription = self.create_subscription(
            PointCloud2, topic, self.on_cloud, qos_profile_sensor_data)
        self.get_logger().info(f"Waiting for PointCloud2 on {topic}")
        self.wait_timer = self.create_timer(10.0, self.report_wait)

    def report_wait(self):
        if self.frames == 0:
            self.get_logger().warning(
                "No point clouds received yet. Check the PC4 driver and "
                "ROS_DOMAIN_ID / DDS network connectivity.")

    def on_cloud(self, message):
        try:
            xyz = cloud_xyz(message)
        except (ValueError, TypeError) as exc:
            self.get_logger().error(f"Cannot decode point cloud: {exc}")
            return
        rr.set_time("frame", sequence=self.frames)
        rr.log("lidar/points", rr.Points3D(xyz, radii=0.015))
        if self.frames == 0:
            self.get_logger().info(
                f"Received {len(xyz)} points; frame_id={message.header.frame_id}. "
                "Displaying raw sensor coordinates, without TF or mapping.")
            self.wait_timer.cancel()
        self.frames += 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/livox/lidar")
    args, ros_args = parser.parse_known_args()
    rr.init("h1_2_lidar")
    rr.spawn(executable_path=str(Path(sys.executable).parent / "rerun"),
             memory_limit="512MiB", server_memory_limit="512MiB")
    rclpy.init(args=ros_args)
    node = None
    try:
        node = LidarViewer(args.topic)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        rr.disconnect()


if __name__ == "__main__":
    main()
