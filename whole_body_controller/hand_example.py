"""Send one pair of normalized hand targets; never command the body."""

import argparse
import math
import time

import rclpy

from .h1_2_controller import H1_2WholeBodyController


def position(value):
    value = float(value)
    if not math.isfinite(value) or not 0 <= value <= 1:
        raise argparse.ArgumentTypeError("hand positions must be finite values in [0, 1]")
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--left", type=position, nargs=6, required=True,
                        help="little, ring, middle, index, thumb bend, thumb rotation")
    parser.add_argument("--right", type=position, nargs=6, required=True,
                        help="little, ring, middle, index, thumb bend, thumb rotation")
    args = parser.parse_args()

    rclpy.init(args=[])
    node = rclpy.create_node("h1_2_hand_example")
    try:
        controller = H1_2WholeBodyController(node)
        # Allow DDS discovery before a one-shot publication; otherwise it can be lost.
        deadline = time.monotonic() + 5.0
        while node.count_subscribers("/inspire/cmd") == 0:
            if time.monotonic() >= deadline:
                raise RuntimeError("No subscriber on /inspire/cmd; no command sent.")
            rclpy.spin_once(node, timeout_sec=0.1)

        controller.send_hand_cmd(left=args.left, right=args.right)
        print(f"Published once: left={args.left}, right={args.right}")
        print("No body command sent. Publication is not motion completion; exiting does not stop the hands.")
        # Keep the publisher alive briefly to allow transport, not to wait for motion.
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
