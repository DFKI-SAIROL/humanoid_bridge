import rclpy
from robot_bridge_py.robot_client import RobotClient


class RobotController:
    def __init__(self, node):
        self.node = node
        self.num_dof = 30
        self.robot = RobotClient(node=self.node, robot_type="X2", num_dof=self.num_dof, control_frequency=50.0, interpolation_order=0.0)
        self.timer = self.node.create_timer(1.0, self.step)  # 1 Hz state display
        print("Read-only X2 example. No control commands are sent.")
        print("Displayed values may be initial zeros or stale data; state freshness is not checked.")

    def step(self):
        print("Joint positions:", self.robot.q_pos)
        print("Joint velocities:", self.robot.q_vel)
        print("Joint torques:", self.robot.tau)
        print("Quaternion (w, x, y, z):", self.robot.quat)
        print("Angular velocity:", self.robot.angular_velocity)


if __name__ == "__main__":
    rclpy.init()
    node = rclpy.create_node('robot_client_node')
    controller = RobotController(node)

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
