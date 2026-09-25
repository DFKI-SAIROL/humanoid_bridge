import rclpy
from robot_bridge_py.robot_client import RobotClient


class RobotController:
    def __init__(self, node):
        self.node = node
        self.num_dof = 30
        self.robot = RobotClient(node=self.node, robot_type="X2", num_dof=self.num_dof, control_frequency=50.0, interpolation_order=0.0, enable_joystick=True)
        self.timer = self.node.create_timer(0.02, self.step)  # 50 Hz client state update
        self.vx_cmd = 0.0
        self.vy_cmd = 0.0
        self.vyaw_cmd = 0.0
        print("Control-enabled X2 example. Use game_controller_node on the host.")
        print("L2 + OPTIONS: start; L2 + UP + LEFT: stop publishing; L1: ready; R1: zero.")
        print("Start, ready and zero requests may move the robot. Stop is not a hardware emergency stop.")

    def step(self):
        self.robot.update_robot_state()
        # Expose stick inputs only after client control has started
        if self.robot.control_started:
            self.vx_cmd = self.robot.remote_controller.ly
            self.vy_cmd = -self.robot.remote_controller.lx
            self.vyaw_cmd = -self.robot.remote_controller.rx
        else:
            self.vx_cmd = 0.0
            self.vy_cmd = 0.0
            self.vyaw_cmd = 0.0
        self.robot.joy_key = None


if __name__ == "__main__":
    rclpy.init()
    node = rclpy.create_node('robot_client_node')
    controller = RobotController(node)

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
