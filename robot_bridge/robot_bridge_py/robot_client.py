import rclpy
import time
from enum import Enum
import numpy as np
from bridge_interface.msg import RobotCmd, MotorCmd
from bridge_interface.srv import SetDefaultPosition


from std_srvs.srv import Trigger
from rclpy.node import Node
from rclpy.client import Client as ROSClient
import numpy as np
from scipy.spatial.transform import Rotation as R
import struct
from copy import copy


def rpy_to_quat(rpy):
    r = R.from_euler('xyz', rpy)  
    quat = r.as_quat(scalar_first=True)  # return [w, x, y, z]
    return quat


# class WirelessKey_H1_G1:
#     KEY_R1     = 1 << 0
#     KEY_L1     = 1 << 1
#     KEY_START  = 1 << 2
#     KEY_SELECT = 1 << 3
#     KEY_R2     = 1 << 4
#     KEY_L2     = 1 << 5
#     KEY_A      = 1 << 8
#     KEY_B      = 1 << 9
#     KEY_X      = 1 << 10
#     KEY_Y      = 1 << 11
#     KEY_UP     = 1 << 12
#     KEY_RIGHT  = 1 << 13
#     KEY_DOWN   = 1 << 14
#     KEY_LEFT   = 1 << 15

class KeyMap:
    R1 = 0
    L1 = 1
    start = 2
    select = 3
    R2 = 4
    L2 = 5
    F1 = 6
    F2 = 7
    A = 8
    B = 9
    X = 10
    Y = 11
    up = 12
    right = 13
    down = 14
    left = 15


class RemoteController:
    def __init__(self):
        self.lx = 0
        self.ly = 0
        self.rx = 0
        self.ry = 0
        self.button = [0] * 16
        self.last_button = [0] * 16
        self.new_event = False

    def set(self, data):
        # wireless_remote
        keys = struct.unpack("H", data[2:4])[0]
        for i in range(16):
            self.button[i] = (keys & (1 << i)) >> i
        self.lx = struct.unpack("f", data[4:8])[0]
        self.rx = struct.unpack("f", data[8:12])[0]
        self.ry = struct.unpack("f", data[12:16])[0]
        self.ly = struct.unpack("f", data[20:24])[0]

        if self.button != self.last_button:
            self.last_button = copy(self.button)
            self.new_event = True
        else:
            self.new_event = False

    def set_joy(self, msg):
        # ROS game_controller_node uses SDL's standardized button and axis order
        if len(msg.axes) < 6 or len(msg.buttons) < 15:
            return False
        if not np.all(np.isfinite(msg.axes)) or np.any(np.abs(msg.axes) > 1.0):
            return False
        if any(button not in (0, 1) for button in msg.buttons):
            return False

        buttons = [0] * 16
        mapping = ((KeyMap.R1, 10), (KeyMap.L1, 9), (KeyMap.start, 6),
                   (KeyMap.select, 4), (KeyMap.F1, 7), (KeyMap.F2, 8),
                   (KeyMap.A, 0), (KeyMap.B, 1), (KeyMap.X, 2), (KeyMap.Y, 3),
                   (KeyMap.up, 11), (KeyMap.right, 14), (KeyMap.down, 12), (KeyMap.left, 13))
        for key, index in mapping:
            buttons[key] = msg.buttons[index]
        # Standardized trigger axes are zero when released and negative when pressed
        buttons[KeyMap.L2] = int(msg.axes[4] < -0.5)
        buttons[KeyMap.R2] = int(msg.axes[5] < -0.5)
        self.last_button = self.button
        self.button = buttons
        self.new_event = any(value and not self.last_button[i] for i, value in enumerate(buttons))
        # Preserve the signs expected by the existing H1 policy example
        self.lx, self.ly = -msg.axes[0], msg.axes[1]
        self.rx, self.ry = -msg.axes[2], msg.axes[3]
        if msg.buttons[5] or any(msg.buttons[15:]):
            self.new_event = False
        return True

    def is_exact_combo(self, buttons, combo_keys):
        return (
            all(buttons[k] for k in combo_keys) and
            not any(buttons[i] for i in range(len(buttons)) if i not in combo_keys)
        )



class RobotClient:
    def __init__(self, node, robot_type, num_dof, control_frequency, interpolation_order=0, enable_joystick=False) -> None:
        self.node: Node = node
        self.robot_type = robot_type
        self.num_dof = num_dof
        self.time_count = 0
        self.control_frequency = control_frequency

        self.cmd = RobotCmd()
        self.cmd.motor_cmd = [MotorCmd() for _ in range(self.num_dof)]
        self.cmd.interpolation_order = interpolation_order
        self.cmd.hold_position = False
        self.cmd.duration = 1 / control_frequency

        self.base_ang_vel = np.zeros(3, dtype=np.float32)
        self.projected_gravity = np.zeros(3, dtype=np.float32)
        
        self._q_pos = np.zeros(num_dof, dtype=np.float32)
        self._q_vel = np.zeros(num_dof, dtype=np.float32)
        self._tau = np.zeros(num_dof, dtype=np.float32)
        self._quat = np.zeros(4, dtype=np.float32)
        self._angular_velocity = np.zeros(3, dtype=np.float32)
        

        self._default_duration = 2.0
        self.joy_key = None
        
        if self.robot_type == "T1":
            from booster_interface.msg import LowState
            from booster_interface.msg import RemoteControllerState as JoyMsg
            state_topic_name = '/low_state' 
            joy_topic_name = '/remote_controller_state'
            low_state_handler = self._low_state_handler_booster
            joy_handler = self._joy_handler_booster

            self._default_pos = np.array([0.0,  0.0,
                                        0.25, -1.4, 0.0, -0.5,
                                        0.25, 1.4, 0.0, 0.5,
                                        0.0,
                                        -0.1, 0.0, 0.0, 0.2, -0.1, 0.0,
                                        -0.1, 0.0, 0.0, 0.2, -0.1, 0.0,])
            self._default_kp =  np.array([5., 5.,
                                        40., 50., 20., 10.,
                                        40., 50., 20., 10.,
                                        100., 
                                        350., 350., 180., 350., 300., 300.,
                                        350., 350., 180., 350., 300., 300.])
            self._default_kd = np.array([0.1, 0.1,
                                        0.5, 1.5, 0.2, 0.2,
                                        0.5, 1.5, 0.2, 0.2,
                                        5.0,
                                        7.5, 7.5, 3., 5.5, 0.5, 0.5,
                                        7.5, 7.5, 3., 5.5, 0.5, 0.5])
            self.key_count = 0
            
            self.joystick_subscription = self.node.create_subscription(JoyMsg, joy_topic_name, joy_handler, 1)
            
            print("Key mapping for Booster robot:"
                  "\n- Start control: LT + RT + START"
                  "\n- Ready position: LB"
                  "\n- Zero position: RB"
                  "\n- Stop control: BACK"
                  "\n- Emergency stop: LT + BACK"
                  )
            
        elif self.robot_type == "G1": 
            from unitree_hg.msg import LowState
            from unitree_go.msg import WirelessController as JoyMsg
            state_topic_name = '/lowstate'
            # joy_topic_name = '/wirelesscontroller'
            low_state_handler = self._low_state_handler_unitree
            # joy_handler = self._joy_handler_unitree
            
            self._default_pos = np.array([-0.1,  0.0,  0.0,  0.3, -0.2, 0.0, 
                                          -0.1,  0.0,  0.0,  0.3, -0.2, 0.0,
                                           0.0, 0.0, 0.0,
                                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            
            self._default_kp =  np.array([100, 100, 100, 150, 40, 40, 
                                          100, 100, 100, 150, 40, 40,
                                          200, 200, 200,
                                          100, 100, 50, 50, 20, 20, 20,
                                          100, 100, 50, 50, 20, 20, 20])
            
            self._default_kd = np.array([6, 6, 6, 4, 2, 2, 
                                         6, 6, 6, 4, 2, 2,
                                         1, 1, 1, 
                                         2, 2, 2, 2, 1, 1, 1,
                                         2, 2, 2, 2, 1, 1, 1])
            
            self.remote_controller = RemoteController()
            print("Key mapping for Unitree G1 robot:"
                  "\n- Start control: L2 + START"
                  "\n- Stop control: L2 + UP + LEFT"
                  "\n- Ready position: L1"
                  "\n- Zero position: R1"
                  )
            
        elif self.robot_type == "H1": 
            from unitree_go.msg import LowState
            from unitree_go.msg import WirelessController as JoyMsg
            state_topic_name = '/lowstate'
            # joy_topic_name = '/wirelesscontroller'
            low_state_handler = self._low_state_handler_unitree
            # joy_handler = self._joy_handler_unitree
            
            self._default_pos = np.array([ 0.0, -0.1,  0.3,  
                                           0.0, -0.1,  0.3,
                                           0.0,  0.0,  0.0,
                                           0.0, -0.2, -0.2,
                                           0.0,  0.0,  0.0, 0.0,
                                           0.0,  0.0,  0.0, 0.0]) 
                                           
   
            self._default_kp =  np.array([150, 150, 200, 
                                          150, 150, 200, 
                                          300, 150, 150,
                                            0,  40,  40,
                                          100, 100, 50, 50, 
                                          100, 100, 50, 50])
            
            self._default_kd = np.array([2, 2, 4,
                                         2, 2, 4,
                                         3, 2, 2,
                                         0, 2, 2,
                                         2, 2, 2, 2,
                                         2, 2, 2, 2])
            
            self.remote_controller = RemoteController()
            print("Key mapping for Unitree H1 robot:"
                  "\n- Start control: L2 + START"
                  "\n- Stop control: L2 + UP + LEFT"
                  "\n- Ready position: L1"
                  "\n- Zero position: R1"
                  )
            

        elif self.robot_type == "H1_2": 
            from unitree_hg.msg import LowState
            from unitree_go.msg import WirelessController as JoyMsg
            state_topic_name = '/lowstate'
            # joy_topic_name = '/wirelesscontroller'
            low_state_handler = self._low_state_handler_unitree
            # joy_handler = self._joy_handler_unitree
            
            self._default_pos = np.array([0.0, -0.2,  0.0,  0.5, -0.3, 0.0,
                                          0.0, -0.2,  0.0,  0.5, -0.3, 0.0,
                                          0.0,
                                          0.28, 0.0,  0.0,  0.52, 0.0, 0.0, 0.0,
                                          0.28, 0.0,  0.0,  0.52, 0.0, 0.0, 0.0]) 
                                           
   
            self._default_kp = np.array([200, 200, 200, 300, 80, 40,
                                         100, 100, 100, 150, 60, 60,
                                         300,
                                         100, 100, 50, 50, 20, 20, 20,
                                         100, 100, 50, 50, 20, 20, 20])
            
            self._default_kd = np.array([2.5, 2.5, 2.5, 4, 2, 2,
                                         2, 2, 2, 4, 2, 2,
                                         3,
                                         2, 2, 2, 2, 1, 1, 1,
                                         2, 2, 2, 2, 1, 1, 1])
            
            self.remote_controller = RemoteController()
            print("Key mapping for Unitree H1-2 robot:"
                  "\n- Start control: L2 + START"
                  "\n- Stop control: L2 + UP + LEFT"
                  "\n- Ready position: L1"
                  "\n- Zero position: R1"
                  )

        elif self.robot_type == "X2":
            from aimdk_msgs.msg import JointStateArray
            from sensor_msgs.msg import Imu
            from rclpy.qos import qos_profile_sensor_data

            if self.num_dof not in (29, 30):
                raise ValueError("X2 requires 29 joints without the head or 30 joints with head yaw.")

            self._angular_acceleration = np.zeros(3, dtype=np.float32)

            # Offline reference values; keep aligned with X2_config.yaml
            self._default_pos = np.array([-0.05, 0.0, 0.0, 0.1, -0.05, 0.0,
                                          -0.05, 0.0, 0.0, 0.1, -0.05, 0.0,
                                           0.0, 0.0, 0.0,
                                           0.4, 0.0, 0.0, -1.2, 0.0, 0.0, 0.0,
                                           0.4, 0.0, 0.0, -1.2, 0.0, 0.0, 0.0,
                                           0.0])[:self.num_dof]

            self._default_kp = np.array([40.0, 40.0, 30.0, 80.0, 40.0, 20.0,
                                        40.0, 40.0, 30.0, 80.0, 40.0, 20.0,
                                        150.0, 300.0, 300.0,
                                        30.0, 20.0, 20.0, 50.0, 50.0, 20.0, 20.0,
                                        30.0, 20.0, 20.0, 50.0, 50.0, 20.0, 20.0,
                                        20.0])[:self.num_dof]

            self._default_kd = np.array([4.0, 4.0, 3.0, 8.0, 4.0, 2.0,
                                        4.0, 4.0, 3.0, 8.0, 4.0, 2.0,
                                        3.0, 3.0, 3.0,
                                        1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                                        1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                                        1.0])[:self.num_dof]

            self._default_duration = 1.5

            self.leg_state_subscription = self.node.create_subscription(
                JointStateArray, '/aima/hal/joint/leg/state', self._leg_state_handler_x2, qos_profile_sensor_data)
            self.waist_state_subscription = self.node.create_subscription(
                JointStateArray, '/aima/hal/joint/waist/state', self._waist_state_handler_x2, qos_profile_sensor_data)
            self.arm_state_subscription = self.node.create_subscription(
                JointStateArray, '/aima/hal/joint/arm/state', self._arm_state_handler_x2, qos_profile_sensor_data)
            if self.num_dof == 30:
                self.head_state_subscription = self.node.create_subscription(
                    JointStateArray, '/aima/hal/joint/head/state', self._head_state_handler_x2, qos_profile_sensor_data)
            self.imu_state_subscription = self.node.create_subscription(
                Imu, '/aima/hal/imu/torso/state', self._imu_state_handler_x2, qos_profile_sensor_data)

        if self.robot_type != "X2":
            self.low_state_subscription = self.node.create_subscription(LowState, state_topic_name, low_state_handler, 1)
        # self.joystick_subscription = self.node.create_subscription(JoyMsg, joy_topic_name, joy_handler, 1)
        self.low_cmd_publisher = self.node.create_publisher(RobotCmd, '/robot_cmd', 1)
        
        self.start_control_client = self.node.create_client(SetDefaultPosition, '/start_control')
        self.goto_zero_position_client = self.node.create_client(Trigger, '/zero_position_control')
        self.ready_position_client = self.node.create_client(Trigger, '/ready_position_control')
        self.stop_control_client = self.node.create_client(Trigger, '/stop_control')

        self.control_start_time = None
        self.control_started = False

        if self.robot_type == "X2" and enable_joystick:
            from sensor_msgs.msg import Joy
            from rclpy.qos import qos_profile_sensor_data

            self.remote_controller = RemoteController()
            self._joy_future_x2 = None
            self._joy_stop_requested_x2 = False
            self.joystick_subscription = self.node.create_subscription(
                Joy, '/joy', self._joy_handler_x2, qos_profile_sensor_data)

    @property
    def q_pos(self):
        return self._q_pos

    @property
    def q_vel(self):
        return self._q_vel

    @property
    def tau(self):
        return self._tau
    
    @property
    def quat(self):
        return self._quat
    
    @property
    def angular_velocity(self):
        return self._angular_velocity

    def _low_state_handler_booster(self, low_state_msg):
        self.time_count += 1
        self._angular_velocity = low_state_msg.imu_state.gyro
        self._angular_acceleration = low_state_msg.imu_state.acc
        
        for i, motor in enumerate(low_state_msg.motor_state_serial):
            self._q_pos[i] = motor.q
            self._q_vel[i] = motor.dq
            self._tau[i] = motor.tau_est
        
        self._quat = rpy_to_quat(low_state_msg.imu_state.rpy)
        
    def _low_state_handler_unitree(self, low_state_msg):
        self.time_count += 1
        self._quat = low_state_msg.imu_state.quaternion
        self._angular_velocity = low_state_msg.imu_state.gyroscope
        self._angular_acceleration = low_state_msg.imu_state.accelerometer
        
        for i, motor in enumerate(low_state_msg.motor_state):
            if i < self._q_pos.shape[0]:
                self._q_pos[i] = motor.q
                self._q_vel[i] = motor.dq
                self._tau[i] = motor.tau_est

        # Handle joystick input
        self.remote_controller.set(low_state_msg.wireless_remote)
        
        if self.remote_controller.new_event:
            time_now = time.time()
            buttons = self.remote_controller.button
            
            if self.remote_controller.is_exact_combo(buttons, [KeyMap.L2, KeyMap.start]):
                self.node.get_logger().info("Starting control...")
                if not self.control_started:
                    future = self.init_control()
                    self.control_start_time = time_now + self._default_duration
                return
            elif self.remote_controller.is_exact_combo(buttons, [KeyMap.L2, KeyMap.up, KeyMap.left]):
                self.node.get_logger().info("Stopping control...")
                # Stop control is done in the Robot Bridge.
                # future = self.stop_control()
                self.control_start_time = None
                return
            elif self.remote_controller.is_exact_combo(buttons, [KeyMap.L1]):
                self.node.get_logger().info("Ready position control...")
                if not self.control_started:
                    self.goto_default_position()
                    self.control_start_time = None
                else:
                    self.node.get_logger().warn("Control already started, please stop the control first by pressing BACK.")
                return
            elif self.remote_controller.is_exact_combo(buttons, [KeyMap.R1]):
                self.node.get_logger().info("Zero position control...")
                if not self.control_started:
                    self.goto_zero_position()
                    self.control_start_time = None
                else:
                    self.node.get_logger().warn("Control already started, please stop the control first by pressing BACK.")
                return  
            else:
                self.joy_key = buttons

    def _joy_handler_x2(self, msg):
        if not self.remote_controller.set_joy(msg):
            self.node.get_logger().error("Invalid X2 Joy message; use game_controller_node mapping.")
            return
        if not self.remote_controller.new_event:
            return

        buttons = self.remote_controller.button
        try:
            if self.remote_controller.is_exact_combo(buttons, [KeyMap.L2, KeyMap.up, KeyMap.left]):
                if self._joy_future_x2 is not None and not self._joy_future_x2.done():
                    self._joy_stop_requested_x2 = True
                    return
                self._joy_future_x2 = self.stop_control()
            elif self._joy_future_x2 is not None and not self._joy_future_x2.done():
                self.node.get_logger().warn("A joystick control request is still pending.")
                return
            elif self.remote_controller.is_exact_combo(buttons, [KeyMap.L2, KeyMap.start]):
                if self.control_start_time is not None:
                    return
                self._joy_future_x2 = self.init_control()
            elif self.remote_controller.is_exact_combo(buttons, [KeyMap.L1]):
                if self.control_start_time is not None:
                    self.node.get_logger().warn("Stop client control before requesting the ready position.")
                    return
                self._joy_future_x2 = self.goto_default_position()
            elif self.remote_controller.is_exact_combo(buttons, [KeyMap.R1]):
                if self.control_start_time is not None:
                    self.node.get_logger().warn("Stop client control before requesting the zero position.")
                    return
                self._joy_future_x2 = self.goto_zero_position()
            else:
                self.joy_key = copy(buttons)
                return
        except RuntimeError as error:
            self.node.get_logger().error(str(error))
            return
        self._joy_future_x2.add_done_callback(self._joy_control_response_x2)

    def _joy_control_response_x2(self, future):
        if self._joy_stop_requested_x2:
            self._joy_stop_requested_x2 = False
            try:
                self._joy_future_x2 = self.stop_control()
            except RuntimeError as error:
                self.node.get_logger().error(str(error))
                return
            self._joy_future_x2.add_done_callback(self._joy_control_response_x2)

    def _check_joint_state_message_x2(self, msg, message_count, active_count, group_name):
        from aimdk_msgs.msg import DomainErrorState

        if len(msg.joints) != message_count:
            self.node.get_logger().error(f"X2 {group_name} state length mismatch.")
            return False
        if msg.state.value != DomainErrorState.NONE:
            self.node.get_logger().error(f"X2 {group_name} group reports state {msg.state.value}.")
            return False

        max_value = np.finfo(np.float32).max
        for joint in msg.joints[:active_count]:
            values = (joint.position, joint.velocity, joint.effort)
            if joint.error_code != 0 or not np.all(np.isfinite(values)) or np.any(np.abs(values) > max_value):
                self.node.get_logger().error(f"X2 {group_name} joint has invalid data or error code {joint.error_code}.")
                return False
        return True

    def _leg_state_handler_x2(self, msg):
        if not self._check_joint_state_message_x2(msg, 12, 12, "leg"):
            return

        for i, joint in enumerate(msg.joints):
            self._q_pos[i] = joint.position
            self._q_vel[i] = joint.velocity
            self._tau[i] = joint.effort

    def _waist_state_handler_x2(self, msg):
        if not self._check_joint_state_message_x2(msg, 3, 3, "waist"):
            return

        for i, joint in enumerate(msg.joints):
            self._q_pos[12 + i] = joint.position
            self._q_vel[12 + i] = joint.velocity
            self._tau[12 + i] = joint.effort

    def _arm_state_handler_x2(self, msg):
        if not self._check_joint_state_message_x2(msg, 14, 14, "arm"):
            return

        for i, joint in enumerate(msg.joints):
            self._q_pos[15 + i] = joint.position
            self._q_vel[15 + i] = joint.velocity
            self._tau[15 + i] = joint.effort

    def _head_state_handler_x2(self, msg):
        if self.num_dof != 30:
            return
        if not self._check_joint_state_message_x2(msg, 2, 1, "head"):
            return

        joint = msg.joints[0]
        self._q_pos[29] = joint.position
        self._q_vel[29] = joint.velocity
        self._tau[29] = joint.effort

    def _imu_state_handler_x2(self, msg):
        if (msg.orientation_covariance[0] == -1.0 or
                msg.angular_velocity_covariance[0] == -1.0 or
                msg.linear_acceleration_covariance[0] == -1.0):
            self.node.get_logger().error("X2 IMU reports unavailable measurements.")
            return

        values = np.array([
            msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z,
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
        ], dtype=np.float64)
        if not np.all(np.isfinite(values)) or np.any(np.abs(values) > np.finfo(np.float32).max):
            self.node.get_logger().error("X2 IMU contains invalid values.")
            return

        norm = np.linalg.norm(values[:4])
        if norm < 1e-12:
            self.node.get_logger().error("X2 IMU quaternion has zero norm.")
            return

        # Keep the HAL torso frame; quaternion order: w, x, y, z
        self._quat = (values[:4] / norm).astype(np.float32)
        self._angular_velocity = values[4:7].astype(np.float32)
        # Existing member stores linear acceleration
        self._angular_acceleration = values[7:10].astype(np.float32)

    def update_robot_state(self):
        time_now = time.time()
        if self.control_start_time is not None and time_now > self.control_start_time:
            self.control_started = True
        else:
            self.control_started = False
    
    def _joy_handler_booster(self, joy_msg):
        """
        Handle joystick messages for the Booster robot.
        """
        time_now = time.time()

        self.key_count = sum([
            joy_msg.a, joy_msg.b, joy_msg.x, joy_msg.y,
            joy_msg.lb, joy_msg.rb, joy_msg.lt, joy_msg.rt,
            joy_msg.ls, joy_msg.rs, joy_msg.back, joy_msg.start,
            joy_msg.hat_u, joy_msg.hat_d,
            joy_msg.hat_l, joy_msg.hat_r, joy_msg.hat_lu,
            joy_msg.hat_ld, joy_msg.hat_ru, joy_msg.hat_rd
        ])
    
        if joy_msg.lt and joy_msg.rt and joy_msg.start and self.key_count == 3:  # start: LT + RT + START
            self.node.get_logger().info("Starting control...")
            if not self.control_started:
                future = self.init_control()
                self.control_start_time = time_now + self._default_duration
            return
        elif joy_msg.lb and self.key_count == 1:  # ready position: LB
            self.node.get_logger().info("Ready position control...")
            if not self.control_started:
                self.goto_default_position()
                self.control_start_time = None
            else:
                self.node.get_logger().warn("Control already started, please stop the control first by pressing BACK.")
            return
        elif joy_msg.rb and self.key_count == 1:  # zero position: RB
            self.node.get_logger().info("Zero position control...")
            if not self.control_started:
                self.goto_zero_position()
                self.control_start_time = None
            else:
                self.node.get_logger().warn("Control already started, please stop the control first by pressing BACK.")
            return
        elif joy_msg.back and self.key_count == 1:  # stop: BACK
            self.node.get_logger().info("Stopping control...")
            future = self.stop_control()
            self.control_start_time = None
            return
        elif joy_msg.lt and joy_msg.back and self.key_count == 2:  # emergency stop: LT + BACK
            self.control_start_time = None
        else:
            # Set key only for unknown key combinations
            self.joy_key = joy_msg
  
    # def _joy_handler_unitree(self, joy_msg):
    #     key = joy_msg.keys
    #     time_now = time.time()
        
    #     if (key & (WirelessKey_H1_G1.KEY_L2 | WirelessKey_H1_G1.KEY_START)) == (WirelessKey_H1_G1.KEY_L2 | WirelessKey_H1_G1.KEY_START):
    #         self.node.get_logger().info("Starting control...")
    #         if not self.control_started:
    #             future = self.init_control()
    #             self.control_start_time = time_now + self._default_duration
    #         return
    #     elif (key & (WirelessKey_H1_G1.KEY_L2 | WirelessKey_H1_G1.KEY_UP | WirelessKey_H1_G1.KEY_LEFT)) == (WirelessKey_H1_G1.KEY_L2 | WirelessKey_H1_G1.KEY_UP | WirelessKey_H1_G1.KEY_LEFT):
    #         self.node.get_logger().info("Stopping control...")
    #         future = self.stop_control()
    #         self.control_start_time = None
    #         return
    #     elif key & WirelessKey_H1_G1.KEY_L1:
    #         self.node.get_logger().info("Ready position control...")
    #         if not self.control_started:
    #             self.goto_default_position()
    #             self.control_start_time = None
    #         else:
    #             self.node.get_logger().warn("Control already started, please stop the control first by pressing BACK.")
    #         return
    #     elif key & WirelessKey_H1_G1.KEY_R1:
    #         self.node.get_logger().info("Zero position control...")
    #         if not self.control_started:
    #             self.goto_zero_position()
    #             self.control_start_time = None
    #         else:
    #             self.node.get_logger().warn("Control already started, please stop the control first by pressing BACK.")
    #         return  
    #     else:
    #         # Set key only for unknown key combinations
    #         self.joy_key = joy_msg
            
    def send_cmd(self, q_target_pos=None, q_target_vel=None, target_tau=None, target_kp=None, target_kd=None):
        for i in range(self.num_dof):
            self.cmd.motor_cmd[i].q = float(q_target_pos[i]) if q_target_pos is not None else self._default_pos[i]
            self.cmd.motor_cmd[i].dq = float(q_target_vel[i]) if q_target_vel is not None else 0.0
            self.cmd.motor_cmd[i].tau = float(target_tau[i]) if target_tau is not None else 0.0
            self.cmd.motor_cmd[i].kp = float(target_kp[i]) if target_kp is not None else self._default_kp[i]
            self.cmd.motor_cmd[i].kd = float(target_kd[i]) if target_kd is not None else self._default_kd[i]
        self.low_cmd_publisher.publish(self.cmd)
        
    def set_default_cmd(self, default_pos=None, default_kp=None, default_kd=None):
        """
        Set the default command for the robot.
        """
        if default_pos is not None:
            self._default_pos = np.array(default_pos, dtype=np.float32)
        if default_kp is not None:
            self._default_kp = np.array(default_kp, dtype=np.float32)
        if default_kd is not None:
            self._default_kd = np.array(default_kd, dtype=np.float32)
        
    def init_control(self, default_pos=None):
        """
        Start the control loop by calling the init_control service.
        """
        if default_pos is None:
            default_pos = self._default_pos

        if self.robot_type == "X2" and not self.start_control_client.service_is_ready():
            raise RuntimeError("start_control_client service is unavailable.")
        while self.robot_type != "X2" and not self.start_control_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('start_control service not available, waiting again...')
        
        request = SetDefaultPosition.Request()
        request.default_position = default_pos.tolist()
        future = self.start_control_client.call_async(request)
        if self.robot_type == "X2":
            future.add_done_callback(self._start_control_response_x2)
            return future
        return future.result()
    
    def stop_control(self):
        """
        Stop the control loop by calling the stop_control service.
        """        
        if self.robot_type == "X2" and not self.stop_control_client.service_is_ready():
            raise RuntimeError("stop_control_client service is unavailable.")
        while self.robot_type != "X2" and not self.stop_control_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('stop_control service not available, waiting again...')

        request = Trigger.Request()
        future = self.stop_control_client.call_async(request)
        if self.robot_type == "X2":
            future.add_done_callback(self._stop_control_response_x2)
            return future
        return future.result()
    
    def _check_control_response_x2(self, future):
        try:
            response = future.result()
        except Exception as error:
            self.node.get_logger().error(f"Control request failed: {error}")
            return False
        if not response.success:
            self.node.get_logger().error(f"Control request rejected: {response.message}")
            return False
        return True

    def _start_control_response_x2(self, future):
        if not self._check_control_response_x2(future):
            return

        # Wait for the default position transition before enabling client control
        self.control_start_time = time.time() + self._default_duration
        self.control_started = False

    def _stop_control_response_x2(self, future):
        if not self._check_control_response_x2(future):
            return

        self.control_start_time = None
        self.control_started = False

    def goto_default_position(self):
        """
        Send robot to default position by calling the ready_position_control service.
        """
        if self.robot_type == "X2" and not self.ready_position_client.service_is_ready():
            raise RuntimeError("ready_position_client service is unavailable.")
        while self.robot_type != "X2" and not self.ready_position_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('ready_position_control service not available, waiting again...')

        request = Trigger.Request()
        future = self.ready_position_client.call_async(request)
        if self.robot_type == "X2":
            future.add_done_callback(self._check_control_response_x2)
            return future
        return future.result()
        
    def goto_zero_position(self):
        """
        Send robot to zero position by calling the zero_position_control service.
        """
        if self.robot_type == "X2" and not self.goto_zero_position_client.service_is_ready():
            raise RuntimeError("goto_zero_position_client service is unavailable.")
        while self.robot_type != "X2" and not self.goto_zero_position_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('zero_position_control service not available, waiting again...')

        request = Trigger.Request()
        future = self.goto_zero_position_client.call_async(request)
        if self.robot_type == "X2":
            future.add_done_callback(self._check_control_response_x2)
            return future
        return future.result()
