# /robot_state: the robot's state as the bridge read it, the same type on every robot.
# Arrays follow joint_names, the bridge YAML's order. One per robot state message, at its rate.
builtin_interfaces/Time stamp   # when the bridge received the robot's message (ROS clock)
uint32 tick                     # the robot's own counter; 0 on a robot without one
string[] joint_names
float32[] q
float32[] dq
float32[] tau_est
ImuState imu
RemoteState remote
