# /hand_state/<left|right>: one hand, only on a robot with hands. Arrays follow joint_names.
builtin_interfaces/Time stamp   # when the bridge received the hand's message (ROS clock)
string[] joint_names
float32[] q
float32[] dq
float32[] tau_est
