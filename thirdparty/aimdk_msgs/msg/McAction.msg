# Enumeration value.
int32 value

# Passive mode.
int32 PASSIVE_DEFAULT = 1

# Soft emergency stop.
int32 SOFT_EMERGENCY_STOP = 2

# Damping mode.
int32 DAMPING_DEFAULT = 3

# Zero-torque mode.
int32 ZERO_TORQUE_DEFAULT = 4

# Position-controlled standing.
int32 JOINT_DEFAULT = 100

# Position-controlled joint freeze.
int32 JOINT_FREEZE = 101

# Torque-controlled standing.
int32 STAND_DEFAULT = 200

# Standing with body motion.
int32 STAND_BODY_CONTROL = 201

# Walking mode.
int32 LOCOMOTION_DEFAULT = 300

# Running mode.
int32 RUN_DEFAULT = 301

# Off-road locomotion.
int32 LOCOMOTION_STEP = 302

# VR teleoperation.
int32 VR_REMOTE_CONTROLLER = 400

# Skills.
# Sit down.
int32 SIT_DOWN_DEFAULT = 2000

# Crouch down.
int32 CROUCH_DOWN_DEFAULT = 2002

# Lie down.
int32 LIE_DOWN_DEFAULT = 2004

# Stand up from lying down.
int32 STAND_UP_DEFAULT = 2005

# Ascend stairs.
int32 ASCEND_STAIRS = 2006

# Descend stairs.
int32 DESCEND_STAIRS = 2008
