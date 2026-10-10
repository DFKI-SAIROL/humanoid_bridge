uint8 value

# 0: No state, 1: Damping, 2: Power off, 3: Disable, 4: CAN disconnected; other values mean no state.

# No state.
uint8 NONE=0x00         

# Damping mode.
uint8 DAMPING=0x01 

# Power-off mode.
uint8 POWERDOWN=0x02         

# Disabled mode.
uint8 DISABLE=0x03 

# Communication disconnected.
uint8 DISCONNECT=0x04