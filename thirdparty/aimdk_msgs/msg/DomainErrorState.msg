uint8 value

# 0: 无状态, 1: 阻尼模式, 2: 下电模式, 3: 下使能模式, 4: CAN通信断连模式, 其他：等同于无状态

# 无状态
uint8 NONE=0x00         

# 阻尼模式
uint8 DAMPING=0x01 

# 下电模式
uint8 POWERDOWN=0x02         

# 下使能模式
uint8 DISABLE=0x03 

# 通信断连模式
uint8 DISCONNECT=0x04