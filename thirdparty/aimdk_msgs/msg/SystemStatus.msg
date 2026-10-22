uint8 value

uint8 IN_INITIAL = 0         # Initializing
uint8 IN_READY = 1           # Ready
uint8 IN_MOVE = 2            # Transitioning; new transitions rejected
uint8 IN_ROLLBACK = 3        # Rolling back; new transitions rejected
uint8 IN_FALLBACK = 4        # Degraded; transitions accepted
uint8 IN_FALLBACK_MOVE = 5   # Fallback transition; new transitions rejected
