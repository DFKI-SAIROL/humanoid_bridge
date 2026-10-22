int8 mode
float32 q
float32 dq
float32 ddq
float32 tau_est
# Motor temperature: int8_t; display the reported value (documented range: -100 to 150).
int8 temperature
# Motor packet loss: display the reported value (documented range: 0 to 9999999999).
uint32 lost
# Motor status array: index 0 is the error flags (0 to 255); index 1 is the communication frequency (0 to 800).
uint32[2] reserve