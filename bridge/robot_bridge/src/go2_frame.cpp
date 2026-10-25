#include "go2_frame.hpp"

#include "common/motor_crc.h"

namespace sairol_bridge::go2
{
    void finishFrame(unitree_go::msg::LowCmd &cmd, int num_motors)
    {
        cmd.head[0] = HEAD_0;
        cmd.head[1] = HEAD_1;
        cmd.level_flag = LEVEL_LOW;
        cmd.gpio = 0;
        for (int i = 0; i < NUM_SLOTS; ++i)
        {
            auto &motor = cmd.motor_cmd[i];
            motor.mode = MODE_SERVO;
            if (i >= num_motors)
            {
                motor.q = POS_STOP;
                motor.dq = VEL_STOP;
                motor.kp = 0.0f;
                motor.kd = 0.0f;
                motor.tau = 0.0f;
            }
        }
        get_crc(cmd);
    }
} // namespace sairol_bridge::go2
