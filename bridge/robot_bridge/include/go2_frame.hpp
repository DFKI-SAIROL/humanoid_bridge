// go2_frame.hpp ----------------------------------------------------------------
// The Go2's /lowcmd envelope around the bridge's motor targets: header, level flag, gpio, the
// unused motor slots, the CRC. As unitree_sdk2's go2 examples set them. tests/test_go2.cpp covers it.
#pragma once

#include "unitree_go/msg/low_cmd.hpp"

namespace sairol_bridge::go2
{
    constexpr int NUM_SLOTS = 20;        // unitree_go LowCmd.motor_cmd
    constexpr uint8_t HEAD_0 = 0xFE, HEAD_1 = 0xEF;
    constexpr uint8_t LEVEL_LOW = 0xFF;  // low-level control
    constexpr uint8_t MODE_SERVO = 0x01; // PMSM servo
    constexpr float POS_STOP = 2.146e9f; // a slot past the robot's motors: no position target
    constexpr float VEL_STOP = 16000.0f; // and no velocity target

    // Header, level flag and gpio; every slot in servo mode, slots from num_motors on stopped
    // (POS_STOP, VEL_STOP, no gains, no torque); then the CRC. The body slots keep their targets.
    void finishFrame(unitree_go::msg::LowCmd &cmd, int num_motors);
} // namespace sairol_bridge::go2
