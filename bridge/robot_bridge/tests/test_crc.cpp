// get_crc against Unitree's own SDK (unitree_sdk2py CRC().Crc on the same command).
// The robot drops a frame whose CRC is wrong, so a build that breaks this moves nothing.
#include "common/motor_crc_hg.h"

#include <gtest/gtest.h>

namespace
{
    unitree_hg::msg::LowCmd command()
    {
        unitree_hg::msg::LowCmd msg;
        msg.mode_pr = 0;
        msg.mode_machine = 5;
        for (int i = 0; i < 35; ++i)
        {
            auto &c = msg.motor_cmd[i];
            c.mode = i < 29 ? 1 : 0;
            c.q = static_cast<float>(0.01 * i);     // as Python's struct.pack('f') rounds it
            c.dq = static_cast<float>(-0.02 * i);
            c.tau = 0.5f;
            c.kp = 10.0f + i;
            c.kd = 1.5f;
            c.reserve = 0;
        }
        return msg;
    }
} // namespace

TEST(CrcHg, MatchesUnitreeSdk)
{
    auto msg = command();
    get_crc(msg);
    EXPECT_EQ(msg.crc, 1534576640u);
}

TEST(CrcHg, FollowsAChangedTarget)
{
    auto msg = command();
    msg.motor_cmd[3].q = static_cast<float>(0.3001);
    get_crc(msg);
    EXPECT_EQ(msg.crc, 3675310309u);
}
