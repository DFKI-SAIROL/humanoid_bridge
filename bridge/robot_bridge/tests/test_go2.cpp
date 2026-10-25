// go2_frame.hpp against Unitree's SDK (unitree_sdk2py CRC packing on the same command), and the
// Go2's stop frames. The robot drops a frame whose CRC is wrong. No ROS runtime.
#include "go2_frame.hpp"
#include "estop.hpp"

#include <gtest/gtest.h>

using namespace sairol_bridge;

namespace
{
    constexpr int NUM_MOTORS = 12;

    // The body motors' targets, as the bridge fills them; the frame fills the rest.
    unitree_go::msg::LowCmd body()
    {
        unitree_go::msg::LowCmd msg;
        for (int i = 0; i < NUM_MOTORS; ++i)
        {
            auto &c = msg.motor_cmd[i];
            c.q = static_cast<float>(0.01 * i);   // as Python's struct.pack('f') rounds it
            c.dq = static_cast<float>(-0.02 * i);
            c.tau = 0.5f;
            c.kp = 10.0f + i;
            c.kd = 1.5f;
        }
        for (int i = NUM_MOTORS; i < go2::NUM_SLOTS; ++i)
        {
            msg.motor_cmd[i].kp = 99.0f;   // a stale value the frame must clear
        }
        return msg;
    }

    Joint joint(float_t kp, float_t kd, float_t tau_limit)
    {
        Joint j;
        j.idx = 0;
        j.kp = kp;
        j.kd = kd;
        j.tau_limit = tau_limit;
        j.q_min = -3.0f;
        j.q_max = 3.0f;
        return j;
    }
} // namespace

TEST(Go2Frame, HeaderLevelFlagAndGpio)
{
    auto msg = body();
    go2::finishFrame(msg, NUM_MOTORS);
    EXPECT_EQ(msg.head[0], 0xFE);
    EXPECT_EQ(msg.head[1], 0xEF);
    EXPECT_EQ(msg.level_flag, 0xFF);
    EXPECT_EQ(msg.gpio, 0);
}

TEST(Go2Frame, BodyKeepsItsTargetsAndTheRestStops)
{
    auto msg = body();
    go2::finishFrame(msg, NUM_MOTORS);
    for (int i = 0; i < go2::NUM_SLOTS; ++i)
    {
        EXPECT_EQ(msg.motor_cmd[i].mode, go2::MODE_SERVO) << "slot " << i;
    }
    EXPECT_FLOAT_EQ(msg.motor_cmd[11].kp, 21.0f);
    for (int i = NUM_MOTORS; i < go2::NUM_SLOTS; ++i)
    {
        const auto &c = msg.motor_cmd[i];
        EXPECT_EQ(c.q, go2::POS_STOP) << "slot " << i;
        EXPECT_EQ(c.dq, go2::VEL_STOP);
        EXPECT_EQ(c.kp, 0.0f);
        EXPECT_EQ(c.kd, 0.0f);
        EXPECT_EQ(c.tau, 0.0f);
    }
}

TEST(Go2Frame, CrcMatchesUnitreeSdk)
{
    auto msg = body();
    go2::finishFrame(msg, NUM_MOTORS);
    EXPECT_EQ(msg.crc, 1669865753u);
    msg.motor_cmd[3].q = static_cast<float>(0.3001);
    go2::finishFrame(msg, NUM_MOTORS);
    EXPECT_EQ(msg.crc, 3373397468u);
}

// The latch on the Go2: a damp stop frame in the Go2 envelope damps every motor and stays valid.
TEST(Go2Frame, DampLatchFrame)
{
    unitree_go::msg::LowCmd msg;
    const Joint calf = joint(20.0f, 0.5f, 45.43f);
    for (int i = 0; i < NUM_MOTORS; ++i)
    {
        const auto frame = estop::stopFrame(estop::Mode::Damp, calf, -1.5f, -1.5f, 2.0f);
        msg.motor_cmd[i].q = frame.q;
        msg.motor_cmd[i].dq = frame.dq;
        msg.motor_cmd[i].kp = frame.kp;
        msg.motor_cmd[i].kd = frame.kd;
        msg.motor_cmd[i].tau = frame.tau;
    }
    go2::finishFrame(msg, NUM_MOTORS);
    for (int i = 0; i < NUM_MOTORS; ++i)
    {
        EXPECT_EQ(msg.motor_cmd[i].kp, 0.0f);
        EXPECT_EQ(msg.motor_cmd[i].kd, 2.0f);
    }
    auto again = msg;
    go2::finishFrame(again, NUM_MOTORS);
    EXPECT_EQ(again.crc, msg.crc) << "the frame is stable: a re-send is the same frame";
}

TEST(Go2Frame, HoldLatchFrameKeepsThePose)
{
    const Joint thigh = joint(20.0f, 0.5f, 23.7f);
    const auto frame = estop::stopFrame(estop::Mode::Hold, thigh, 0.8f, 0.8f, 2.0f);
    unitree_go::msg::LowCmd msg;
    msg.motor_cmd[1].q = frame.q;
    msg.motor_cmd[1].kp = frame.kp;
    msg.motor_cmd[1].kd = frame.kd;
    go2::finishFrame(msg, NUM_MOTORS);
    EXPECT_FLOAT_EQ(msg.motor_cmd[1].q, 0.8f);
    EXPECT_GT(msg.motor_cmd[1].kp, 0.0f);
}
