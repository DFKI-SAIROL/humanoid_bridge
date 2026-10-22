// Unit tests for gates.hpp. No ROS. Run: build/robot_bridge/test_gates
#include "gates.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

using namespace sairol_bridge;
using namespace sairol_bridge::gates;

namespace
{
    Joint elbow(float_t rate = 3.0f)
    {
        Joint j;
        j.idx = 18;
        j.q_min = -1.0472f;
        j.q_max = 2.0944f;
        j.tau_limit = 25.0f;
        j.kp = 50.0f;
        j.kd = 2.0f;
        j.q_rate_limit = rate;
        return j;
    }
    constexpr float_t DT = 0.001f;
} // namespace

// --- gate 1: the joint range -------------------------------------------------

TEST(Range, ClampsBothEnds)
{
    EXPECT_FLOAT_EQ(limitTarget(5.0f, 2.0944f, elbow(), DT), 2.0944f);
    EXPECT_FLOAT_EQ(limitTarget(-5.0f, -1.0472f, elbow(), DT), -1.0472f);
}

TEST(Range, WithoutARateLimitOnlyTheRange)
{
    EXPECT_FLOAT_EQ(limitTarget(1.7f, 0.0f, elbow(0.0f), DT), 1.7f);
    EXPECT_FLOAT_EQ(limitTarget(9.0f, 0.0f, elbow(0.0f), DT), 2.0944f);
}

// --- gate 2: the rate limit ---------------------------------------------------

TEST(Rate, OneTickMovesAtMostRateTimesDt)
{
    EXPECT_NEAR(limitTarget(1.7f, 0.0f, elbow(), DT), 0.003f, 1e-6);
    EXPECT_NEAR(limitTarget(-1.0f, 0.0f, elbow(), DT), -0.003f, 1e-6);
    EXPECT_FLOAT_EQ(limitTarget(0.002f, 0.0f, elbow(), DT), 0.002f);   // within one step: exact
}

TEST(Rate, PsiZeroFirstTargetBecomesARamp)
{
    // The shadow run's worst first target: right elbow 1.30 -> 0.30.
    float_t q = 1.30f;
    int ticks = 0;
    while (std::fabs(q - 0.30f) > 1e-5f && ticks < 10000)
    {
        q = limitTarget(0.30f, q, elbow(), DT);
        ++ticks;
    }
    EXPECT_NEAR(ticks, 1.0 / 3.0 * 1000, 2);   // 1.0 rad at 3 rad/s
}

TEST(Rate, OutOfRangeTargetRampsToTheLimit)
{
    float_t q = 2.0f;
    for (int i = 0; i < 1000; ++i)
    {
        q = limitTarget(9.0f, q, elbow(), DT);
    }
    EXPECT_FLOAT_EQ(q, 2.0944f);
}

TEST(Rate, NonFinitePreviousFallsBackToTheRange)
{
    EXPECT_FLOAT_EQ(limitTarget(1.0f, NAN, elbow(), DT), 1.0f);
}

// --- gate 4: the torque window --------------------------------------------------

TEST(Window, BoundsByTorqueOverKp)
{
    // 25 Nm / 50 = 0.5 rad either side of the measured pose, at rest.
    EXPECT_FLOAT_EQ(torqueWindow(2.0f, 0.0f, 0.0f, 0.0f, 50.0f, 2.0f, 25.0f), 0.5f);
    EXPECT_FLOAT_EQ(torqueWindow(-2.0f, 0.0f, 0.0f, 0.0f, 50.0f, 2.0f, 25.0f), -0.5f);
    EXPECT_FLOAT_EQ(torqueWindow(0.2f, 0.0f, 0.0f, 0.0f, 50.0f, 2.0f, 25.0f), 0.2f);
}

TEST(Window, ZeroKpGivesTheMeasuredPose)
{
    for (const float_t kd : {0.0f, 2.0f, 8.0f})
    {
        for (const float_t dq : {0.0f, 20.0f, -20.0f})
        {
            const float_t q = torqueWindow(1.5f, 0.3f, dq, 0.0f, 0.0f, kd, 25.0f);
            EXPECT_TRUE(std::isfinite(q));
            EXPECT_FLOAT_EQ(q, 0.3f);
        }
    }
}

// --- last check: a tracking frame goes out only when every field is finite ----

TEST(Finite, EveryFieldIsChecked)
{
    EXPECT_TRUE(finiteCommand(0.1f, 0.0f, 50.0f, 2.0f, 0.5f));
    const float_t nan = std::numeric_limits<float_t>::quiet_NaN();
    const float_t inf = std::numeric_limits<float_t>::infinity();
    for (const float_t bad : {nan, inf, -inf})
    {
        EXPECT_FALSE(finiteCommand(bad, 0.0f, 50.0f, 2.0f, 0.0f));
        EXPECT_FALSE(finiteCommand(0.0f, bad, 50.0f, 2.0f, 0.0f));
        EXPECT_FALSE(finiteCommand(0.0f, 0.0f, bad, 2.0f, 0.0f));
        EXPECT_FALSE(finiteCommand(0.0f, 0.0f, 50.0f, bad, 0.0f));
        EXPECT_FALSE(finiteCommand(0.0f, 0.0f, 50.0f, 2.0f, bad));
    }
}

TEST(Finite, CatchesANanMeasuredQAtKpZero)
{
    // A NaN /lowstate after checkState_'s copy: gate 4 at kp 0 passes q_meas through.
    const float_t nan = std::numeric_limits<float_t>::quiet_NaN();
    const float_t q = torqueWindow(0.3f, nan, 0.0f, 0.0f, 0.0f, 2.0f, 25.0f);
    EXPECT_FALSE(finiteCommand(q, 0.0f, 0.0f, 2.0f, 0.0f));
}
