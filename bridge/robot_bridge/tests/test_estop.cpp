// Unit tests for estop.hpp. No ROS. Run: build/robot_bridge/test_estop
#include "estop.hpp"

#include <gtest/gtest.h>

#include <limits>

using namespace sairol_bridge;
using namespace sairol_bridge::estop;
using State = SafetyLatch::State;

namespace
{
    Joint joint(float_t kp, float_t kd, float_t tau_limit, bool parallel = false)
    {
        Joint j;
        j.idx = 0;
        j.kp = kp;
        j.kd = kd;
        j.tau_limit = tau_limit;
        j.if_parallel_joint = parallel;
        return j;
    }

    SafetyLatch latchedFromTracking(Cause cause)
    {
        SafetyLatch latch;
        latch.trigger(cause, true);
        return latch;
    }
} // namespace

// --- stop frames -------------------------------------------------------------

TEST(StopFrame, DampHasNoSpringAndConfigKd)
{
    const auto f = stopFrame(Mode::Damp, joint(100, 2, 88), 0.4f, 9.9f, 8.0f);
    EXPECT_FLOAT_EQ(f.kp, 0.0f);
    EXPECT_FLOAT_EQ(f.kd, 8.0f);
    EXPECT_FLOAT_EQ(f.tau, 0.0f);
    EXPECT_FLOAT_EQ(f.dq, 0.0f);
    EXPECT_FLOAT_EQ(f.q, 0.4f);
}

TEST(StopFrame, DampIgnoresParallel)
{
    const auto f = stopFrame(Mode::Damp, joint(100, 2, 88, true), 0.4f, 9.9f, 8.0f);
    EXPECT_FLOAT_EQ(f.kp, 0.0f);
    EXPECT_FLOAT_EQ(f.kd, 8.0f);
    EXPECT_FLOAT_EQ(f.tau, 0.0f);
}

TEST(StopFrame, HoldUsesSnapshotAndJointGains)
{
    const auto f = stopFrame(Mode::Hold, joint(100, 2, 88), 0.40f, 0.42f, 8.0f);
    EXPECT_FLOAT_EQ(f.q, 0.42f);
    EXPECT_FLOAT_EQ(f.kp, 100.0f);
    EXPECT_FLOAT_EQ(f.kd, 2.0f);
    EXPECT_FLOAT_EQ(f.tau, 0.0f);
}

TEST(StopFrame, HoldClampsToTorqueWindow)
{
    // window = 88 / 100 = 0.88 rad
    const auto above = stopFrame(Mode::Hold, joint(100, 2, 88), 0.0f, 5.0f, 8.0f);
    EXPECT_FLOAT_EQ(above.q, 0.88f);
    const auto below = stopFrame(Mode::Hold, joint(100, 2, 88), 0.0f, -5.0f, 8.0f);
    EXPECT_FLOAT_EQ(below.q, -0.88f);
}

TEST(StopFrame, HoldParallelBecomesTorque)
{
    const auto f = stopFrame(Mode::Hold, joint(40, 2, 50, true), 0.1f, 0.2f, 8.0f);
    EXPECT_FLOAT_EQ(f.q, 0.1f);
    EXPECT_FLOAT_EQ(f.kp, 0.0f);
    EXPECT_FLOAT_EQ(f.kd, 2.0f);
    EXPECT_NEAR(f.tau, 40.0f * 0.1f, 1e-5);
}

TEST(StopFrame, HoldWithZeroKpDoesNotDivide)
{
    const auto f = stopFrame(Mode::Hold, joint(0, 2, 88), 0.3f, 1.0f, 8.0f);
    EXPECT_FLOAT_EQ(f.q, 0.3f);
    EXPECT_FLOAT_EQ(f.kp, 0.0f);
    EXPECT_FLOAT_EQ(f.kd, 2.0f);
}

TEST(StopFrame, NonFiniteMeasuredNeverReachesTheWire)
{
    const float_t nan = std::numeric_limits<float_t>::quiet_NaN();
    const float_t inf = std::numeric_limits<float_t>::infinity();
    for (const Mode mode : {Mode::Damp, Mode::Hold})
    {
        for (const float_t bad : {nan, inf, -inf})
        {
            const auto f = stopFrame(mode, joint(100, 2, 88), bad, 0.3f, 8.0f);
            EXPECT_FLOAT_EQ(f.q, 0.0f);
            EXPECT_FLOAT_EQ(f.kp, 0.0f);
            EXPECT_FLOAT_EQ(f.kd, 8.0f);
            EXPECT_FLOAT_EQ(f.tau, 0.0f);
        }
    }
}

TEST(StopFrame, NonFiniteSnapshotDampsThatJoint)
{
    const float_t nan = std::numeric_limits<float_t>::quiet_NaN();
    const auto f = stopFrame(Mode::Hold, joint(100, 2, 88, true), 0.3f, nan, 8.0f);
    EXPECT_FLOAT_EQ(f.q, 0.0f);
    EXPECT_FLOAT_EQ(f.kp, 0.0f);
    EXPECT_FLOAT_EQ(f.kd, 8.0f);
    EXPECT_FLOAT_EQ(f.tau, 0.0f);
}

// --- onset -------------------------------------------------------------------

TEST(Onset, FaultFiresOnceUntilItClears)
{
    Cause last = Cause::None;
    EXPECT_EQ(faultOnset(Cause::Imu, last), Cause::Imu);
    for (int i = 0; i < 10; ++i)
    {
        EXPECT_EQ(faultOnset(Cause::Imu, last), Cause::None);
    }
    EXPECT_EQ(faultOnset(Cause::StateLoss, last), Cause::StateLoss);   // a new kind fires
    EXPECT_EQ(faultOnset(Cause::None, last), Cause::None);
    EXPECT_EQ(faultOnset(Cause::Imu, last), Cause::Imu);               // back again fires
}

TEST(Onset, RemotePressFiresOnce)
{
    Cause last = Cause::None;
    EXPECT_EQ(remoteOnset(Cause::RemoteHold, last), Cause::RemoteHold);
    EXPECT_EQ(remoteOnset(Cause::RemoteHold, last), Cause::None);
    EXPECT_EQ(remoteOnset(Cause::None, last), Cause::None);
    EXPECT_EQ(remoteOnset(Cause::RemoteHold, last), Cause::RemoteHold);
}

TEST(Onset, RemoteHoldToDampEscalatesButNotBack)
{
    Cause last = Cause::None;
    remoteOnset(Cause::RemoteHold, last);
    EXPECT_EQ(remoteOnset(Cause::RemoteDamp, last), Cause::RemoteDamp);   // L1 added
    EXPECT_EQ(remoteOnset(Cause::RemoteHold, last), Cause::None);         // L1 released first
}

TEST(Onset, OperatorHoldSurvivesALastingFault)
{
    SafetyLatch latch;
    Cause last_fault = Cause::None;
    latch.trigger(Cause::ServiceDamp, true);
    latch.trigger(faultOnset(Cause::Imu, last_fault), false);
    latch.trigger(Cause::RemoteHold, false);
    for (int i = 0; i < 100; ++i)
    {
        latch.trigger(faultOnset(Cause::Imu, last_fault), false);   // fault still present
    }
    EXPECT_EQ(latch.mode(), Mode::Hold);
    EXPECT_EQ(latch.cause(), Cause::RemoteHold);
}

// --- remote ------------------------------------------------------------------

TEST(Remote, KeysFromBytesTwoAndThree)
{
    const uint8_t idle[40] = {85, 81, 0, 0, 0, 0, 0, 128};   // captured on the G1, no key held
    EXPECT_EQ(remoteKeys(idle, sizeof(idle)), 0);

    uint8_t pressed[40] = {85, 81};
    pressed[2] = KEY_L1 | KEY_R1;
    pressed[3] = static_cast<uint8_t>(KEY_LEFT >> 8);
    EXPECT_EQ(remoteKeys(pressed, sizeof(pressed)), KEY_L1 | KEY_R1 | KEY_LEFT);
}

TEST(Remote, ShortBufferIsNoKeys)
{
    const uint8_t raw[3] = {85, 81, 3};
    EXPECT_EQ(remoteKeys(raw, sizeof(raw)), 0);
    EXPECT_EQ(remoteKeys(nullptr, 40), 0);
}

TEST(Remote, CombosAndDampWins)
{
    const Config config;
    EXPECT_EQ(remoteCause(KEY_L1 | KEY_R1, config), Cause::RemoteDamp);
    EXPECT_EQ(remoteCause(KEY_R1 | KEY_R2, config), Cause::RemoteHold);
    EXPECT_EQ(remoteCause(KEY_L1 | KEY_R1 | KEY_R2, config), Cause::RemoteDamp);
    EXPECT_EQ(remoteCause(KEY_R1, config), Cause::None);
    EXPECT_EQ(remoteCause(KEY_L2 | KEY_B, config), Cause::None);
}

TEST(Remote, ParseCombo)
{
    EXPECT_EQ(parseCombo({"L1", "R1"}), KEY_L1 | KEY_R1);
    EXPECT_EQ(parseCombo({"r1", "r2"}), KEY_R1 | KEY_R2);
    EXPECT_EQ(parseCombo({"L1", "Z"}), 0);
    EXPECT_EQ(parseCombo({}), 0);
    EXPECT_FALSE(held(0xFFFF, 0));   // an empty combo never fires
}

TEST(Remote, ParseMode)
{
    Mode mode = Mode::Damp;
    EXPECT_TRUE(parseMode("hold", mode));
    EXPECT_EQ(mode, Mode::Hold);
    EXPECT_FALSE(parseMode("limp", mode));
}

// --- latch -------------------------------------------------------------------

TEST(Latch, StopWhileDisarmedDoesNothing)
{
    SafetyLatch latch;
    EXPECT_FALSE(latch.trigger(Cause::Stop, false).changed);
    EXPECT_EQ(latch.describe(false), "DISARMED");
}

TEST(Latch, EstopWhileDisarmedLatchesSilent)
{
    SafetyLatch latch;
    EXPECT_TRUE(latch.trigger(Cause::RemoteDamp, false).changed);
    EXPECT_TRUE(latch.latched());
    EXPECT_FALSE(latch.publishing());
    EXPECT_EQ(latch.describe(false), "LATCHED damp remote_damp silent");

    std::string why;
    ASSERT_TRUE(latch.reset(false, why));
    EXPECT_EQ(latch.state(), State::Idle);
    EXPECT_EQ(latch.describe(false), "DISARMED");
}

TEST(Latch, LatchRefusesArmingUntilReset)
{
    SafetyLatch latch = latchedFromTracking(Cause::ServiceDamp);
    EXPECT_TRUE(latch.publishing());
    EXPECT_FALSE(latch.commitArm(latch.generation()));

    std::string why;
    EXPECT_FALSE(latch.reset(true, why));
    EXPECT_EQ(why, "cause still present");
    EXPECT_TRUE(latch.latched());

    ASSERT_TRUE(latch.reset(false, why));
    EXPECT_EQ(latch.state(), State::Stopped);   // keeps publishing the stop frame
    EXPECT_TRUE(latch.publishing());
    EXPECT_TRUE(latch.commitArm(latch.generation()));
    EXPECT_EQ(latch.describe(true), "ARMED");
}

TEST(Latch, ResetWhenNotLatchedIsRefused)
{
    SafetyLatch latch;
    std::string why;
    EXPECT_FALSE(latch.reset(false, why));
    EXPECT_EQ(why, "not latched");
}

TEST(Latch, StopControlHoldsWithoutLatch)
{
    SafetyLatch latch;
    const auto t = latch.trigger(Cause::Stop, true);
    EXPECT_TRUE(t.changed);
    EXPECT_TRUE(t.snapshot);
    EXPECT_EQ(latch.state(), State::Stopped);
    EXPECT_EQ(latch.mode(), Mode::Hold);
    EXPECT_TRUE(latch.commitArm(latch.generation()));
}

TEST(Latch, StopControlModeFromConfig)
{
    Config config;
    config.stop = Mode::Damp;
    SafetyLatch latch(config);
    latch.trigger(Cause::Stop, true);
    EXPECT_EQ(latch.mode(), Mode::Damp);
}

TEST(Latch, SecondStopIsNoChange)
{
    SafetyLatch latch;
    latch.trigger(Cause::Stop, true);
    EXPECT_FALSE(latch.trigger(Cause::Stop, false).changed);
}

TEST(Latch, StopDoesNotUnlatch)
{
    SafetyLatch latch = latchedFromTracking(Cause::ServiceDamp);
    EXPECT_FALSE(latch.trigger(Cause::Stop, false).changed);
    EXPECT_TRUE(latch.latched());
}

TEST(Latch, OperatorHoldAfterDampReStiffens)
{
    SafetyLatch latch = latchedFromTracking(Cause::RemoteDamp);
    const auto t = latch.trigger(Cause::RemoteHold, false);
    EXPECT_TRUE(t.changed);
    EXPECT_TRUE(t.snapshot);
    EXPECT_EQ(latch.mode(), Mode::Hold);
    EXPECT_EQ(latch.cause(), Cause::RemoteHold);
}

TEST(Latch, OperatorDampAfterHold)
{
    SafetyLatch latch = latchedFromTracking(Cause::ServiceHold);
    EXPECT_TRUE(latch.trigger(Cause::ServiceDamp, false).changed);
    EXPECT_EQ(latch.mode(), Mode::Damp);
}

TEST(Latch, FaultNeverTurnsDampIntoHold)
{
    SafetyLatch latch = latchedFromTracking(Cause::ServiceDamp);
    EXPECT_FALSE(latch.trigger(Cause::CmdTimeout, false).changed);
    EXPECT_EQ(latch.mode(), Mode::Damp);
    EXPECT_EQ(latch.cause(), Cause::ServiceDamp);
}

TEST(Latch, FaultEscalatesHoldToDamp)
{
    SafetyLatch latch = latchedFromTracking(Cause::ServiceHold);
    EXPECT_TRUE(latch.trigger(Cause::Imu, false).changed);
    EXPECT_EQ(latch.mode(), Mode::Damp);
    EXPECT_EQ(latch.describe(false), "LATCHED damp imu");
}

TEST(Latch, FaultWhileStoppedLatches)
{
    SafetyLatch latch;
    latch.trigger(Cause::Stop, true);
    EXPECT_TRUE(latch.trigger(Cause::StateLoss, false).changed);
    EXPECT_TRUE(latch.latched());
    EXPECT_EQ(latch.mode(), Mode::Damp);
    EXPECT_TRUE(latch.publishing());
}

TEST(Latch, TimeoutHoldsAndLatches)
{
    SafetyLatch latch = latchedFromTracking(Cause::CmdTimeout);
    EXPECT_TRUE(latch.latched());
    EXPECT_EQ(latch.mode(), Mode::Hold);
}

TEST(Latch, HoldAfterStopHoldKeepsPose)
{
    SafetyLatch latch;
    latch.trigger(Cause::Stop, true);
    const auto t = latch.trigger(Cause::ServiceHold, false);
    EXPECT_TRUE(t.changed);      // Stopped -> Latched
    EXPECT_FALSE(t.snapshot);    // already holding: keep the first pose
}

TEST(Latch, RepeatedTriggerIsNoChange)
{
    SafetyLatch latch = latchedFromTracking(Cause::RemoteDamp);
    for (int i = 0; i < 100; ++i)
    {
        EXPECT_FALSE(latch.trigger(Cause::RemoteDamp, false).changed);
    }
}

TEST(Latch, TriggerDuringArmingVoidsIt)
{
    SafetyLatch latch;
    const uint64_t generation = latch.generation();
    latch.trigger(Cause::Imu, true);
    std::string why;
    latch.reset(false, why);
    EXPECT_FALSE(latch.commitArm(generation));
}

TEST(Latch, StopDuringArmingVoidsIt)
{
    SafetyLatch latch;
    latch.trigger(Cause::Stop, true);
    const uint64_t generation = latch.generation();
    latch.trigger(Cause::Stop, true);   // tracking had started again
    EXPECT_FALSE(latch.commitArm(generation));
}

// --- remote header -----------------------------------------------------------

TEST(Remote, HeaderMeansConnected)
{
    const uint8_t on[40] = {0x55, 0x51};
    const uint8_t off[40] = {};
    EXPECT_TRUE(remoteHeader(on, sizeof(on)));
    EXPECT_FALSE(remoteHeader(off, sizeof(off)));     // measured: all 40 bytes 0 with the remote off
}

TEST(Remote, ShortOrNullIsNotConnected)
{
    const uint8_t on[2] = {0x55, 0x51};
    EXPECT_FALSE(remoteHeader(on, 1));
    EXPECT_FALSE(remoteHeader(nullptr, 40));
}

// --- silence after state loss ------------------------------------------------

TEST(Silence, DueOnlyAfterStateLossPlusGrace)
{
    EXPECT_FALSE(silenceDue(true, true, STATE_LOSS_S + SILENCE_AFTER_S - 0.01));
    EXPECT_TRUE(silenceDue(true, true, STATE_LOSS_S + SILENCE_AFTER_S + 0.01));
    EXPECT_FALSE(silenceDue(false, true, 10.0));   // STOPPED is not latched: its fault latches first
    EXPECT_FALSE(silenceDue(true, false, 10.0));   // already silent
}

TEST(Silence, LatchedGoesSilentThenResetsToIdle)
{
    auto latch = latchedFromTracking(Cause::StateLoss);
    ASSERT_TRUE(latch.publishing());
    const auto generation = latch.generation();
    EXPECT_TRUE(latch.silence());
    EXPECT_FALSE(latch.publishing());
    EXPECT_TRUE(latch.latched());
    EXPECT_NE(latch.generation(), generation);
    EXPECT_FALSE(latch.silence());                 // no change the second time
    std::string why;
    ASSERT_TRUE(latch.reset(false, why));
    EXPECT_EQ(latch.state(), State::Idle);
}

TEST(Silence, StaysSilentThroughLaterTriggers)
{
    auto latch = latchedFromTracking(Cause::StateLoss);
    latch.silence();
    latch.trigger(Cause::ServiceHold, false);
    EXPECT_FALSE(latch.publishing());              // a restarted robot must not get old frames
    EXPECT_EQ(latch.mode(), Mode::Hold);
}

TEST(Silence, NotWhileIdleOrStopped)
{
    SafetyLatch idle;
    EXPECT_FALSE(idle.silence());
    SafetyLatch stopped;
    stopped.trigger(Cause::Stop, true);
    EXPECT_FALSE(stopped.silence());
    EXPECT_TRUE(stopped.publishing());
}
