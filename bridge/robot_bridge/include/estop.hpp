// estop.hpp ------------------------------------------------------------------
// E-stop state, stop frames and remote keys. No ROS: tests/test_estop.cpp
// covers all of it. Design: docs/estop.md.
#pragma once

#include "joint.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace sairol_bridge::estop
{
    enum class Mode : uint8_t { Damp, Hold };

    enum class Cause : uint8_t
    {
        None,
        RemoteDamp, RemoteHold,     // operator, mode fixed
        ServiceDamp, ServiceHold,   // operator, mode fixed
        Shutdown,                   // operator: SIGINT or SIGTERM while sending; damp, final
        Stop,                       // /stop_control, the only cause that does not latch
        Imu, StateLoss, StateInvalid, CmdTimeout,  // faults
        OtherLowcmd                 // fault, damp fixed: another /lowcmd publisher while sending
    };

    // Same bits as WirelessKey_H1_G1, plus F1/F2.
    enum Key : uint16_t
    {
        KEY_R1 = 1 << 0, KEY_L1 = 1 << 1, KEY_START = 1 << 2, KEY_SELECT = 1 << 3,
        KEY_R2 = 1 << 4, KEY_L2 = 1 << 5, KEY_F1 = 1 << 6, KEY_F2 = 1 << 7,
        KEY_A = 1 << 8, KEY_B = 1 << 9, KEY_X = 1 << 10, KEY_Y = 1 << 11,
        KEY_UP = 1 << 12, KEY_RIGHT = 1 << 13, KEY_DOWN = 1 << 14, KEY_LEFT = 1 << 15
    };

    // /lowstate older than this is a state_loss fault.
    constexpr double STATE_LOSS_S = 0.2;
    // A latch stops publishing this long after /lowstate went missing: a robot
    // that restarts meanwhile must not get old frames (docs/psi0_gates.md).
    constexpr double SILENCE_AFTER_S = 1.0;
    // The remote counts as connected while its header arrived this recently.
    constexpr double REMOTE_TIMEOUT_S = 0.5;
    // A clean exit while sending keeps the damp frames up this long, then goes silent.
    constexpr double SHUTDOWN_DAMP_S = 1.0;

    struct Config
    {
        float_t damp_kd{8.0f};
        uint16_t combo_damp{KEY_R1 | KEY_R2};  // one hand
        uint16_t combo_hold{KEY_L1 | KEY_R1};
        Mode stop{Mode::Hold};
        Mode imu{Mode::Damp};
        Mode state_loss{Mode::Damp};
        Mode state_invalid{Mode::Damp};
        Mode cmd_timeout{Mode::Hold};
        bool require_remote{false};   // arming services refused without the remote; g1_node: true if unset
    };

    bool isOperator(Cause cause);
    bool latches(Cause cause);
    Mode modeFor(const Config &config, Cause cause);

    const char *name(Mode mode);
    const char *name(Cause cause);
    bool parseMode(const std::string &text, Mode &mode);
    // Key names as in the config ("L1", "R1", "UP", ...). 0 on an unknown or empty list.
    uint16_t parseCombo(const std::vector<std::string> &keys);

    // LowState.wireless_remote: bytes 2-3, little-endian. 0 if the buffer is short.
    uint16_t remoteKeys(const uint8_t *raw, size_t size);
    // Bytes 0-1 are 0x55 0x51 while a remote is connected; all 40 are 0 without one.
    bool remoteHeader(const uint8_t *raw, size_t size);
    // A publishing latch goes silent once /lowstate is older than STATE_LOSS_S + SILENCE_AFTER_S.
    bool silenceDue(bool latched, bool publishing, double state_age_s);
    bool held(uint16_t keys, uint16_t combo);
    // Damp wins when both combos are held.
    Cause remoteCause(uint16_t keys, const Config &config);

    // A held combo or a lasting fault fires once, so an operator hold is not
    // undone 1000 times a second. `last` is the caller's per-source memory.
    // Fault: fires when it starts or changes kind.
    Cause faultOnset(Cause current, Cause &last);
    // Remote: fires on a press from no combo, and on hold -> damp (R2 added).
    // Damp -> hold (R2 released first) does not fire.
    Cause remoteOnset(Cause current, Cause &last);

    struct MotorFrame
    {
        float_t q{0}, dq{0}, kp{0}, kd{0}, tau{0};
    };

    // One motor's stop command. Never divides by kp: hold needs joint.kp > 0,
    // which BridgeCore checks at load. A non-finite q_meas (or q_hold in hold)
    // never reaches the wire: that joint gets damp at q 0.
    MotorFrame stopFrame(Mode mode, const Joint &joint, float_t q_meas, float_t q_hold, float_t damp_kd);

    struct Transition
    {
        bool changed{false};
        bool snapshot{false};   // take a new hold pose from the measured q
    };

    // Stop state beside BridgeCore's controlStarted_ (tracking). Not thread-safe:
    // BridgeCore guards it with its mutex.
    class SafetyLatch
    {
    public:
        enum class State : uint8_t { Idle, Stopped, Latched };

        explicit SafetyLatch(const Config &config = Config()) : config_(config) {}

        // was_tracking: controlStarted_ was true when the trigger arrived.
        Transition trigger(Cause cause, bool was_tracking);
        // Latched -> Stopped, or Idle if it was silent. Refused while not latched
        // or while the cause is still present.
        bool reset(bool cause_present, std::string &why);
        // Arming services: refused while latched, and when a trigger landed after
        // `generation` was read.
        bool commitArm(uint64_t generation);
        // Latched and publishing -> latched and silent; reset then leaves it Idle.
        // Stays silent through later triggers. True if it changed.
        bool silence();

        State state() const { return state_; }
        Mode mode() const { return mode_; }
        Cause cause() const { return cause_; }
        bool latched() const { return state_ == State::Latched; }
        bool publishing() const { return publishing_; }
        uint64_t generation() const { return generation_; }
        std::string describe(bool tracking) const;

    private:
        Config config_;
        State state_{State::Idle};
        Mode mode_{Mode::Damp};
        Cause cause_{Cause::None};
        bool publishing_{false};
        uint64_t generation_{0};
    };

} // namespace sairol_bridge::estop
