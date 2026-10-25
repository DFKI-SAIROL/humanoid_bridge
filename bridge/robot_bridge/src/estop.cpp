#include "estop.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <utility>

namespace sairol_bridge::estop
{
    namespace
    {
        // A fault may turn hold into damp, never damp into hold.
        Mode escalate(Mode current, Mode wanted)
        {
            return (current == Mode::Damp || wanted == Mode::Damp) ? Mode::Damp : Mode::Hold;
        }

        const std::pair<const char *, uint16_t> KEY_NAMES[] = {
            {"R1", KEY_R1}, {"L1", KEY_L1}, {"START", KEY_START}, {"SELECT", KEY_SELECT},
            {"R2", KEY_R2}, {"L2", KEY_L2}, {"F1", KEY_F1}, {"F2", KEY_F2},
            {"A", KEY_A}, {"B", KEY_B}, {"X", KEY_X}, {"Y", KEY_Y},
            {"UP", KEY_UP}, {"RIGHT", KEY_RIGHT}, {"DOWN", KEY_DOWN}, {"LEFT", KEY_LEFT},
        };
    } // namespace

    bool isOperator(Cause cause)
    {
        return cause == Cause::RemoteDamp || cause == Cause::RemoteHold ||
               cause == Cause::ServiceDamp || cause == Cause::ServiceHold || cause == Cause::Shutdown;
    }

    bool latches(Cause cause)
    {
        return cause != Cause::None && cause != Cause::Stop;
    }

    Mode modeFor(const Config &config, Cause cause)
    {
        switch (cause)
        {
        case Cause::RemoteHold:
        case Cause::ServiceHold:
            return Mode::Hold;
        case Cause::Stop:
            return config.stop;
        case Cause::Imu:
            return config.imu;
        case Cause::StateLoss:
            return config.state_loss;
        case Cause::StateInvalid:
            return config.state_invalid;
        case Cause::CmdTimeout:
            return config.cmd_timeout;
        default:
            return Mode::Damp;
        }
    }

    const char *name(Mode mode)
    {
        return mode == Mode::Hold ? "hold" : "damp";
    }

    const char *name(Cause cause)
    {
        switch (cause)
        {
        case Cause::RemoteDamp: return "remote_damp";
        case Cause::RemoteHold: return "remote_hold";
        case Cause::ServiceDamp: return "service_damp";
        case Cause::ServiceHold: return "service_hold";
        case Cause::Shutdown: return "shutdown";
        case Cause::Stop: return "stop";
        case Cause::Imu: return "imu";
        case Cause::StateLoss: return "state_loss";
        case Cause::StateInvalid: return "state_invalid";
        case Cause::CmdTimeout: return "cmd_timeout";
        case Cause::OtherLowcmd: return "other_lowcmd";
        case Cause::RobotRestart: return "robot_restart";
        default: return "none";
        }
    }

    bool parseMode(const std::string &text, Mode &mode)
    {
        if (text == "damp") { mode = Mode::Damp; return true; }
        if (text == "hold") { mode = Mode::Hold; return true; }
        return false;
    }

    uint16_t parseCombo(const std::vector<std::string> &keys)
    {
        uint16_t combo = 0;
        for (auto key : keys)
        {
            std::transform(key.begin(), key.end(), key.begin(),
                           [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
            const auto found = std::find_if(std::begin(KEY_NAMES), std::end(KEY_NAMES),
                                            [&](const auto &entry) { return key == entry.first; });
            if (found == std::end(KEY_NAMES))
            {
                return 0;
            }
            combo |= found->second;
        }
        return combo;
    }

    uint16_t remoteKeys(const uint8_t *raw, size_t size)
    {
        if (raw == nullptr || size < 4)
        {
            return 0;
        }
        return static_cast<uint16_t>(raw[2] | (raw[3] << 8));
    }

    bool remoteHeader(const uint8_t *raw, size_t size)
    {
        return raw != nullptr && size >= 2 && raw[0] == 0x55 && raw[1] == 0x51;
    }

    std::vector<std::string> keyNames(uint16_t keys)
    {
        std::vector<std::string> names;
        for (int bit = 0; bit < 16; ++bit)
        {
            const uint16_t key = static_cast<uint16_t>(1u << bit);
            if (!(keys & key))
            {
                continue;
            }
            const auto found = std::find_if(std::begin(KEY_NAMES), std::end(KEY_NAMES),
                                            [&](const auto &entry) { return entry.second == key; });
            names.emplace_back(found->first);
        }
        return names;
    }

    Sticks remoteSticks(const uint8_t *raw, size_t size)
    {
        Sticks sticks;
        if (raw == nullptr || size < 24)
        {
            return sticks;
        }
        // Little-endian on the robot and on every host this builds for.
        std::memcpy(&sticks.lx, raw + 4, 4);
        std::memcpy(&sticks.rx, raw + 8, 4);
        std::memcpy(&sticks.ry, raw + 12, 4);
        std::memcpy(&sticks.ly, raw + 20, 4);
        return sticks;
    }

    bool silenceDue(bool latched, bool publishing, double state_age_s)
    {
        return latched && publishing && state_age_s > STATE_LOSS_S + SILENCE_AFTER_S;
    }

    bool held(uint16_t keys, uint16_t combo)
    {
        return combo != 0 && (keys & combo) == combo;
    }

    Cause remoteCause(uint16_t keys, const Config &config)
    {
        if (held(keys, config.combo_damp))
        {
            return Cause::RemoteDamp;
        }
        if (held(keys, config.combo_hold))
        {
            return Cause::RemoteHold;
        }
        return Cause::None;
    }

    Cause faultOnset(Cause current, Cause &last)
    {
        const Cause fired = (current != Cause::None && current != last) ? current : Cause::None;
        last = current;
        return fired;
    }

    Cause remoteOnset(Cause current, Cause &last)
    {
        const bool press = current != Cause::None && last == Cause::None;
        const bool escalate = current == Cause::RemoteDamp && last == Cause::RemoteHold;
        last = current;
        return (press || escalate) ? current : Cause::None;
    }

    MotorFrame stopFrame(Mode mode, const Joint &joint, float_t q_meas, float_t q_hold, float_t damp_kd)
    {
        MotorFrame frame;
        frame.q = q_meas;

        if (!std::isfinite(q_meas) || (mode == Mode::Hold && !std::isfinite(q_hold)))
        {
            frame.q = 0;
            frame.kd = damp_kd;
            return frame;
        }

        if (mode == Mode::Damp)
        {
            frame.kd = damp_kd;
            return frame;
        }

        frame.kd = joint.kd;
        if (joint.kp <= 0)
        {
            return frame;   // load rejects this; no spring rather than a division by zero
        }

        // Same torque window as publishLowCommand_, with dq terms at 0.
        const float_t window = joint.tau_limit / joint.kp;
        frame.q = std::clamp(q_hold, q_meas - window, q_meas + window);
        frame.kp = joint.kp;

        // Same conversion as publishLowCommand_ for parallel joints.
        if (joint.if_parallel_joint)
        {
            frame.tau = std::clamp((frame.q - q_meas) * joint.kp, -joint.tau_limit, joint.tau_limit);
            frame.q = q_meas;
            frame.kp = 0;
        }
        return frame;
    }

    Transition SafetyLatch::trigger(Cause cause, bool was_tracking)
    {
        Transition result;
        // Shutdown is final: the process is exiting, so nothing changes its mode or re-arms.
        if (cause == Cause::None || cause_ == Cause::Shutdown)
        {
            return result;
        }
        // Any trigger that ends tracking voids an arming in progress.
        if (was_tracking)
        {
            ++generation_;
        }

        const Mode wanted = modeFor(config_, cause);
        const bool latching = latches(cause);
        State next_state = state_;
        Mode next_mode = mode_;
        bool next_publishing = publishing_;

        switch (state_)
        {
        case State::Idle:
            if (!was_tracking && !latching)
            {
                return result;   // /stop_control while disarmed
            }
            next_state = latching ? State::Latched : State::Stopped;
            next_mode = wanted;
            next_publishing = was_tracking;
            break;
        case State::Stopped:
        case State::Latched:
            if (!latching)
            {
                return result;   // /stop_control while already stopped
            }
            next_state = State::Latched;
            next_mode = isOperator(cause) ? wanted : escalate(mode_, wanted);
            break;
        }

        // Shutdown always takes the cause, even over a damp latch: the final rule hangs on it.
        // robot_restart too, over a silent damp state_loss: /bridge_state names the restart.
        const bool takes_cause = cause == Cause::Shutdown || (cause == Cause::RobotRestart && cause_ != cause);
        if (!takes_cause && next_state == state_ && next_mode == mode_ && next_publishing == publishing_)
        {
            return result;
        }

        result.changed = true;
        result.snapshot = next_mode == Mode::Hold && (state_ == State::Idle || mode_ != Mode::Hold);
        state_ = next_state;
        mode_ = next_mode;
        publishing_ = next_publishing;
        cause_ = cause;
        ++generation_;
        return result;
    }

    bool SafetyLatch::reset(bool cause_present, std::string &why)
    {
        if (state_ != State::Latched)
        {
            why = "not latched";
            return false;
        }
        if (cause_ == Cause::Shutdown)
        {
            why = "shutting down";
            return false;
        }
        if (cause_present)
        {
            why = "cause still present";
            return false;
        }
        state_ = publishing_ ? State::Stopped : State::Idle;
        return true;
    }

    bool SafetyLatch::commitArm(uint64_t generation)
    {
        if (state_ == State::Latched || generation != generation_)
        {
            return false;
        }
        state_ = State::Idle;
        publishing_ = false;
        cause_ = Cause::None;
        return true;
    }

    bool SafetyLatch::silence()
    {
        if (state_ != State::Latched || !publishing_)
        {
            return false;
        }
        publishing_ = false;
        ++generation_;
        return true;
    }

    std::string SafetyLatch::describe(bool tracking) const
    {
        switch (state_)
        {
        case State::Latched:
            return std::string("LATCHED ") + name(mode_) + " " + name(cause_) + (publishing_ ? "" : " silent");
        case State::Stopped:
            return std::string("STOPPED ") + name(mode_) + " " + name(cause_);
        default:
            return tracking ? "ARMED" : "DISARMED";
        }
    }

    bool LinkWatch::onState(double now_s, uint32_t tick)
    {
        const double gap = now_s - lastState_;
        const uint32_t last_tick = tick_;
        const bool first = !seen_;
        seen_ = true;
        lastState_ = now_s;
        tick_ = tick;
        if (first || (!lost_ && gap <= STATE_LOSS_S))
        {
            return false;
        }
        lost_ = false;
        back_ = true;
        gap_ = gap;
        // A running robot's tick moves on through a link drop; a booted one starts again near 0.
        restarted_ = restarted_ || tick <= last_tick;
        return true;
    }

    bool LinkWatch::poll(double now_s)
    {
        const double age = now_s - lastState_;
        if (!seen_ || age <= STATE_LOSS_S || (lost_ && std::floor(age) <= std::floor(lostShown_)))
        {
            return false;
        }
        lost_ = true;
        lostShown_ = age;
        return true;
    }

    bool LinkWatch::clear()
    {
        if (lost_ || restarted_ || !back_)
        {
            return false;
        }
        back_ = false;
        return true;
    }

    std::string LinkWatch::describe() const
    {
        char text[48];
        if (lost_)
        {
            std::snprintf(text, sizeof(text), "LOST %.1f", lostShown_);
        }
        else if (restarted_)
        {
            std::snprintf(text, sizeof(text), "RESTARTED %.1f", gap_);
        }
        else if (back_)
        {
            std::snprintf(text, sizeof(text), "BACK %.1f no_restart", gap_);
        }
        else
        {
            return "OK";
        }
        return text;
    }

} // namespace sairol_bridge::estop
