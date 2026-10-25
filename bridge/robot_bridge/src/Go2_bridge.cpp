#include "Go2_bridge.hpp"
#include "gates.hpp"
#include "go2_frame.hpp"

using namespace std::chrono_literals;

sairol_bridge::Go2Bridge::Go2Bridge(rclcpp::Node::SharedPtr node) : BridgeCore(node)
{
    if (numJoint_ > go2::NUM_SLOTS)
    {
        RCLCPP_FATAL(nh->get_logger(), "num_joint %d: the Go2 frame holds %d motors", numJoint_, go2::NUM_SLOTS);
        throw ConfigError("num_joint past the Go2 frame");
    }
    lowCommandDesired_.motor_cmd.resize(numJoint_);
    lastCommand_.motor_cmd.resize(numJoint_);
    currentState_.motor_state.resize(numJoint_);
    cmdParams_.resize(numJoint_);

    lowStateSubscriber_ = nh->create_subscription<unitree_go::msg::LowState>(
        "/lowstate", 1, std::bind(&Go2Bridge::lowStateHandler_, this, std::placeholders::_1));

    // Let discovery settle: checked too early, an existing /lowcmd publisher is missed and nothing waits.
    rclcpp::sleep_for(1s);
    // rclcpp::ok(): after Ctrl-C the graph calls throw. main() then skips spin.
    while (rclcpp::ok() && !checkExternalPublisher_("/lowcmd"))
    {
        rclcpp::sleep_for(1s);
    }
    if (!rclcpp::ok())
    {
        return;
    }
    lowCommandPublisher_ = nh->create_publisher<unitree_go::msg::LowCmd>("/lowcmd", 10);

    RCLCPP_INFO(nh->get_logger(), "Waiting for publisher on topic /lowstate...");
    while (rclcpp::ok() && nh->count_publishers("/lowstate") == 0)
    {
        rclcpp::sleep_for(100ms);
    }
    if (!rclcpp::ok())
    {
        return;
    }
    // Runs while the context is still valid, so update_() keeps publishing the damp frames.
    nh->get_node_base_interface()->get_context()->add_pre_shutdown_callback([this] { dampBeforeShutdown_(); });

    createArmingServices_();
    // Arming checks /lowcmd once; this catches a sender that starts later (the sport service, a script).
    lowcmdWatchTimer_ = nh->create_wall_timer(1s, std::bind(&Go2Bridge::watchLowcmd_, this));
    startLinkWatch_();
    RCLCPP_INFO(nh->get_logger(), "Go2 bridge up: %d joints, DISARMED", numJoint_);
}

bool sairol_bridge::Go2Bridge::checkExternalPublisher_(const std::string &topic_name)
{
    const size_t publishers = nh->get_publishers_info_by_topic(topic_name).size();
    if (publishers > 0)
    {
        RCLCPP_ERROR_STREAM(nh->get_logger(), "Detected " << publishers << " publishers on " << topic_name
                            << ": release the sport service first (tools/robot-bringup.sh --robot go2)");
        return false;
    }
    return true;
}

bool sairol_bridge::Go2Bridge::robotAllowsArming_(std::string &why)
{
    // This bridge's own publisher counts once.
    const size_t publishers = nh->count_publishers("/lowcmd");
    if (publishers > 1)
    {
        why = std::to_string(publishers - 1) + " other /lowcmd publisher(s), e.g. the sport service "
              "after a restart: release it (tools/g1-release-lowcmd.py --stand-down)";
        return false;
    }
    return true;
}

sairol_bridge::estop::Cause sairol_bridge::Go2Bridge::robotFault_()
{
    return nh->count_publishers("/lowcmd") > 1 ? estop::Cause::OtherLowcmd : estop::Cause::None;
}

void sairol_bridge::Go2Bridge::watchLowcmd_()
{
    const bool sending = controlStarted_ || stopPublishing_;
    const estop::Cause fault =
        estop::faultOnset(sending ? robotFault_() : estop::Cause::None, lastLowcmdFault_);
    if (fault != estop::Cause::None)
    {
        RCLCPP_ERROR(nh->get_logger(), "Another node publishes /lowcmd while this bridge sends: two senders fight");
        enterStop_(fault);
    }
}

void sairol_bridge::Go2Bridge::stop()
{
    if (controlThread_.joinable())
    {
        controlThread_.join();
    }
    RCLCPP_INFO(nh->get_logger(), "Go2 Bridge stopped.");
}

void sairol_bridge::Go2Bridge::lowStateHandler_(unitree_go::msg::LowState::SharedPtr msg)
{
    std::unique_lock<std::mutex> lock(mutex_);
    last_state_time_ = steadyClock_.now();
    // motor_state is a fixed [20]; currentState_ holds numJoint_ (12). Copy only those.
    for (int i = 0; i < numJoint_; ++i)
    {
        currentState_.motor_state[i].q = msg->motor_state[i].q;
        currentState_.motor_state[i].dq = msg->motor_state[i].dq;
        currentState_.motor_state[i].ddq = msg->motor_state[i].ddq;
        currentState_.motor_state[i].tau_est = msg->motor_state[i].tau_est;
    }
    imu_.rpy = msg->imu_state.rpy;
    imu_.gyroscope = msg->imu_state.gyroscope;
    imu_.accelerometer = msg->imu_state.accelerometer;
    imu_.quaternion = msg->imu_state.quaternion;
    auto state = robotState_(msg->tick);
    lock.unlock();

    if (!stateLogged_)
    {
        stateLogged_ = true;
        RCLCPP_INFO(nh->get_logger(), "Go2 type: %d motors", numJoint_);
    }
    noteLink_(msg->tick);

    // The remote rides on LowState, as on the G1: same header and key bytes.
    const uint8_t *raw = msg->wireless_remote.data();
    const size_t raw_size = msg->wireless_remote.size();
    const uint16_t keys = estop::remoteKeys(raw, raw_size);
    remoteKeys_ = keys;
    const bool header = estop::remoteHeader(raw, raw_size);
    if (header)
    {
        lastRemoteHeader_ = steadyClock_.now().seconds();
    }
    const estop::Cause remote = estop::remoteOnset(estop::remoteCause(keys, estopConfig_), lastRemote_);
    if (remote != estop::Cause::None)
    {
        enterStop_(remote);
    }

    state.remote.connected = header;
    state.remote.pressed = estop::keyNames(keys, *keyTable_);
    const estop::Sticks sticks = estop::remoteSticks(raw, raw_size);
    state.remote.lx = sticks.lx;
    state.remote.ly = sticks.ly;
    state.remote.rx = sticks.rx;
    state.remote.ry = sticks.ry;
    robotStatePublisher_->publish(state);
}

void sairol_bridge::Go2Bridge::publishLowCommand_()
{
    rclcpp::Time current = steadyClock_.now();

    float_t phase = 1.0f;
    if (tFinal_ - tStart_ > 1e-6) {
        phase = (current.seconds() - tStart_) / (tFinal_ - tStart_);
    }
    phase = std::clamp(phase, 0.0f, 1.0f);

    unitree_go::msg::LowCmd unitree_cmd;
    bool finite = true;
    for (int i = 0; i < numJoint_; ++i)
    {
        auto &cmd = unitree_cmd.motor_cmd[i];
        auto &last_cmd = lastCommand_.motor_cmd[i];
        auto &joint_info = joints_[i];

        if (1e-6 < cmdInterpOrder_ && cmdInterpOrder_ < 1.0 - 1e-6)
        {
            cmd.q = last_cmd.q * cmdInterpOrder_ + cmdParams_[i].q_0 * (1 - cmdInterpOrder_);
            cmd.dq = last_cmd.dq * cmdInterpOrder_ + cmdParams_[i].dq_0 * (1 - cmdInterpOrder_);
        }
        else
        {
            cmd.q = cmdParams_[i].q_0 + cmdParams_[i].q_1 * phase;
            cmd.dq = cmdParams_[i].dq_0 + cmdParams_[i].dq_1 * phase;
        }

        // Gates 1 and 2 (docs/psi0_gates.md): the joint range, then the target speed.
        if (cmd.q < joint_info.q_min || cmd.q > joint_info.q_max)
        {
            RCLCPP_WARN_THROTTLE(nh->get_logger(), steadyClock_, 1000,
                                 "joint %d target %.3f outside [%.3f, %.3f]: clamped",
                                 i, cmd.q, joint_info.q_min, joint_info.q_max);
        }
        cmd.q = gates::limitTarget(cmd.q, last_cmd.q, joint_info, controlDt_);

        cmd.kp = cmdParams_[i].kp_0 + cmdParams_[i].kp_1 * phase;
        cmd.kd = cmdParams_[i].kd_0 + cmdParams_[i].kd_1 * phase;
        cmd.dq = std::clamp(cmd.dq, -joint_info.dq_limit, joint_info.dq_limit);

        if (torqueControl_)
        {
            cmd.tau = cmd.kp * (cmd.q - currentState_.motor_state[i].q) +
                      cmd.kd * (cmd.dq - currentState_.motor_state[i].dq) + cmdParams_[i].tau_0;
            cmd.tau = std::clamp(cmd.tau, -joint_info.tau_limit, joint_info.tau_limit);
            cmd.kp = 0.0;
            cmd.kd = 0.0;
        }
        else
        {
            cmd.tau = cmdParams_[i].tau_0 + cmdParams_[i].tau_1 * phase;
        }
        // Gate 4: the torque window, feedforward included; at kp 0 the measured q, never inf or NaN.
        cmd.q = gates::torqueWindow(cmd.q, currentState_.motor_state[i].q, currentState_.motor_state[i].dq,
                                    cmd.dq, cmd.kp, cmd.kd, cmd.tau, joint_info.tau_limit);

        last_cmd.q = cmd.q;
        last_cmd.dq = cmd.dq;
        last_cmd.kp = cmd.kp;
        last_cmd.kd = cmd.kd;
        last_cmd.tau = cmd.tau;
        finite = finite && gates::finiteCommand(cmd.q, cmd.dq, cmd.kp, cmd.kd, cmd.tau);
    }
    if (!finite)
    {
        invalidFrame_ = true;   // update_() latches state_invalid once mutex_ is free
        return;
    }
    go2::finishFrame(unitree_cmd, numJoint_);
    lowCommandPublisher_->publish(unitree_cmd);
}

// mutex_ held (update_). Bypasses publishLowCommand_: its q clamp divides by kp.
void sairol_bridge::Go2Bridge::publishStopCommand_()
{
    unitree_go::msg::LowCmd unitree_cmd;
    const estop::Mode mode = latch_.mode();
    for (int i = 0; i < numJoint_; ++i)
    {
        const auto frame = estop::stopFrame(mode, joints_[i], currentState_.motor_state[i].q,
                                            holdQ_[i], estopConfig_.damp_kd);
        auto &cmd = unitree_cmd.motor_cmd[i];
        cmd.q = frame.q;
        cmd.dq = frame.dq;
        cmd.kp = frame.kp;
        cmd.kd = frame.kd;
        cmd.tau = frame.tau;

        // A re-arm interpolates from lastCommand_.
        auto &last_cmd = lastCommand_.motor_cmd[i];
        last_cmd.q = frame.q;
        last_cmd.dq = frame.dq;
        last_cmd.kp = frame.kp;
        last_cmd.kd = frame.kd;
        last_cmd.tau = frame.tau;
    }
    go2::finishFrame(unitree_cmd, numJoint_);
    lowCommandPublisher_->publish(unitree_cmd);
}

bool sairol_bridge::Go2Bridge::initControl_(bridge_interface::msg::RobotCmd default_cmd)
{
    {
        // Gate 2 ramps from lastCommand_: all zeros at boot, which would pull every joint toward 0.
        std::lock_guard<std::mutex> lock(mutex_);
        for (int i = 0; i < numJoint_; ++i)
        {
            const float_t q = std::clamp(currentState_.motor_state[i].q, joints_[i].q_min, joints_[i].q_max);
            lastCommand_.motor_cmd[i].q = q;
            lastCommand_.motor_cmd[i].dq = 0.0;
            lastCommand_.motor_cmd[i].tau = 0.0;
            lowCommandDesired_.motor_cmd[i].q = q;
            lowCommandDesired_.motor_cmd[i].dq = 0.0;
            lowCommandDesired_.motor_cmd[i].tau = 0.0;
            lowCommandDesired_.motor_cmd[i].kp = joints_[i].kp;
            lowCommandDesired_.motor_cmd[i].kd = joints_[i].kd;
        }
    }
    calculateInterpolationParams_(0.0, 1, true);

    controlStarted_ = true;
    receivedCmd_ = true;
    rclcpp::Rate rate(100);
    rate.sleep();
    RCLCPP_INFO(nh->get_logger(), "Control initialized successfully.");

    if (default_cmd.motor_cmd.size() == static_cast<size_t>(numJoint_))
    {
        for (int i = 0; i < numJoint_; ++i)
        {
            lowCommandDesired_.motor_cmd[i].q = std::clamp(default_cmd.motor_cmd[i].q, joints_[i].q_min, joints_[i].q_max);
            lowCommandDesired_.motor_cmd[i].kp = default_cmd.motor_cmd[i].kp;
            lowCommandDesired_.motor_cmd[i].kd = default_cmd.motor_cmd[i].kd;
        }
    }
    else
    {
        for (int i = 0; i < numJoint_; ++i)
        {
            lowCommandDesired_.motor_cmd[i].q = ready_q_[i];
            lowCommandDesired_.motor_cmd[i].kp = joints_[i].kp;
            lowCommandDesired_.motor_cmd[i].kd = joints_[i].kd;
        }
    }
    calculateInterpolationParams_(duration_, 1, true);
    return true;
}

void sairol_bridge::Go2Bridge::finishControl_()
{
    RCLCPP_INFO(nh->get_logger(), "finishControl_ called from Go2Bridge");
}

// Silence at once leaves the Go2's motors passive: a standing robot sinks. Damp frames first.
void sairol_bridge::Go2Bridge::dampBeforeShutdown_()
{
    enterStop_(estop::Cause::Shutdown);   // also voids an arming in progress
    if (!stopPublishing_)
    {
        return;
    }
    RCLCPP_WARN(nh->get_logger(), "Shutdown: damp frames for %.1f s, then /lowcmd goes silent",
                estop::SHUTDOWN_DAMP_S);
    std::this_thread::sleep_for(std::chrono::duration<double>(estop::SHUTDOWN_DAMP_S));
}
