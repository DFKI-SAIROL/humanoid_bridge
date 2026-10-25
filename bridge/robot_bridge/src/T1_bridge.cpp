#include "T1_bridge.hpp"
#include "gates.hpp"

#include <cmath>

using namespace std::chrono_literals;

namespace
{
    // booster_interface/ImuState has rpy only: the quaternion (w, x, y, z) from ZYX rpy.
    std::array<float, 4> quatFromRpy(const std::array<float, 3> &rpy)
    {
        const double cr = std::cos(rpy[0] / 2), sr = std::sin(rpy[0] / 2);
        const double cp = std::cos(rpy[1] / 2), sp = std::sin(rpy[1] / 2);
        const double cy = std::cos(rpy[2] / 2), sy = std::sin(rpy[2] / 2);
        return {static_cast<float>(cr * cp * cy + sr * sp * sy), static_cast<float>(sr * cp * cy - cr * sp * sy),
                static_cast<float>(cr * sp * cy + sr * cp * sy), static_cast<float>(cr * cp * sy - sr * sp * cy)};
    }
} // namespace

sairol_bridge::T1Bridge::T1Bridge(rclcpp::Node::SharedPtr node) : BridgeCore(node)
{
    lowCommandDesired_.motor_cmd.resize(numJoint_);
    lastCommand_.motor_cmd.resize(numJoint_);
    currentState_.motor_state.resize(numJoint_);
    cmdParams_.resize(numJoint_);

    client_ = nh->create_client<booster_interface::srv::RpcService>("booster_rpc_service");
    // rclcpp::ok(): after Ctrl-C the graph calls throw. main() then skips spin.
    while (rclcpp::ok() && !client_->wait_for_service(1s))
    {
        RCLCPP_INFO(nh->get_logger(), "Waiting for booster_rpc_service...");
    }
    if (!rclcpp::ok())
    {
        return;
    }

    lowStateSubscriber_ = nh->create_subscription<booster_interface::msg::LowState>(
        "/low_state", 1, std::bind(&T1Bridge::lowStateHandler_, this, std::placeholders::_1));
    remoteSubscriber_ = nh->create_subscription<booster_interface::msg::RemoteControllerState>(
        "/remote_controller_state", 10, std::bind(&T1Bridge::remoteHandler_, this, std::placeholders::_1));

    lowCommandPublisher_ = nh->create_publisher<booster_interface::msg::LowCmd>("/joint_ctrl", 1);

    RCLCPP_INFO(nh->get_logger(), "Waiting for publisher on topic /low_state...");
    while (rclcpp::ok() && nh->count_publishers("/low_state") == 0)
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
    // Arming checks /joint_ctrl once; this catches a sender that starts later.
    jointCtrlWatchTimer_ = nh->create_wall_timer(1s, std::bind(&T1Bridge::watchJointCtrl_, this));
    link_ = estop::LinkWatch(false);   // no tick on /low_state: a reboot cannot be told from a drop
    startLinkWatch_();
    RCLCPP_INFO(nh->get_logger(), "T1 bridge up: %d joints, DISARMED", numJoint_);
}

void sairol_bridge::T1Bridge::stop()
{
    if (controlThread_.joinable())
    {
        controlThread_.join();
    }
    if (rclcpp::ok())   // after a shutdown dampBeforeShutdown_ asked already
    {
        switchMode_(booster::RobotMode::Damping);
    }
    RCLCPP_INFO(nh->get_logger(), "T1 Bridge stopped.");
}

void sairol_bridge::T1Bridge::lowStateHandler_(booster_interface::msg::LowState::SharedPtr msg)
{
    std::unique_lock<std::mutex> lock(mutex_);
    last_state_time_ = steadyClock_.now();

    // motor_state_serial is an unbounded sequence; currentState_ holds numJoint_. Bound the copy.
    const size_t count = msg->motor_state_serial.size();
    const size_t copy_count = std::min(count, static_cast<size_t>(numJoint_));
    if (count != static_cast<size_t>(numJoint_))
    {
        RCLCPP_WARN_ONCE(nh->get_logger(),
                         "LowState carries %zu serial motor states, robot_bridge is configured for %d joints; "
                         "the first %zu are used",
                         count, numJoint_, copy_count);
    }
    for (size_t i = 0; i < copy_count; ++i)
    {
        currentState_.motor_state[i].q = msg->motor_state_serial[i].q;
        currentState_.motor_state[i].dq = msg->motor_state_serial[i].dq;
        currentState_.motor_state[i].ddq = msg->motor_state_serial[i].ddq;
        currentState_.motor_state[i].tau_est = msg->motor_state_serial[i].tau_est;
    }
    imu_.rpy = msg->imu_state.rpy;
    imu_.gyroscope = msg->imu_state.gyro;
    imu_.accelerometer = msg->imu_state.acc;
    imu_.quaternion = quatFromRpy(msg->imu_state.rpy);
    auto state = robotState_(0);   // booster_interface/LowState has no tick
    lock.unlock();

    if (!stateLogged_)
    {
        stateLogged_ = true;
        RCLCPP_INFO(nh->get_logger(), "T1 type: %zu serial motors", count);
    }
    noteLink_(0);

    const bool connected = steadyClock_.now().seconds() - lastRemoteHeader_ <= estop::REMOTE_TIMEOUT_S;
    const uint16_t keys = remoteKeys_;
    state.remote.connected = connected;
    state.remote.pressed = connected ? estop::keyNames(keys, *keyTable_) : std::vector<std::string>{};
    robotStatePublisher_->publish(state);
}

void sairol_bridge::T1Bridge::remoteHandler_(booster_interface::msg::RemoteControllerState::SharedPtr msg)
{
    // Every message counts as the remote being there (hardware check: does the T1 send only on events?).
    lastRemoteHeader_ = steadyClock_.now().seconds();
    estop::T1Pad pad;
    pad.a = msg->a, pad.b = msg->b, pad.x = msg->x, pad.y = msg->y;
    pad.lb = msg->lb, pad.rb = msg->rb, pad.lt = msg->lt, pad.rt = msg->rt;
    pad.back = msg->back, pad.start = msg->start, pad.ls = msg->ls, pad.rs = msg->rs;
    pad.up = msg->hat_u || msg->hat_lu || msg->hat_ru, pad.down = msg->hat_d || msg->hat_ld || msg->hat_rd;
    pad.left = msg->hat_l || msg->hat_lu || msg->hat_ld, pad.right = msg->hat_r || msg->hat_ru || msg->hat_rd;
    const uint16_t keys = estop::t1KeyMask(pad);
    remoteKeys_ = keys;
    const estop::Cause remote = estop::remoteOnset(estop::remoteCause(keys, estopConfig_), lastRemote_);
    if (remote != estop::Cause::None)
    {
        enterStop_(remote);
    }
}

bool sairol_bridge::T1Bridge::robotAllowsArming_(std::string &why)
{
    // This bridge's own publisher counts once.
    const size_t publishers = nh->count_publishers("/joint_ctrl");
    if (publishers > 1)
    {
        why = std::to_string(publishers - 1) + " other /joint_ctrl publisher(s): stop them first";
        return false;
    }
    return true;
}

sairol_bridge::estop::Cause sairol_bridge::T1Bridge::robotFault_()
{
    return nh->count_publishers("/joint_ctrl") > 1 ? estop::Cause::OtherLowcmd : estop::Cause::None;
}

void sairol_bridge::T1Bridge::watchJointCtrl_()
{
    const bool sending = controlStarted_ || stopPublishing_;
    const estop::Cause fault =
        estop::faultOnset(sending ? robotFault_() : estop::Cause::None, lastJointCtrlFault_);
    if (fault != estop::Cause::None)
    {
        RCLCPP_ERROR(nh->get_logger(), "Another node publishes /joint_ctrl while this bridge sends: two senders fight");
        enterStop_(fault);
    }
}

void sairol_bridge::T1Bridge::publishLowCommand_()
{
    rclcpp::Time current = steadyClock_.now();

    float_t phase = 1.0f;
    if (tFinal_ - tStart_ > 1e-6) {
        phase = (current.seconds() - tStart_) / (tFinal_ - tStart_);
    }
    phase = std::clamp(phase, 0.0f, 1.0f);

    booster_interface::msg::LowCmd booster_cmd;
    booster_cmd.motor_cmd.resize(numJoint_);
    booster_cmd.cmd_type = booster_interface::msg::LowCmd::CMD_TYPE_SERIAL;

    bool finite = true;
    for (int i = 0; i < numJoint_; ++i)
    {
        auto &cmd = booster_cmd.motor_cmd[i];
        auto &last_cmd = lastCommand_.motor_cmd[i];
        auto &joint_info = joints_[i];

        cmd.mode = 0;
        cmd.weight = 1.0;

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
        const bool parallel = receivedCmd_ && joint_info.if_parallel_joint;
        cmd.q = gates::torqueWindow(cmd.q, currentState_.motor_state[i].q, currentState_.motor_state[i].dq,
                                    cmd.dq, cmd.kp, cmd.kd, parallel ? 0.0f : cmd.tau, joint_info.tau_limit);

        last_cmd.q = cmd.q;
        last_cmd.dq = cmd.dq;
        last_cmd.kp = cmd.kp;
        last_cmd.kd = cmd.kd;
        last_cmd.tau = cmd.tau;

        if (parallel)
        {
            cmd.tau = std::clamp((last_cmd.q - currentState_.motor_state[i].q) * cmd.kp,
                                 -joint_info.tau_limit, joint_info.tau_limit);
            cmd.q = currentState_.motor_state[i].q;
            cmd.kp = 0.0;
        }
        finite = finite && gates::finiteCommand(cmd.q, cmd.dq, cmd.kp, cmd.kd, cmd.tau);
    }
    if (!finite)
    {
        invalidFrame_ = true;   // update_() latches state_invalid once mutex_ is free
        return;
    }
    lowCommandPublisher_->publish(booster_cmd);
}

// mutex_ held (update_). Bypasses publishLowCommand_: its q clamp divides by kp.
void sairol_bridge::T1Bridge::publishStopCommand_()
{
    booster_interface::msg::LowCmd booster_cmd;
    booster_cmd.motor_cmd.resize(numJoint_);
    booster_cmd.cmd_type = booster_interface::msg::LowCmd::CMD_TYPE_SERIAL;

    const estop::Mode mode = latch_.mode();
    for (int i = 0; i < numJoint_; ++i)
    {
        const auto frame = estop::stopFrame(mode, joints_[i], currentState_.motor_state[i].q,
                                            holdQ_[i], estopConfig_.damp_kd);
        auto &cmd = booster_cmd.motor_cmd[i];
        cmd.mode = 0;
        cmd.weight = 1.0;
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
    lowCommandPublisher_->publish(booster_cmd);
}

bool sairol_bridge::T1Bridge::initControl_(bridge_interface::msg::RobotCmd default_cmd)
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

    // Custom mode follows /joint_ctrl: the hold above streams before the switch lands.
    controlStarted_ = true;
    receivedCmd_ = true;
    if (!switchMode_(booster::RobotMode::Custom))
    {
        controlStarted_ = false;
        RCLCPP_ERROR(nh->get_logger(), "booster_rpc_service is gone: not armed");
        return false;
    }
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

void sairol_bridge::T1Bridge::finishControl_()
{
    RCLCPP_INFO(nh->get_logger(), "finishControl_ called from T1Bridge");
}

bool sairol_bridge::T1Bridge::switchMode_(booster::RobotMode mode)
{
    if (!client_ || !client_->service_is_ready())
    {
        RCLCPP_ERROR(nh->get_logger(), "booster_rpc_service not ready: %s mode not requested", booster::name(mode));
        return false;
    }
    auto req = std::make_shared<booster_interface::srv::RpcService::Request>();
    req->msg.api_id = booster::API_CHANGE_MODE;
    req->msg.body = booster::changeModeBody(mode);
    const std::string mode_name = booster::name(mode);
    client_->async_send_request(
        req,
        [this, mode_name](rclcpp::Client<booster_interface::srv::RpcService>::SharedFuture future)
        {
            try
            {
                const auto response = future.get();
                RCLCPP_INFO(nh->get_logger(), "%s mode: status %ld %s", mode_name.c_str(),
                            static_cast<long>(response->msg.status), response->msg.body.c_str());
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(nh->get_logger(), "%s mode: %s", mode_name.c_str(), e.what());
            }
        });
    RCLCPP_INFO(nh->get_logger(), "requested %s mode", mode_name.c_str());
    return true;
}

// Silence at once leaves the T1 in custom mode without targets: damp frames, then Booster's damping.
// Here, while the context is valid: the request goes out before the executor stops.
void sairol_bridge::T1Bridge::dampBeforeShutdown_()
{
    enterStop_(estop::Cause::Shutdown);   // also voids an arming in progress
    if (stopPublishing_)
    {
        RCLCPP_WARN(nh->get_logger(), "Shutdown: damp frames for %.1f s, then damping mode",
                    estop::SHUTDOWN_DAMP_S);
        std::this_thread::sleep_for(std::chrono::duration<double>(estop::SHUTDOWN_DAMP_S));
    }
    switchMode_(booster::RobotMode::Damping);
}
