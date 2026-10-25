#include "G1_bridge.hpp"
#include "gates.hpp"
// #include "unitree_hg/message_utils.hpp"

using namespace std::chrono_literals;

sairol_bridge::G1Bridge::G1Bridge(rclcpp::Node::SharedPtr node) : BridgeCore(node)
{

    lowCommandDesired_.motor_cmd.resize(numJoint_);
    lastCommand_.motor_cmd.resize(numJoint_);
    currentState_.motor_state.resize(numJoint_);
    cmdParams_.resize(numJoint_);

    lowStateSubscriber_ = nh->create_subscription<unitree_hg::msg::LowState>(
        "/lowstate", 1, std::bind(&sairol_bridge::G1Bridge::lowStateHandler_, this, std::placeholders::_1));

    // Let discovery settle: checked too early, an existing /lowcmd publisher is missed and nothing waits.
    rclcpp::sleep_for(std::chrono::seconds(1));
    // rclcpp::ok(): after Ctrl-C the graph calls throw, and the bridge aborted. main() then skips spin.
    while (rclcpp::ok() && !checkExternalPublisher_("/lowcmd"))
    {
        rclcpp::sleep_for(std::chrono::milliseconds(1000));
    }
    if (!rclcpp::ok())
    {
        return;
    }

    lowCommandPublisher_ = nh->create_publisher<unitree_hg::msg::LowCmd>("/lowcmd", 10); // /joint_ctrl

    // Waiting for publisher on topic lowstate
    RCLCPP_INFO(nh->get_logger(), "Waiting for publisher on topic /lowstate...");

    while (rclcpp::ok() && nh->count_publishers("/lowstate") == 0)
    {
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
    if (!rclcpp::ok())
    {
        return;
    }
    // Runs while the context is still valid, so update_() keeps publishing the damp frames.
    nh->get_node_base_interface()->get_context()->add_pre_shutdown_callback([this] { dampBeforeShutdown_(); });

    // Here, not last: the hand block below returns early without hand parameters.
    createArmingServices_();
    // Arming checks /lowcmd once; this catches a sender that starts later (motion service, a script).
    lowcmdWatchTimer_ = nh->create_wall_timer(1s, std::bind(&G1Bridge::watchLowcmd_, this));
    startLinkWatch_();

    hands_[HAND_LEFT].name = "left";
    hands_[HAND_RIGHT].name = "right";

    if (!loadHandParameters_())
    {
        RCLCPP_WARN(nh->get_logger(),
                    "No Dex3 hand parameters in this config; hand control is off. "
                    "The body path is unaffected.");
        return;
    }

    for (int side = 0; side < NUM_HANDS; ++side)
    {
        auto &hand = hands_[side];
        const std::string cmd_topic = "/dex3/" + hand.name + "/cmd";

        // Warn, unlike /lowcmd's blocking loop: a contested hand topic must not
        // stop the body bridge from coming up. This sees ROS 2 publishers only --
        // a raw unitree_sdk2py publisher such as `dex3_probe --command` is
        // invisible to the ROS graph and will not be caught here.
        if (nh->count_publishers(cmd_topic) > 0)
        {
            RCLCPP_WARN(nh->get_logger(),
                        "Another ROS 2 publisher already holds %s; two publishers will fight.",
                        cmd_topic.c_str());
        }

        hand.state_time = steadyClock_.now();
        hand.desired.motor_cmd.resize(numHandJoint_);
        hand.last.motor_cmd.resize(numHandJoint_);
        hand.params.resize(numHandJoint_);

        hand.cmd_pub = nh->create_publisher<unitree_hg::msg::HandCmd>(cmd_topic, 10);
        hand.state_pub = nh->create_publisher<bridge_interface::msg::HandState>("/hand_state/" + hand.name, 1);

        hand.state_sub = nh->create_subscription<unitree_hg::msg::HandState>(
            "/dex3/" + hand.name + "/state", 1,
            [this, side](unitree_hg::msg::HandState::SharedPtr msg) { handStateHandler_(side, msg); });

        hand.cmd_sub = nh->create_subscription<bridge_interface::msg::HandCmd>(
            "/hand_cmd/" + hand.name, 1,
            [this, side](bridge_interface::msg::HandCmd::SharedPtr msg) { handCmdCallBack_(side, msg); });
    }

    startHandControlService_ = nh->create_service<std_srvs::srv::Trigger>(
        "start_hand_control",
        std::bind(&G1Bridge::startHandControlServiceCB_, this, std::placeholders::_1, std::placeholders::_2));

    stopHandControlService_ = nh->create_service<std_srvs::srv::Trigger>(
        "stop_hand_control",
        std::bind(&G1Bridge::stopHandControlServiceCB_, this, std::placeholders::_1, std::placeholders::_2));

    handTimer_ = nh->create_wall_timer(std::chrono::duration<double>(handControlDt_),
                                       std::bind(&G1Bridge::publishHandCommand_, this));

    RCLCPP_INFO(nh->get_logger(),
                "Dex3 hand path ready: /hand_cmd/{left,right} at %.0f Hz, %d joints per hand.",
                1.0 / handControlDt_, numHandJoint_);
}


bool sairol_bridge::G1Bridge::checkExternalPublisher_(std::string topic_name)
{
    auto publishers_info = nh->get_publishers_info_by_topic(topic_name);
    int publisher_count = publishers_info.size();
    if (publisher_count > 0)
    {
        RCLCPP_ERROR_STREAM(nh->get_logger(),
                     "Detected " << publisher_count << " publishers on " << topic_name.c_str() << 
                     ". Please change to the debug mode by pressing L2 + R2");
        return false;
    }
    return true;
}


bool sairol_bridge::G1Bridge::robotAllowsArming_(std::string &why)
{
    // This bridge's own publisher counts once.
    const size_t publishers = nh->count_publishers("/lowcmd");
    if (publishers > 1)
    {
        why = std::to_string(publishers - 1) + " other /lowcmd publisher(s), e.g. the robot's motion service "
              "after a restart: release it (L2 + R2, or tools/g1-release-lowcmd.py)";
        return false;
    }
    return true;
}

sairol_bridge::estop::Cause sairol_bridge::G1Bridge::robotFault_()
{
    // This bridge's own publisher counts once.
    return nh->count_publishers("/lowcmd") > 1 ? estop::Cause::OtherLowcmd : estop::Cause::None;
}

void sairol_bridge::G1Bridge::watchLowcmd_()
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

void sairol_bridge::G1Bridge::stop()
{
    if (handTimer_)
    {
        handTimer_->cancel();
    }

    // Stop the control thread if it's running
    if (controlThread_.joinable())
    {
        controlThread_.join();
    }

    RCLCPP_INFO(nh->get_logger(), "G1 Bridge stopped.");
}

void sairol_bridge::G1Bridge::lowStateHandler_(unitree_hg::msg::LowState::SharedPtr msg)
{
    // The IMU trip lives in checkState_ (control thread); this handler only copies.
    std::unique_lock<std::mutex> lock(mutex_);

        // update mode machine
    if (mode_machine_ != msg->mode_machine) {
      if (mode_machine_ == 0) {
        RCLCPP_INFO(nh->get_logger(), "G1 type: %d", unsigned(msg->mode_machine));
      }
      mode_machine_ = msg->mode_machine;
    }

    // Update the last state time using the same clock source
    last_state_time_ = steadyClock_.now();

    // unitree_hg::msg::LowState declares `MotorState[35] motor_state`, a fixed
    // array, so size() is always 35 whatever the robot. currentState_ is a
    // bridge_interface::msg::LowState, whose motor_state is an unbounded
    // sequence resized once to numJoint_ (29 for the G1, per G1_config.yaml).
    // Copying the full 35 therefore writes six MotorState past the end of that
    // allocation on every message. Bound the copy, and say so once.
    const size_t motor_state_count = msg->motor_state.size();
    const size_t expected_motor_state_count = static_cast<size_t>(numJoint_);
    const size_t copy_count = std::min(motor_state_count, expected_motor_state_count);

    if (motor_state_count != expected_motor_state_count)
    {
        RCLCPP_WARN_ONCE(
            nh->get_logger(),
            "LowState carries %zu motor states, but robot_bridge is configured for %zu joints. "
            "Only the first %zu entries will be used.",
            motor_state_count,
            expected_motor_state_count,
            copy_count);
    }

    for (size_t i = 0; i < copy_count; ++i)
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

    noteLink_(msg->tick);

    // The remote rides on LowState at 500 Hz; /wirelesscontroller stays silent in debug mode.
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
    state.remote.pressed = estop::keyNames(keys);
    const estop::Sticks sticks = estop::remoteSticks(raw, raw_size);
    state.remote.lx = sticks.lx;
    state.remote.ly = sticks.ly;
    state.remote.rx = sticks.rx;
    state.remote.ry = sticks.ry;
    robotStatePublisher_->publish(state);
}

void sairol_bridge::G1Bridge::publishLowCommand_()
{
    rclcpp::Time current = steadyClock_.now();

    float_t phase = 1.0f;
    if (tFinal_ - tStart_ > 1e-6) {
        phase = (current.seconds() - tStart_) / (tFinal_ - tStart_);
    }

    phase = std::clamp(phase, 0.0f, 1.0f); // Ensure phase is between 0 and 1

    unitree_hg::msg::LowCmd unitree_cmd;
    unitree_cmd.mode_machine = mode_machine_;
    unitree_cmd.mode_pr = PRorAB::PR;

    bool finite = true;
    for (int i = 0; i < numJoint_; ++i)
    {
        auto &cmd = unitree_cmd.motor_cmd[i];
        auto &last_cmd = lastCommand_.motor_cmd[i];
        auto &joint_info = joints_[i];

        unitree_cmd.motor_cmd[i].mode = 1; // Enable motor

        if (1e-6 < cmdInterpOrder_ && cmdInterpOrder_ < 1.0 - 1e-6)
        {
            // Low pass filter for cmd
            cmd.q = last_cmd.q * cmdInterpOrder_ + cmdParams_[i].q_0 * (1 - cmdInterpOrder_);
            cmd.dq = last_cmd.dq * cmdInterpOrder_ + cmdParams_[i].dq_0 * (1 - cmdInterpOrder_);
        }
        else {
            // Interpolation
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
            cmd.tau = cmd.kp * (cmd.q - currentState_.motor_state[i].q) + cmd.kd * (cmd.dq - currentState_.motor_state[i].dq) + cmdParams_[i].tau_0;
            cmd.tau = std::clamp(cmd.tau, -joint_info.tau_limit, joint_info.tau_limit);
            cmd.kp = 0.0;
            cmd.kd = 0.0;
        }
        else
        {
            cmd.tau = cmdParams_[i].tau_0 + cmdParams_[i].tau_1 * phase;
        }  
        // Gate 4: the torque window, feedforward included; at kp 0 the measured q, never inf or NaN.
        // A parallel joint sends its P part as tau below and drops this tau, so its window leaves it out.
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
            cmd.tau = std::clamp((last_cmd.q - currentState_.motor_state[i].q) * cmd.kp, -joint_info.tau_limit, joint_info.tau_limit);
            cmd.q = currentState_.motor_state[i].q;
            cmd.kp = 0.0;
        }   
        finite = finite && gates::finiteCommand(cmd.q, cmd.dq, cmd.kp, cmd.kd, cmd.tau);
    }
    // A NaN /lowstate after checkState_'s copy reaches kp-0 and parallel joints: send nothing.
    if (!finite)
    {
        invalidFrame_ = true;   // update_() latches state_invalid once mutex_ is free
        return;
    }
    get_crc(unitree_cmd);
    lowCommandPublisher_->publish(unitree_cmd);
}

bool sairol_bridge::G1Bridge::initControl_(bridge_interface::msg::RobotCmd default_cmd)
{
    {
        // Gate 2 ramps from lastCommand_: all zeros at boot, which would pull every joint toward 0.
        std::lock_guard<std::mutex> lock(mutex_);
        // dq and tau too: the last /robot_cmd's would otherwise ride on the hold, with no timeout.
        for (int i = 0; i < numJoint_; ++i)
        {
            // Gate 1 clamps every frame anyway; a pose resting past a limit (g1_sim's waist
            // pitch at 0.529) would otherwise log "outside [...]: clamped" every second.
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
    
    if (default_cmd.motor_cmd.size() == numJoint_)
    {
        for (size_t i = 0; i < numJoint_; ++i)
        {
            // Gate 9 admits 0.1 rad past the range: a measured pose from /lowstate.
            lowCommandDesired_.motor_cmd[i].q = std::clamp(default_cmd.motor_cmd[i].q, joints_[i].q_min, joints_[i].q_max);
            lowCommandDesired_.motor_cmd[i].kp = default_cmd.motor_cmd[i].kp;
            lowCommandDesired_.motor_cmd[i].kd = default_cmd.motor_cmd[i].kd;
        }
    }
    else 
    {
        for (size_t i = 0; i < numJoint_; ++i)
        {
            lowCommandDesired_.motor_cmd[i].q = ready_q_[i];
            lowCommandDesired_.motor_cmd[i].kp = joints_[i].kp;
            lowCommandDesired_.motor_cmd[i].kd = joints_[i].kd;
        }
    }

    calculateInterpolationParams_(duration_, 1, true);

    return true;
}

// mutex_ held (update_). Bypasses publishLowCommand_: its q clamp divides by kp.
void sairol_bridge::G1Bridge::publishStopCommand_()
{
    unitree_hg::msg::LowCmd unitree_cmd;
    unitree_cmd.mode_machine = mode_machine_;
    unitree_cmd.mode_pr = PRorAB::PR;

    const estop::Mode mode = latch_.mode();
    for (int i = 0; i < numJoint_; ++i)
    {
        const auto frame = estop::stopFrame(mode, joints_[i], currentState_.motor_state[i].q,
                                            holdQ_[i], estopConfig_.damp_kd);
        auto &cmd = unitree_cmd.motor_cmd[i];
        cmd.mode = 1; // Enable motor
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
    get_crc(unitree_cmd);
    lowCommandPublisher_->publish(unitree_cmd);
}

void sairol_bridge::G1Bridge::finishControl_() {
    RCLCPP_INFO(nh->get_logger(), "finishControl_ called from G1Bridge");
}

// Silence at once drops a standing robot about 0.4 s later (docs/estop.md, G0).
void sairol_bridge::G1Bridge::dampBeforeShutdown_()
{
    enterStop_(estop::Cause::Shutdown);   // also voids an arming in progress
    if (!stopPublishing_)
    {
        return;   // DISARMED, or a latch already silent: stays silent
    }
    RCLCPP_WARN(nh->get_logger(), "Shutdown: damp frames for %.1f s, then /lowcmd goes silent",
                estop::SHUTDOWN_DAMP_S);
    std::this_thread::sleep_for(std::chrono::duration<double>(estop::SHUTDOWN_DAMP_S));
}




// --- Dex3 hands -------------------------------------------------------------
//
// A path of its own, beside the body path rather than inside it. num_joint stays
// 29: the hands are a separate device, on their own state topic, at their own
// rate. Everything below runs on the executor thread, which a single-threaded
// spin serialises, so none of it locks against the body's control thread.

bool sairol_bridge::G1Bridge::loadHandParameters_()
{
    if (!nh->get_parameter("num_hand_joint", numHandJoint_) ||
        !nh->get_parameter("hand_control_dt", handControlDt_) ||
        !nh->get_parameter("hand_state_timeout", handStateTimeout_) ||
        !nh->get_parameter("hand_temperature_limit", handTemperatureLimit_) ||
        !nh->get_parameter("hand_kp_min", handKpMin_) ||
        !nh->get_parameter("hand_kp_max", handKpMax_) ||
        !nh->get_parameter("hand_kd_min", handKdMin_) ||
        !nh->get_parameter("hand_kd_max", handKdMax_) ||
        !nh->get_parameter("hand_ready_q", handReadyQ_))
    {
        return false;
    }

    if (numHandJoint_ <= 0 || handControlDt_ <= 0.0)
    {
        RCLCPP_ERROR(nh->get_logger(), "num_hand_joint and hand_control_dt must be positive.");
        return false;
    }
    if (handKpMax_ < handKpMin_ || handKdMax_ < handKdMin_)
    {
        RCLCPP_ERROR(nh->get_logger(), "hand gain ranges are inverted.");
        return false;
    }
    if (static_cast<int>(handReadyQ_.size()) != numHandJoint_)
    {
        RCLCPP_ERROR(nh->get_logger(), "hand_ready_q has %ld entries, expected %d.",
                     handReadyQ_.size(), numHandJoint_);
        return false;
    }

    const char *name_param[NUM_HANDS] = {"left_hand_joint_names", "right_hand_joint_names"};
    for (int side = 0; side < NUM_HANDS; ++side)
    {
        std::vector<std::string> joint_names;
        if (!nh->get_parameter(name_param[side], joint_names))
        {
            RCLCPP_ERROR(nh->get_logger(), "Failed to get '%s' from parameters", name_param[side]);
            return false;
        }
        if (static_cast<int>(joint_names.size()) != numHandJoint_)
        {
            RCLCPP_ERROR(nh->get_logger(), "'%s' has %ld names, expected %d.",
                         name_param[side], joint_names.size(), numHandJoint_);
            return false;
        }

        hands_[side].joint_names = joint_names;
        auto &joints = hands_[side].joints;
        joints.clear();
        for (const auto &name : joint_names)
        {
            Joint joint_info;
            bool ret = true;
            ret = ret && nh->get_parameter(name + ".idx", joint_info.idx);
            ret = ret && nh->get_parameter(name + ".q_min", joint_info.q_min);
            ret = ret && nh->get_parameter(name + ".q_max", joint_info.q_max);
            ret = ret && nh->get_parameter(name + ".dq_limit", joint_info.dq_limit);
            ret = ret && nh->get_parameter(name + ".tau_limit", joint_info.tau_limit);
            ret = ret && nh->get_parameter(name + ".kp", joint_info.kp);
            ret = ret && nh->get_parameter(name + ".kd", joint_info.kd);
            if (!ret)
            {
                RCLCPP_ERROR(nh->get_logger(), "Hand joint '%s' is missing parameters.", name.c_str());
                return false;
            }
            // Loud here rather than a wrong clamp later: these come from the
            // Dex3 URDFs by hand, and the two hands are mirrored.
            if (joint_info.q_max < joint_info.q_min || joint_info.dq_limit < 0 ||
                joint_info.tau_limit < 0 || joint_info.kp < 0 || joint_info.kd < 0)
            {
                RCLCPP_ERROR(nh->get_logger(), "Hand joint '%s' has invalid limits.", name.c_str());
                return false;
            }
            joints.push_back(joint_info);
        }

        for (int i = 0; i < numHandJoint_; ++i)
        {
            if (handReadyQ_[i] < joints[i].q_min || handReadyQ_[i] > joints[i].q_max)
            {
                RCLCPP_ERROR(nh->get_logger(),
                             "hand_ready_q[%d] = %.4f is outside %s's range [%.4f, %.4f].",
                             i, handReadyQ_[i], joint_names[i].c_str(), joints[i].q_min, joints[i].q_max);
                return false;
            }
        }
    }

    RCLCPP_INFO(nh->get_logger(), "Loaded %d Dex3 joint limits per hand", numHandJoint_);
    RCLCPP_INFO(nh->get_logger(), "Hand kp range: [%.3f, %.3f], kd range: [%.3f, %.3f]",
                handKpMin_, handKpMax_, handKdMin_, handKdMax_);
    return true;
}

void sairol_bridge::G1Bridge::handStateHandler_(int side, unitree_hg::msg::HandState::SharedPtr msg)
{
    auto &hand = hands_[side];
    hand.state = *msg;
    hand.state_time = steadyClock_.now();
    hand.state_seen = true;

    bridge_interface::msg::HandState state;
    state.stamp = nh->now();
    state.joint_names = hand.joint_names;
    const size_t n = std::min(hand.joint_names.size(), msg->motor_state.size());
    for (size_t i = 0; i < n; ++i)
    {
        state.q.push_back(msg->motor_state[i].q);
        state.dq.push_back(msg->motor_state[i].dq);
        state.tau_est.push_back(msg->motor_state[i].tau_est);
    }
    state.joint_names.resize(n);
    hand.state_pub->publish(state);
}

bool sairol_bridge::G1Bridge::checkHandState_(int side)
{
    auto &hand = hands_[side];

    if (!hand.state_seen)
    {
        RCLCPP_ERROR_THROTTLE(nh->get_logger(), steadyClock_, 2000,
                              "%s hand: nothing has arrived on /dex3/%s/state.",
                              hand.name.c_str(), hand.name.c_str());
        return false;
    }

    const double dt_state = (steadyClock_.now() - hand.state_time).seconds();
    if (dt_state > handStateTimeout_)
    {
        RCLCPP_ERROR_THROTTLE(nh->get_logger(), steadyClock_, 2000,
                              "%s hand signal lost! No HandState for %.2f s (limit %.2f s).",
                              hand.name.c_str(), dt_state, handStateTimeout_);
        return false;
    }

    // unitree_hg/HandState carries an unbounded MotorState[], unlike LowState's
    // fixed [35], so the length is whatever the device sent. Bound every read.
    if (static_cast<int>(hand.state.motor_state.size()) < numHandJoint_)
    {
        RCLCPP_ERROR_THROTTLE(nh->get_logger(), steadyClock_, 2000,
                              "%s hand state has %ld motors, expected at least %d.",
                              hand.name.c_str(), hand.state.motor_state.size(), numHandJoint_);
        return false;
    }

    for (int i = 0; i < numHandJoint_; ++i)
    {
        const auto &motor = hand.state.motor_state[i];

        if (!std::isfinite(motor.q) || !std::isfinite(motor.dq) || !std::isfinite(motor.tau_est))
        {
            RCLCPP_ERROR(nh->get_logger(), "%s hand motor %d state contains invalid (NaN/Inf) values.",
                         hand.name.c_str(), i);
            return false;
        }

        if (std::abs(motor.dq) > hand.joints[i].dq_limit)
        {
            RCLCPP_ERROR(nh->get_logger(), "%s hand motor %d dq (%.3f) exceeds limit (%.3f).",
                         hand.name.c_str(), i, motor.dq, hand.joints[i].dq_limit);
            return false;
        }

        // The body path has no equivalent: a stalled finger is the failure a
        // hand actually has, and the Dex3 reports two temperatures per motor.
        for (const auto &temperature : motor.temperature)
        {
            if (static_cast<float_t>(temperature) > handTemperatureLimit_)
            {
                RCLCPP_ERROR(nh->get_logger(), "%s hand motor %d is at %d degrees (limit %.0f).",
                             hand.name.c_str(), i, static_cast<int>(temperature), handTemperatureLimit_);
                return false;
            }
        }
    }

    return true;
}

bool sairol_bridge::G1Bridge::checkHandCommand_(int side, bridge_interface::msg::HandCmd::SharedPtr message)
{
    auto &hand = hands_[side];

    if (static_cast<int>(message->motor_cmd.size()) != numHandJoint_)
    {
        RCLCPP_ERROR(nh->get_logger(), "%s hand command: motor_cmd size mismatch. Expected %d, got %ld.",
                     hand.name.c_str(), numHandJoint_, message->motor_cmd.size());
        return false;
    }

    if (!std::isfinite(message->duration) || message->duration < 0.0)
    {
        RCLCPP_ERROR(nh->get_logger(), "%s hand command: duration %.3f is not a non-negative number.",
                     hand.name.c_str(), message->duration);
        return false;
    }

    bool clipped = false;
    for (size_t i = 0; i < static_cast<size_t>(numHandJoint_); ++i)
    {
        const auto &joint_info = hand.joints[i];
        if (!checkMotorCmd_(message->motor_cmd[i], joint_info,
                            handKpMin_, handKpMax_, handKdMin_, handKdMax_, i, clipped))
        {
            return false;
        }

        // Unlike the body path, clamp q outright. A finger has a hard URDF range
        // and no torque-implied bound to fall back on.
        auto &cmd = message->motor_cmd[i];
        if (cmd.q < joint_info.q_min || cmd.q > joint_info.q_max)
        {
            RCLCPP_WARN_ONCE(nh->get_logger(), "Clipping %s hand motor_cmd[%lu] q: %.3f",
                             hand.name.c_str(), i, cmd.q);
            cmd.q = std::clamp(cmd.q, joint_info.q_min, joint_info.q_max);
            clipped = true;
        }
    }

    if (clipped)
    {
        RCLCPP_WARN_THROTTLE(nh->get_logger(), steadyClock_, 2000,
                             "%s hand: some motors were clipped to stay within safety limits",
                             hand.name.c_str());
    }

    return true;
}

void sairol_bridge::G1Bridge::handCmdCallBack_(int side, bridge_interface::msg::HandCmd::SharedPtr message)
{
    auto &hand = hands_[side];

    if (!handStarted_)
    {
        RCLCPP_WARN_ONCE(nh->get_logger(),
                         "Hand control not started. Call /start_hand_control first.");
        return;
    }

    if (!hand.live)
    {
        RCLCPP_WARN_THROTTLE(nh->get_logger(), steadyClock_, 2000,
                             "%s hand was released by a guard; ignoring commands until "
                             "/start_hand_control is called again.", hand.name.c_str());
        return;
    }

    if (!checkHandCommand_(side, message))
    {
        RCLCPP_ERROR(nh->get_logger(), "%s hand command check failed; command dropped.",
                     hand.name.c_str());
        return;
    }

    hand.desired = *message;

    // Always a linear ramp from what was last published. The body path's
    // interpolation_order is deliberately absent here: its order-0 low-pass buys
    // nothing at 100 Hz and is the confusing half of that knob.
    for (int i = 0; i < numHandJoint_; ++i)
    {
        const auto &target = hand.desired.motor_cmd[i];
        const auto &last = hand.last.motor_cmd[i];
        auto &params = hand.params[i];

        params.q_0 = last.q;
        params.q_1 = target.q - last.q;
        params.dq_0 = last.dq;
        params.dq_1 = target.dq - last.dq;
        params.kp_0 = last.kp;
        params.kp_1 = target.kp - last.kp;
        params.kd_0 = last.kd;
        params.kd_1 = target.kd - last.kd;
        params.tau_0 = target.tau;
        params.tau_1 = 0.0;
    }

    hand.t_start = steadyClock_.now().seconds();
    hand.t_final = hand.t_start + message->duration;
    hand.t_valid = message->hold_position ? INF_ : hand.t_final + 0.2;
}

void sairol_bridge::G1Bridge::publishOneHand_(int side, bool limp)
{
    auto &hand = hands_[side];
    if (!hand.cmd_pub)
    {
        return;
    }

    unitree_hg::msg::HandCmd out;
    // unitree_hg/HandCmd carries an unbounded MotorCmd[], unlike LowCmd's fixed
    // [35]. Without this resize every index below is out of bounds.
    out.motor_cmd.resize(numHandJoint_);

    float_t phase = 1.0f;
    if (hand.t_final - hand.t_start > 1e-6)
    {
        phase = static_cast<float_t>((steadyClock_.now().seconds() - hand.t_start) /
                                     (hand.t_final - hand.t_start));
    }
    phase = std::clamp(phase, 0.0f, 1.0f);

    for (int i = 0; i < numHandJoint_; ++i)
    {
        auto &cmd = out.motor_cmd[i];
        auto &last = hand.last.motor_cmd[i];
        cmd.mode = handMode_(i);

        if (limp)
        {
            // Zero gains and zero feedforward is what makes the finger slack; q
            // is carried only so a recording shows where the hand was. Non-finite: 0, as in estop::stopFrame.
            cmd.q = (static_cast<int>(hand.state.motor_state.size()) > i &&
                     std::isfinite(hand.state.motor_state[i].q))
                        ? hand.state.motor_state[i].q
                        : 0.0f;
            cmd.dq = 0.0f;
            cmd.tau = 0.0f;
            cmd.kp = 0.0f;
            cmd.kd = 0.0f;
        }
        else
        {
            cmd.q = hand.params[i].q_0 + hand.params[i].q_1 * phase;
            cmd.dq = hand.params[i].dq_0 + hand.params[i].dq_1 * phase;
            cmd.kp = hand.params[i].kp_0 + hand.params[i].kp_1 * phase;
            cmd.kd = hand.params[i].kd_0 + hand.params[i].kd_1 * phase;
            cmd.tau = hand.params[i].tau_0 + hand.params[i].tau_1 * phase;
            cmd.q = std::clamp(cmd.q, hand.joints[i].q_min, hand.joints[i].q_max);
        }

        last.q = cmd.q;
        last.dq = cmd.dq;
        last.kp = cmd.kp;
        last.kd = cmd.kd;
        last.tau = cmd.tau;
    }

    hand.cmd_pub->publish(out);
}

void sairol_bridge::G1Bridge::publishHandCommand_()
{
    // Silent while hand control is off. The mode's timeout bit, which every
    // commanded frame carries, releases the fingers once frames stop arriving.
    if (!handStarted_)
    {
        return;
    }

    if (latched_)
    {
        for (int side = 0; side < NUM_HANDS; ++side)
        {
            if (hands_[side].live)
            {
                RCLCPP_WARN(nh->get_logger(), "E-stop latched; releasing the %s hand.",
                            hands_[side].name.c_str());
                hands_[side].live = false;
            }
        }
    }

    const double now = steadyClock_.now().seconds();

    for (int side = 0; side < NUM_HANDS; ++side)
    {
        auto &hand = hands_[side];

        if (hand.live)
        {
            if (!checkHandState_(side))
            {
                RCLCPP_ERROR(nh->get_logger(), "%s hand state check failed; releasing it.",
                             hand.name.c_str());
                hand.live = false;
            }
            else if (now > hand.t_valid)
            {
                RCLCPP_WARN(nh->get_logger(), "%s hand watchdog expired; releasing it.",
                            hand.name.c_str());
                hand.live = false;
            }
        }

        // Per side, so a silent left hand does not drop the right. A released
        // hand keeps getting explicit limp frames rather than silence: an
        // overheating finger should go slack now, not on the device's timeout.
        // Not once its state is gone for 1.2 s: a restarted hand must not get old frames.
        const bool gone = !hand.state_seen ||
                          (steadyClock_.now() - hand.state_time).seconds() >
                              estop::STATE_LOSS_S + estop::SILENCE_AFTER_S;
        if (!hand.live && gone)
        {
            continue;
        }
        publishOneHand_(side, !hand.live);
    }
}

bool sairol_bridge::G1Bridge::handsAllowStart_(std::string &why)
{
    // Before the latch, as in armControl_: after a restart a reset is refused too.
    if (!linkAllowsArming_(why))
    {
        return false;
    }
    if (latched_)
    {
        why = "e-stop latched; call /estop_reset first";
        return false;
    }
    // The body state rule of body arming: a tilted robot or a stale /lowstate starts no hand.
    const estop::Cause fault = checkState_();
    if (fault != estop::Cause::None)
    {
        why = std::string("robot state fault: ") + estop::name(fault);
        return false;
    }
    return armingGates_(why);
}

void sairol_bridge::G1Bridge::startHands_(std::string &started, std::string &silent)
{
    for (int side = 0; side < NUM_HANDS; ++side)
    {
        auto &hand = hands_[side];

        // Refuse a hand that is not publishing state: nothing would show whether
        // the fingers answered, and the ramp below needs a real starting q.
        if (!checkHandState_(side))
        {
            hand.live = false;
            silent += (silent.empty() ? "" : ", ") + hand.name;
            continue;
        }

        // A live hand ramps from its last frame, gains and all: a restart must not drop it slack.
        const bool from_last = handStarted_ && hand.live;

        for (int i = 0; i < numHandJoint_; ++i)
        {
            auto &last = hand.last.motor_cmd[i];
            auto &target = hand.desired.motor_cmd[i];
            auto &params = hand.params[i];

            // Otherwise start from where the hand actually is, at zero gain, so
            // the first ramp cannot jerk a finger.
            if (!from_last)
            {
                last.q = hand.state.motor_state[i].q;
                last.dq = 0.0;
                last.tau = 0.0;
                last.kp = 0.0;
                last.kd = 0.0;
            }

            target.q = handReadyQ_[i];
            target.dq = 0.0;
            target.tau = 0.0;
            target.kp = hand.joints[i].kp;
            target.kd = hand.joints[i].kd;

            params.q_0 = last.q;
            params.q_1 = target.q - last.q;
            params.dq_0 = last.dq;
            params.dq_1 = -last.dq;
            params.tau_0 = 0.0;
            params.tau_1 = 0.0;
            params.kp_0 = last.kp;
            params.kp_1 = target.kp - last.kp;
            params.kd_0 = last.kd;
            params.kd_1 = target.kd - last.kd;
        }

        hand.t_start = steadyClock_.now().seconds();
        hand.t_final = hand.t_start + duration_;
        hand.t_valid = INF_;   // hold the ready pose until a command arrives
        hand.live = true;
        started += (started.empty() ? "" : ", ") + hand.name;
    }

    handStarted_ = !started.empty();
}

void sairol_bridge::G1Bridge::startHandControlServiceCB_(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;

    if (!hands_[HAND_LEFT].cmd_pub)
    {
        response->success = false;
        response->message = "hand path is not configured; the config has no hand_* parameters";
        return;
    }

    std::string why;
    if (!handsAllowStart_(why))
    {
        response->success = false;
        response->message = why;
        RCLCPP_WARN(nh->get_logger(), "Hand start refused: %s", why.c_str());
        return;
    }

    std::string started, silent;
    startHands_(started, silent);

    if (started.empty())
    {
        response->success = false;
        response->message = "no Dex3 hand is publishing state; nothing started";
        RCLCPP_ERROR(nh->get_logger(), "%s", response->message.c_str());
        return;
    }

    response->success = true;
    response->message = "hand control started: " + started +
                        (silent.empty() ? "" : "; silent, not started: " + silent);
    RCLCPP_INFO(nh->get_logger(), "%s", response->message.c_str());
}

std::string sairol_bridge::G1Bridge::afterReadyPose_()
{
    if (!hands_[HAND_LEFT].cmd_pub)
    {
        return "";   // no hand_* config: body only
    }

    std::string why, started, silent;
    std::string hands;
    if (!handsAllowStart_(why))
    {
        hands = "none (" + why + ")";
    }
    else
    {
        startHands_(started, silent);
        hands = started.empty() ? "none (no state)" : started;
        if (!started.empty() && !silent.empty())
        {
            hands += "; silent, not started: " + silent;
        }
    }
    RCLCPP_INFO(nh->get_logger(), "Ready: hands to hand_ready_q: %s", hands.c_str());
    return "; hands: " + hands;
}

void sairol_bridge::G1Bridge::stopHandControlServiceCB_(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;

    handStarted_ = false;
    for (int side = 0; side < NUM_HANDS; ++side)
    {
        hands_[side].live = false;
    }

    RCLCPP_INFO(nh->get_logger(), "Stopping hand control...");
    response->success = true;
    response->message = "hand control stopped; the mode timeout bit releases the fingers "
                        "once frames stop";
}
