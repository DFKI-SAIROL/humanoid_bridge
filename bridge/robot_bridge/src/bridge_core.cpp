#include "bridge_core.hpp"
#include <algorithm>

using std::placeholders::_1;
using std::placeholders::_2;

namespace sairol_bridge
{
    BridgeCore::BridgeCore(rclcpp::Node::SharedPtr node)
    {
        nh = node;

        // Load parameters
        if (!loadParameters_())
        {
            RCLCPP_FATAL(nh->get_logger(), "Failed to load parameters.");
            rclcpp::shutdown();
            return;
        }

        // distribute parameters
        lowCommandDesired_.motor_cmd.resize(numJoint_);
        lastCommand_.motor_cmd.resize(numJoint_);
        currentState_.motor_state.resize(numJoint_);
        cmdParams_.resize(numJoint_);
        holdQ_.assign(numJoint_, 0.0f);

        desiredSubscriber_ = nh->create_subscription<bridge_interface::msg::RobotCmd>(
            "/robot_cmd", 1, std::bind(&BridgeCore::robotCmdCallBack_, this, _1));

        // startControlService_ = nh->create_service<std_srvs::srv::Trigger>(
        //     "start_control", std::bind(&BridgeCore::startControlServiceCB_, this, _1, _2));

        stopControlService_ = nh->create_service<std_srvs::srv::Trigger>(
            "stop_control", std::bind(&BridgeCore::stopControlServiceCB_, this, _1, _2));

        // start_control, ready_position_control, zero_position_control: createArmingServices_().

        estopDampService_ = nh->create_service<std_srvs::srv::Trigger>(
            "estop_damp", std::bind(&BridgeCore::estopDampServiceCB_, this, _1, _2));

        estopHoldService_ = nh->create_service<std_srvs::srv::Trigger>(
            "estop_hold", std::bind(&BridgeCore::estopHoldServiceCB_, this, _1, _2));

        estopResetService_ = nh->create_service<std_srvs::srv::Trigger>(
            "estop_reset", std::bind(&BridgeCore::estopResetServiceCB_, this, _1, _2));

        // Transient-local: a late subscriber still gets the current state.
        bridgeStatePublisher_ = nh->create_publisher<std_msgs::msg::String>(
            "/bridge_state", rclcpp::QoS(1).reliable().transient_local());
        publishBridgeState_(describeState_());

        last_state_time_ = nh->get_clock()->now();
    }

    void BridgeCore::createArmingServices_()
    {
        readyPositionService_ = nh->create_service<std_srvs::srv::Trigger>(
            "ready_position_control", std::bind(&BridgeCore::readyPositionControlServiceCB_, this, _1, _2));

        zeroPositionService_ = nh->create_service<std_srvs::srv::Trigger>(
            "zero_position_control", std::bind(&BridgeCore::zeroPositionControlServiceCB_, this, _1, _2));

        startControlService_ = nh->create_service<bridge_interface::srv::SetDefaultPosition>(
            "start_control", std::bind(&BridgeCore::startControlServiceCB_, this, _1, _2));
    }

    void BridgeCore::start()
    {
        // Init Thread
        controlThread_ = std::thread([this]()
                                     { this->update_(); });
    }
    
    void BridgeCore::stop()
    {
        if (controlThread_.joinable())
        {
            controlThread_.join();
        }
    }

    BridgeCore::~BridgeCore() = default;

    bool BridgeCore::loadParameters_()
    {
        nh->get_parameter("num_joint", numJoint_);

        cmdParams_ = std::vector<CmdParams>(numJoint_);

        if (!nh->get_parameter("torque_control", torqueControl_))
        {
            RCLCPP_ERROR(nh->get_logger(), "Failed to get 'torque_control' from parameters");
            return false;
        }
        assert(torqueControl_ == false && "Torque control is not supported yet, please set 'torque_control' to false");
        if (!nh->get_parameter("duration", duration_))
        {
            RCLCPP_ERROR(nh->get_logger(), "Failed to get 'duration' from parameters");
            return false;
        }
        if (!nh->get_parameter("control_dt", controlDt_))
        {
            RCLCPP_ERROR(nh->get_logger(), "Failed to get 'control_dt' from parameters");
            return false;
        }
        if (!nh->get_parameter("upper_limbs_kp_min", upper_limbs_kp_min_))
        {
            RCLCPP_ERROR(nh->get_logger(), "Failed to get 'upper_limbs_kp_min' from parameters");
            return false;
        }
        if (!nh->get_parameter("upper_limbs_kp_max", upper_limbs_kp_max_))
        {
            RCLCPP_ERROR(nh->get_logger(), "Failed to get 'upper_limbs_kp_max' from parameters");
            return false;
        }
        if (!nh->get_parameter("upper_limbs_kd_min", upper_limbs_kd_min_))
        {
            RCLCPP_ERROR(nh->get_logger(), "Failed to get 'upper_limbs_kd_min' from parameters");
            return false;
        }
        if (!nh->get_parameter("upper_limbs_kd_max", upper_limbs_kd_max_))
        {
            RCLCPP_ERROR(nh->get_logger(), "Failed to get 'upper_limbs_kd_max' from parameters");
            return false;
        }
        if (!nh->get_parameter("lower_limbs_kp_min", lower_limbs_kp_min_))
        {
            RCLCPP_ERROR(nh->get_logger(), "Failed to get 'lower_limbs_kp_min' from parameters");
            return false;
        }
        if (!nh->get_parameter("lower_limbs_kp_max", lower_limbs_kp_max_))
        {
            RCLCPP_ERROR(nh->get_logger(), "Failed to get 'lower_limbs_kp_max' from parameters");
            return false;
        }
        if (!nh->get_parameter("lower_limbs_kd_min", lower_limbs_kd_min_))
        {
            RCLCPP_ERROR(nh->get_logger(), "Failed to get 'lower_limbs_kd_min' from parameters");
            return false;
        }
        if (!nh->get_parameter("lower_limbs_kd_max", lower_limbs_kd_max_))
        {
            RCLCPP_ERROR(nh->get_logger(), "Failed to get 'lower_limbs_kd_max' from parameters");
            return false;
        }
        if (!nh->get_parameter("empty_joint_index", emptyJointIndex_))
        {
            RCLCPP_ERROR(nh->get_logger(), "Failed to get 'empty_joint_index' from parameters");
            return false;
        }
        if (!nh->get_parameter("imu_rpy_threshold", imu_rpy_threshold_))
        {
            RCLCPP_ERROR(nh->get_logger(), "Failed to get 'imu_rpy_threshold' from parameters");
            return false;
        }

        RCLCPP_INFO(nh->get_logger(), "Number of the joints  = %d", numJoint_);
        RCLCPP_INFO(nh->get_logger(), "duration = %.3f", duration_);
        RCLCPP_INFO(nh->get_logger(), "control_dt = %.3f", controlDt_);
        RCLCPP_INFO(nh->get_logger(), "Upper limbs kp range: [%.3f, %.3f]", upper_limbs_kp_min_, upper_limbs_kp_max_);
        RCLCPP_INFO(nh->get_logger(), "Upper limbs kd range: [%.3f, %.3f]", upper_limbs_kd_min_, upper_limbs_kd_max_);
        RCLCPP_INFO(nh->get_logger(), "Lower limbs kp range: [%.3f, %.3f]", lower_limbs_kp_min_, lower_limbs_kp_max_);
        RCLCPP_INFO(nh->get_logger(), "Lower limbs kd range: [%.3f, %.3f]", lower_limbs_kd_min_, lower_limbs_kd_max_);
        RCLCPP_INFO(nh->get_logger(), "IMU RPY threshold = %.3f", imu_rpy_threshold_);

        // Gate 2: optional; per joint `<name>.q_rate_limit` overrides it. 0 = no limit.
        float_t q_rate_limit_default = 0;
        const bool has_rate_default = nh->get_parameter("q_rate_limit_default", q_rate_limit_default);

        std::vector<std::string> joint_names;
        if (nh->get_parameter("joint_names", joint_names))
        {
            joints_.clear();
            bool ret = true;
            int own_rate_limits = 0;
            for (const auto &name : joint_names)
            {
                Joint joint_info;
                joint_info.q_rate_limit = q_rate_limit_default;
                own_rate_limits += nh->get_parameter(name + ".q_rate_limit", joint_info.q_rate_limit) ? 1 : 0;
                if (!(joint_info.q_rate_limit >= 0))
                {
                    RCLCPP_ERROR(nh->get_logger(), "%s: q_rate_limit must be >= 0", name.c_str());
                    return false;
                }
                ret = ret && nh->get_parameter(name + ".idx", joint_info.idx);
                ret = ret && nh->get_parameter(name + ".q_min", joint_info.q_min);
                ret = ret && nh->get_parameter(name + ".q_max", joint_info.q_max);
                ret = ret && nh->get_parameter(name + ".dq_limit", joint_info.dq_limit);
                ret = ret && nh->get_parameter(name + ".tau_limit", joint_info.tau_limit);
                ret = ret && nh->get_parameter(name + ".kp", joint_info.kp);
                ret = ret && nh->get_parameter(name + ".kd", joint_info.kd);
                ret = ret && nh->get_parameter(name + ".if_strong_joint", joint_info.if_strong_joint);
                ret = ret && nh->get_parameter(name + ".if_parallel_joint", joint_info.if_parallel_joint);
                joints_.push_back(joint_info);
                assert(joint_info.idx < numJoint_ && ("Joint index exceeds the number of joints, please check the joint index: " + std::to_string(joint_info.idx)).c_str());
                assert(joint_info.q_max >= joint_info.q_min && ("Joint q_max must be greater than or equal to q_min, please check the joint index: " + std::to_string(joint_info.idx)).c_str());
                assert(joint_info.dq_limit >= 0 && ("Joint dq_limit must be non-negative, please check the joint index: " + std::to_string(joint_info.idx)).c_str());
                assert(joint_info.tau_limit >= 0 && ("Joint tau_limit must be non-negative, please check the joint index: " + std::to_string(joint_info.idx)).c_str());
                assert(joint_info.kp >= 0 && ("Joint kp must be non-negative, please check the joint index: " + std::to_string(joint_info.idx)).c_str());
                assert(joint_info.kd >= 0 && ("Joint kd must be non-negative, please check the joint index: " + std::to_string(joint_info.idx)).c_str());
                assert(joint_info.if_strong_joint == true || joint_info.if_strong_joint == false && ("Joint if_strong_joint must be a boolean value, please check the joint index: " + std::to_string(joint_info.idx)).c_str());
                assert(joint_info.if_parallel_joint == true || joint_info.if_parallel_joint == false && ("Joint if_parallel_joint must be a boolean value, please check the joint index: " + std::to_string(joint_info.idx)).c_str());
            }
            if (!ret)
            {
                RCLCPP_ERROR(nh->get_logger(), "Failed to get joint parameters from parameters");
                return false;
            }
            RCLCPP_INFO(nh->get_logger(), "Loaded %ld joint limits", joints_.size());
            if (has_rate_default)
            {
                RCLCPP_INFO(nh->get_logger(), "Target speed limit: %.2f rad/s, %d joints with their own",
                            q_rate_limit_default, own_rate_limits);
            }
            else
            {
                RCLCPP_WARN(nh->get_logger(), "No q_rate_limit_default: target speed is unlimited on %d joints",
                            numJoint_ - own_rate_limits);
            }
        }
        else
        {
            RCLCPP_ERROR(nh->get_logger(), "Failed to get 'joint_names' from parameters");
            return false;
        }

        if (!nh->get_parameter("ready_q", ready_q_))
        {
            RCLCPP_ERROR(nh->get_logger(), "Failed to get 'ready_q' from parameters");
            for (int i = 0; i < numJoint_; ++i)
            {
                assert(ready_q_[i] >= joints_[i].q_min && ready_q_[i] <= joints_[i].q_max &&
                       ("Ready position must be within joint limits, please check the joint index: " + std::to_string(i)).c_str());
            }
            return false;
        }

        assert(joints_.size() == numJoint_ && "Number of joints does not match the number of joint limits");
        assert(upper_limbs_kp_max_ >= upper_limbs_kp_min_ && "Upper limbs kp max must be greater than or equal to min");
        assert(upper_limbs_kd_max_ >= upper_limbs_kd_min_ && "Upper limbs kd max must be greater than or equal to min");
        assert(lower_limbs_kp_max_ >= lower_limbs_kp_min_ && "Lower limbs kp max must be greater than or equal to min");
        assert(lower_limbs_kd_max_ >= lower_limbs_kd_min_ && "Lower limbs kd max must be greater than or equal to min");
        return loadEstopParameters_();
    }

    bool BridgeCore::loadEstopParameters_()
    {
        // Every key is optional: a config without an `estop:` block gets the
        // defaults in estop::Config. A key that is present but wrong fails the load.
        nh->get_parameter("estop.damp_kd", estopConfig_.damp_kd);
        if (!(estopConfig_.damp_kd > 0))
        {
            RCLCPP_ERROR(nh->get_logger(), "estop.damp_kd must be > 0, got %.3f", estopConfig_.damp_kd);
            return false;
        }

        const std::pair<const char *, uint16_t *> combos[] = {
            {"estop.remote_combo_damp", &estopConfig_.combo_damp},
            {"estop.remote_combo_hold", &estopConfig_.combo_hold},
        };
        for (const auto &[key, combo] : combos)
        {
            std::vector<std::string> keys;
            if (!nh->get_parameter(key, keys))
            {
                continue;
            }
            *combo = estop::parseCombo(keys);
            if (*combo == 0)
            {
                RCLCPP_ERROR(nh->get_logger(), "%s: empty, or a key name that is not on the remote", key);
                return false;
            }
        }
        if (estopConfig_.combo_damp == estopConfig_.combo_hold)
        {
            RCLCPP_ERROR(nh->get_logger(), "estop.remote_combo_damp and remote_combo_hold are the same");
            return false;
        }

        const std::pair<const char *, estop::Mode *> modes[] = {
            {"estop.mode.stop", &estopConfig_.stop},
            {"estop.mode.imu", &estopConfig_.imu},
            {"estop.mode.state_loss", &estopConfig_.state_loss},
            {"estop.mode.state_invalid", &estopConfig_.state_invalid},
            {"estop.mode.cmd_timeout", &estopConfig_.cmd_timeout},
        };
        for (const auto &[key, mode] : modes)
        {
            std::string text;
            if (nh->get_parameter(key, text) && !estop::parseMode(text, *mode))
            {
                RCLCPP_ERROR(nh->get_logger(), "%s must be damp or hold, got '%s'", key, text.c_str());
                return false;
            }
        }

        // Hold mode divides by kp for its torque window.
        for (size_t i = 0; i < joints_.size(); ++i)
        {
            if (!(joints_[i].kp > 0) || joints_[i].kd < 0)
            {
                RCLCPP_ERROR(nh->get_logger(), "joint %zu: e-stop hold needs kp > 0 and kd >= 0", i);
                return false;
            }
        }

        nh->get_parameter("estop.require_remote", estopConfig_.require_remote);

        latch_ = estop::SafetyLatch(estopConfig_);
        RCLCPP_INFO(nh->get_logger(),
                    "E-stop: damp_kd %.1f, remote combos 0x%04x damp / 0x%04x hold; modes stop %s, imu %s, "
                    "state_loss %s, state_invalid %s, cmd_timeout %s; arming %s the remote",
                    estopConfig_.damp_kd, estopConfig_.combo_damp, estopConfig_.combo_hold,
                    estop::name(estopConfig_.stop), estop::name(estopConfig_.imu),
                    estop::name(estopConfig_.state_loss), estop::name(estopConfig_.state_invalid),
                    estop::name(estopConfig_.cmd_timeout),
                    estopConfig_.require_remote ? "needs" : "does not need");
        return true;
    }

    void BridgeCore::robotCmdCallBack_(bridge_interface::msg::RobotCmd::SharedPtr message)
    {
        auto duration = message->duration;
        auto interpolation_order = message->interpolation_order;
        auto hold_position = message->hold_position;

        if (not controlStarted_)
        {
            RCLCPP_WARN_ONCE(nh->get_logger(), "Control not started. Please start control first.");
            return;
        }

        if (checkCommand_(message))
        {   
            lowCommandDesired_.motor_cmd = message->motor_cmd;
            receivedCmd_ = true;
            calculateInterpolationParams_(duration, interpolation_order, hold_position);
        }
        else
        {
            RCLCPP_ERROR(nh->get_logger(), "Command check failed. Please inspect the command message.");
            return;
        }
    }

    void BridgeCore::zeroPositionControl_()
    {
        for (int i = 0; i < numJoint_; ++i)
        {
            lowCommandDesired_.motor_cmd[i].q = 0.0;
            lowCommandDesired_.motor_cmd[i].dq = 0.0;
            lowCommandDesired_.motor_cmd[i].tau = 0.0;
            lowCommandDesired_.motor_cmd[i].kp = joints_[i].kp;
            lowCommandDesired_.motor_cmd[i].kd = joints_[i].kd;
        }
    }

    void BridgeCore::readyPositionControl_()
    {
        for (int i = 0; i < numJoint_; ++i)
        {
            lowCommandDesired_.motor_cmd[i].q = ready_q_[i];
            lowCommandDesired_.motor_cmd[i].dq = 0.0;
            lowCommandDesired_.motor_cmd[i].tau = 0.0;
            lowCommandDesired_.motor_cmd[i].kp = joints_[i].kp;
            lowCommandDesired_.motor_cmd[i].kd = joints_[i].kd;
        }
    }


    void BridgeCore::update_()
    {
        auto rate = rclcpp::Rate(1.0 / controlDt_);
        while (rclcpp::ok())
        {
            // Faults are checked while anything is published: a fault while
            // stopped in hold still escalates to damp.
            if (controlStarted_ || stopPublishing_)
            {
                // Onset only: a lasting fault must not undo an operator hold every cycle.
                const estop::Cause fault = estop::faultOnset(checkState_(), lastFault_);
                if (fault != estop::Cause::None)
                {
                    enterStop_(fault);
                }
                else if (controlStarted_ && nh->get_clock()->now().seconds() > tValid_)
                {
                    enterStop_(estop::Cause::CmdTimeout);
                }

                std::unique_lock<std::mutex> lock(mutex_);
                const double state_age = (nh->get_clock()->now() - last_state_time_).seconds();
                if (estop::silenceDue(latch_.latched(), latch_.publishing(), state_age) && latch_.silence())
                {
                    syncLatchFlags_();
                    const std::string state = latch_.describe(controlStarted_);
                    publishBridgeState_(state);
                    RCLCPP_WARN(nh->get_logger(), "No /lowstate for %.1f s: %s", state_age, state.c_str());
                }
                // Not while latched: a fault can land between initControl_ and commitArm.
                if (controlStarted_ && !latch_.latched())
                {
                    publishLowCommand_();
                }
                else if (latch_.publishing())
                {
                    publishStopCommand_();
                }
                lock.unlock();
            }
            else
            {
                lastFault_ = estop::Cause::None;   // not watched: the next fault is an onset
            }
            rate.sleep();
        }
    }

    void BridgeCore::calculateInterpolationParams_(float_t duration,
                                                   float_t interpolation_order,
                                                   bool hold_position)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        cmdInterpOrder_ = interpolation_order;
        if (0.0 <= cmdInterpOrder_ && cmdInterpOrder_ < 1.0)
        {
            for (int i = 0; i < numJoint_; i++)
            {
                cmdParams_[i].q_0 = lowCommandDesired_.motor_cmd[i].q;
                cmdParams_[i].q_1 = 0.0;
                cmdParams_[i].tau_0 = lowCommandDesired_.motor_cmd[i].tau;
                cmdParams_[i].tau_1 = 0.0;
                cmdParams_[i].dq_0 = lowCommandDesired_.motor_cmd[i].dq;
                cmdParams_[i].dq_1 = 0.0;
                cmdParams_[i].kp_0 = lowCommandDesired_.motor_cmd[i].kp;
                cmdParams_[i].kp_1 = 0.0;
                cmdParams_[i].kd_0 = lowCommandDesired_.motor_cmd[i].kd;
                cmdParams_[i].kd_1 = 0.0;
            }
        }
        else if (cmdInterpOrder_ == 1.0)
        {
            for (int i = 0; i < numJoint_; i++)
            {
                cmdParams_[i].q_0 = lastCommand_.motor_cmd[i].q; // currentState_.motor_state[i].q;
                cmdParams_[i].q_1 = lowCommandDesired_.motor_cmd[i].q - cmdParams_[i].q_0;
                cmdParams_[i].tau_0 = lowCommandDesired_.motor_cmd[i].tau; // currentState_.motor_state[i].tau;
                cmdParams_[i].tau_1 = 0.0;
                cmdParams_[i].dq_0 = lastCommand_.motor_cmd[i].dq; // currentState_.motor_state[i].dq;
                cmdParams_[i].dq_1 = lowCommandDesired_.motor_cmd[i].dq - cmdParams_[i].dq_0;
                cmdParams_[i].kp_0 = lastCommand_.motor_cmd[i].kp; // currentState_.motor_state[i].kp;
                cmdParams_[i].kp_1 = lowCommandDesired_.motor_cmd[i].kp - cmdParams_[i].kp_0;
                cmdParams_[i].kd_0 = lastCommand_.motor_cmd[i].kd; // currentState_.motor_state[i].kd;
                cmdParams_[i].kd_1 = lowCommandDesired_.motor_cmd[i].kd - cmdParams_[i].kd_0;
            }
        }
        else
        {
            RCLCPP_FATAL(nh->get_logger(), "Invalid interpolation order: %f. Must be 0 or 1.", cmdInterpOrder_);
            throw std::invalid_argument("Invalid interpolation order");
        }

        lock.unlock();

        tStart_ = nh->get_clock()->now().seconds();
        tFinal_ = tStart_ + duration;
        if (hold_position)
        {
            tValid_ = INF_;
        }
        else
        {
            tValid_ = tFinal_ + 0.2;
        }
    }

    bool BridgeCore::checkStateFreshness_()
    {
        std::unique_lock<std::mutex> lock(mutex_);
        const rclcpp::Time last_state_time = last_state_time_;
        lock.unlock();

        auto &clock = *nh->get_clock();
        const auto dt_state_ = (clock.now() - last_state_time).seconds();

        // Check if the state message is received within the expected interval
        if (dt_state_ > estop::STATE_LOSS_S)
        {
            RCLCPP_ERROR_THROTTLE(nh->get_logger(), clock, 1000,
                                  "Robot signal lost! No LowState message received for %.2f seconds. "
                                  "Expected interval ~0.002s (500Hz).",
                                  dt_state_);
            return false;
        }
        return true;
    }

    estop::Cause BridgeCore::checkState_()
    {
        if (!checkStateFreshness_())
        {
            return estop::Cause::StateLoss;
        }

        // Copy under the lock: lowStateHandler_ writes these on the executor thread.
        std::unique_lock<std::mutex> lock(mutex_);
        const auto motor_state = currentState_.motor_state;
        const auto imu = imu_;
        lock.unlock();

        auto &clock = *nh->get_clock();

        // Check if the state message has valid data
        if (motor_state.size() != static_cast<size_t>(numJoint_))
        {
            RCLCPP_ERROR_THROTTLE(nh->get_logger(), clock, 1000,
                                  "Motor state length mismatch: expected %d, got %ld",
                                  numJoint_, motor_state.size());
            return estop::Cause::StateInvalid;
        }
        // Check if the motor state values are valid (not NaN or Inf) and within limits
        for (size_t i = 0; i < static_cast<size_t>(numJoint_); ++i)
        {
            const auto &motor = motor_state[i];
            if (!std::isfinite(motor.q) || !std::isfinite(motor.dq) ||
                !std::isfinite(motor.ddq) || !std::isfinite(motor.tau_est))
            {
                RCLCPP_ERROR_THROTTLE(nh->get_logger(), clock, 1000,
                                      "Motor state at index %lu contains invalid (NaN/Inf) values.", i);
                return estop::Cause::StateInvalid;
            }

            const auto &joint = joints_[i];

            if (std::abs(motor.dq) > joint.dq_limit)
            {
                RCLCPP_ERROR_THROTTLE(nh->get_logger(), clock, 1000,
                                      "Joint [%lu] dq (%.3f) exceeds limit (%.3f).",
                                      i, motor.dq, joint.dq_limit);
                return estop::Cause::StateInvalid;
            }
        }

        // Check if the IMU state values are valid (not NaN or Inf)
        for (int i = 0; i < 3; ++i)
        {
            if (!std::isfinite(imu.rpy[i]) ||
                !std::isfinite(imu.gyroscope[i]) ||
                !std::isfinite(imu.accelerometer[i]))
            {
                RCLCPP_ERROR_THROTTLE(nh->get_logger(), clock, 1000,
                                      "IMU state contains invalid (NaN/Inf) values at index %d.", i);
                return estop::Cause::StateInvalid;
            }
        }

        if (std::abs(imu.rpy[0]) > imu_rpy_threshold_ || std::abs(imu.rpy[1]) > imu_rpy_threshold_)
        {
            RCLCPP_WARN_THROTTLE(nh->get_logger(), clock, 1000,
                                 "IMU base rpy values are too large: [%f, %f]", imu.rpy[0], imu.rpy[1]);
            return estop::Cause::Imu;
        }

        return estop::Cause::None;
    }

    void BridgeCore::syncLatchFlags_()
    {
        latched_ = latch_.latched();
        stopPublishing_ = latch_.publishing();
    }

    std::string BridgeCore::describeState_()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return latch_.describe(controlStarted_);
    }

    void BridgeCore::publishBridgeState_(const std::string &state)
    {
        std_msgs::msg::String message;
        message.data = state;
        bridgeStatePublisher_->publish(message);
    }

    void BridgeCore::enterStop_(estop::Cause cause)
    {
        estop::Transition transition;
        std::string state;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            const bool was_tracking = controlStarted_.exchange(false);
            transition = latch_.trigger(cause, was_tracking);
            if (transition.snapshot)
            {
                for (int i = 0; i < numJoint_; ++i)
                {
                    holdQ_[i] = currentState_.motor_state[i].q;
                }
            }
            syncLatchFlags_();
            state = latch_.describe(controlStarted_);
            if (transition.changed)
            {
                publishBridgeState_(state);
            }
        }

        if (!transition.changed)
        {
            return;
        }
        RCLCPP_WARN(nh->get_logger(), "Stop (%s): %s", estop::name(cause), state.c_str());
        finishControl_();
    }

    bool BridgeCore::remoteAllowsArming_(std::string &why)
    {
        if (!estopConfig_.require_remote)
        {
            return true;
        }
        const double age = nh->get_clock()->now().seconds() - lastRemoteHeader_.load();
        if (age <= estop::REMOTE_TIMEOUT_S)
        {
            return true;
        }
        why = "remote not connected: switch the remote on (estop.require_remote)";
        return false;
    }

    bool BridgeCore::armControl_(bridge_interface::msg::RobotCmd default_cmd, std::string &why)
    {
        uint64_t generation = 0;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (latch_.latched())
            {
                why = latch_.describe(false) + "; call /estop_reset first";
                return false;
            }
            generation = latch_.generation();
        }
        // update_() would latch this fault at once; a stale /lowstate would first track a stale pose.
        const estop::Cause fault = checkState_();
        if (fault != estop::Cause::None)
        {
            why = std::string("robot state fault: ") + estop::name(fault);
            return false;
        }
        if (!remoteAllowsArming_(why) || !robotAllowsArming_(why))
        {
            return false;
        }

        // initControl_ sets controlStarted_; update_() then publishes tracking
        // frames, which take priority over the stop frame.
        if (!initControl_(default_cmd))
        {
            controlStarted_ = false;
            why = "initControl_ failed";
            return false;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        if (!latch_.commitArm(generation))
        {
            controlStarted_ = false;
            why = "a stop arrived while arming: " + latch_.describe(false);
            return false;
        }
        syncLatchFlags_();
        publishBridgeState_(latch_.describe(controlStarted_));
        return true;
    }

    bool BridgeCore::checkMotorCmd_(bridge_interface::msg::MotorCmd &cmd,
                                    const Joint &joint_info,
                                    float_t kp_min, float_t kp_max,
                                    float_t kd_min, float_t kd_max,
                                    size_t index, bool &clipped)
    {
        // Check for invalid numbers
        if (!std::isfinite(cmd.q) || !std::isfinite(cmd.dq) ||
            !std::isfinite(cmd.tau) || !std::isfinite(cmd.kp) ||
            !std::isfinite(cmd.kd))
        {
            RCLCPP_ERROR(nh->get_logger(),
                         "Command check failed: motorCmd[%lu] contains invalid (NaN/Inf) values.", index);
            return false; // Can't clip NaN/Inf, so still return false
        }

        if (std::abs(cmd.q) > 50 || std::abs(cmd.dq) > 1e2 ||
            std::abs(cmd.tau) > 1e3 || std::abs(cmd.kp) > 1e4 ||
            std::abs(cmd.kd) > 1e3)
        {
            RCLCPP_ERROR(nh->get_logger(),
                         "Command check failed: motorCmd[%lu] contains unreasonably large values.", index);
            RCLCPP_ERROR(nh->get_logger(),
                         "Values - q: %.3f, dq: %.3f, tau: %.3f, kp: %.3f, kd: %.3f",
                         cmd.q, cmd.dq, cmd.tau, cmd.kp, cmd.kd);
            return false; // Values too large, likely an error
        }

        // Check gains
        if (cmd.kp < kp_min || cmd.kp > kp_max || cmd.kd < kd_min || cmd.kd > kd_max)
        {
            RCLCPP_WARN_ONCE(nh->get_logger(), "Clipping motor_cmd[%lu] gains", index);
            clipped = true;
        }
        cmd.kp = std::clamp(cmd.kp, kp_min, kp_max);
        cmd.kd = std::clamp(cmd.kd, kd_min, kd_max);

        // Check velocity
        if (cmd.dq < -joint_info.dq_limit || cmd.dq > joint_info.dq_limit)
        {
            RCLCPP_WARN_ONCE(nh->get_logger(), "Clipping motor_cmd[%lu] dq: %.3f", index, cmd.dq);
            cmd.dq = std::clamp(cmd.dq, -joint_info.dq_limit, joint_info.dq_limit);
            clipped = true;
        }

        // Check torque
        if (cmd.tau < -joint_info.tau_limit || cmd.tau > joint_info.tau_limit)
        {
            RCLCPP_WARN_ONCE(nh->get_logger(), "Clipping motor_cmd[%lu] tau: %.3f", index, cmd.tau);
            cmd.tau = std::clamp(cmd.tau, -joint_info.tau_limit, joint_info.tau_limit);
            clipped = true;
        }

        return true;
    }

    bool BridgeCore::checkCommand_(bridge_interface::msg::RobotCmd::SharedPtr robotCommand)
    {
        // An order outside [0, 1] made calculateInterpolationParams_ throw on the executor
        // thread and end the process; a NaN duration made tValid_ NaN, so no timeout.
        const float_t order = robotCommand->interpolation_order;
        const float_t duration = robotCommand->duration;
        if (!std::isfinite(order) || order < 0 || order > 1 || !std::isfinite(duration) || duration < 0)
        {
            RCLCPP_ERROR(nh->get_logger(),
                         "Command check failed: interpolation_order %.3f (needs [0, 1]) or duration %.3f (needs >= 0).",
                         order, duration);
            return false;
        }

        // Check if the command message has valid data
        if (robotCommand->motor_cmd.size() != static_cast<size_t>(numJoint_))
        {
            RCLCPP_ERROR(nh->get_logger(),
                         "Command check failed: motor_cmd size mismatch. Expected %d, got %ld.",
                         numJoint_, robotCommand->motor_cmd.size());
            return false;
        }

        bool clipped = false;
        for (size_t i = 0; i < static_cast<size_t>(numJoint_); ++i)
        {
            const auto &joint_info = joints_[i];
            // Strong joints are the legs and waist, and carry the wider gain range.
            const bool strong = joint_info.if_strong_joint;
            if (!checkMotorCmd_(robotCommand->motor_cmd[i], joint_info,
                                strong ? lower_limbs_kp_min_ : upper_limbs_kp_min_,
                                strong ? lower_limbs_kp_max_ : upper_limbs_kp_max_,
                                strong ? lower_limbs_kd_min_ : upper_limbs_kd_min_,
                                strong ? lower_limbs_kd_max_ : upper_limbs_kd_max_,
                                i, clipped))
            {
                return false;
            }
            // q is deliberately not clamped here; publishLowCommand_ bounds it by
            // the torque that reaching it would take.
        }

        if (clipped)
        {
            RCLCPP_WARN(nh->get_logger(), "Some joints were clipped to stay within safety limits");
        }

        return true;
    }

    void BridgeCore::stopControlServiceCB_(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        RCLCPP_INFO(nh->get_logger(), "Stopping control...");
        enterStop_(estop::Cause::Stop);
        response->success = true;
        response->message = describeState_();
    }

    void BridgeCore::estopDampServiceCB_(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        enterStop_(estop::Cause::ServiceDamp);
        response->success = true;
        response->message = describeState_();
    }

    void BridgeCore::estopHoldServiceCB_(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        enterStop_(estop::Cause::ServiceHold);
        response->success = true;
        response->message = describeState_();
    }

    void BridgeCore::estopResetServiceCB_(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        estop::Cause present = checkState_();
        if (present == estop::Cause::None)
        {
            present = estop::remoteCause(remoteKeys_, estopConfig_);
        }

        std::string why, state;
        bool ok = false;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            ok = latch_.reset(present != estop::Cause::None, why);
            syncLatchFlags_();
            state = latch_.describe(controlStarted_);
            if (ok)
            {
                publishBridgeState_(state);
            }
        }

        response->success = ok;
        if (!ok)
        {
            response->message = "reset refused: " + why +
                                (present != estop::Cause::None ? std::string(" (") + estop::name(present) + ")" : "");
            RCLCPP_WARN(nh->get_logger(), "%s", response->message.c_str());
            return;
        }
        response->message = state;
        RCLCPP_WARN(nh->get_logger(), "E-stop reset: %s", state.c_str());
    }

    void BridgeCore::readyPositionControlServiceCB_(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        std::string why;
        // Moves every joint, so it counts as arming even while tracking.
        const bool allowed = controlStarted_ ? remoteAllowsArming_(why) && robotAllowsArming_(why)
                                             : armControl_(bridge_interface::msg::RobotCmd(), why);
        if (!allowed)
        {
            RCLCPP_WARN(nh->get_logger(), "Ready position refused: %s", why.c_str());
            response->success = false;
            response->message = why;
            return;
        }
        readyPositionControl_();
        calculateInterpolationParams_(duration_, 1, true);
        RCLCPP_INFO(nh->get_logger(), "Ready position control activated");
        response->success = true;
        response->message = "Ready position control activated";
    }

    void BridgeCore::zeroPositionControlServiceCB_(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        std::string why;
        // Moves every joint, so it counts as arming even while tracking.
        const bool allowed = controlStarted_ ? remoteAllowsArming_(why) && robotAllowsArming_(why)
                                             : armControl_(bridge_interface::msg::RobotCmd(), why);
        if (!allowed)
        {
            RCLCPP_WARN(nh->get_logger(), "Zero position refused: %s", why.c_str());
            response->success = false;
            response->message = why;
            return;
        }
        zeroPositionControl_();
        calculateInterpolationParams_(duration_, 1, true);
        RCLCPP_INFO(nh->get_logger(), "Zero position control activated");
        response->success = true;
        response->message = "Zero position control activated";
    }

    void BridgeCore::startControlServiceCB_(
        const std::shared_ptr<bridge_interface::srv::SetDefaultPosition::Request> request,
        std::shared_ptr<bridge_interface::srv::SetDefaultPosition::Response> response)
    {
        auto default_position = request->default_position;
        bridge_interface::msg::RobotCmd default_cmd;

        // Gate 9: empty means ready_q_; any other length, or a value that is not finite or is
        // over 0.1 rad outside its joint range (a typo such as 1e3), moves nothing.
        std::string bad;
        if (!default_position.empty() && default_position.size() != static_cast<size_t>(numJoint_))
        {
            bad = "default_position has " + std::to_string(default_position.size()) + " values, expected " +
                  std::to_string(numJoint_) + " (or none for the ready pose)";
        }
        for (size_t i = 0; bad.empty() && i < default_position.size(); ++i)
        {
            const float_t q = static_cast<float_t>(default_position[i]);   // as MotorCmd.q will hold it
            if (!std::isfinite(q) || q < joints_[i].q_min - 0.1f || q > joints_[i].q_max + 0.1f)
            {
                bad = "default_position[" + std::to_string(i) + "] = " + std::to_string(default_position[i]) +
                      " is not finite or outside [" + std::to_string(joints_[i].q_min) + ", " +
                      std::to_string(joints_[i].q_max) + "]";
            }
        }
        if (!bad.empty())
        {
            RCLCPP_WARN(nh->get_logger(), "Start control refused: %s", bad.c_str());
            response->success = false;
            response->message = bad;
            return;
        }

        if (default_position.size() == static_cast<size_t>(numJoint_))
        {
            default_cmd.motor_cmd.resize(numJoint_);
            for (int i = 0; i < numJoint_; ++i)
            {
                default_cmd.motor_cmd[i].q = default_position[i];
                default_cmd.motor_cmd[i].dq = 0.0;
                default_cmd.motor_cmd[i].tau = 0.0;
                default_cmd.motor_cmd[i].kp = joints_[i].kp;
                default_cmd.motor_cmd[i].kd = joints_[i].kd;
            }
        }

        std::string why;
        if (!armControl_(default_cmd, why))
        {
            RCLCPP_WARN(nh->get_logger(), "Start control refused: %s", why.c_str());
            response->success = false;
            response->message = why;
            return;
        }
        RCLCPP_INFO(nh->get_logger(), "Starting control...");
        response->success = true;
        response->message = default_position.empty() ? "armed: ramping to the ready pose"
                                                     : "armed: ramping to default_position";
    }
}
