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

        desiredSubscriber_ = nh->create_subscription<bridge_interface::msg::RobotCmd>(
            "/robot_cmd", 1, std::bind(&BridgeCore::robotCmdCallBack_, this, _1));

        // startControlService_ = nh->create_service<std_srvs::srv::Trigger>(
        //     "start_control", std::bind(&BridgeCore::startControlServiceCB_, this, _1, _2));

        stopControlService_ = nh->create_service<std_srvs::srv::Trigger>(
            "stop_control", std::bind(&BridgeCore::stopControlServiceCB_, this, _1, _2));

        readyPositionService_ = nh->create_service<std_srvs::srv::Trigger>(
            "ready_position_control", std::bind(&BridgeCore::readyPositionControlServiceCB_, this, _1, _2));

        zeroPositionService_ = nh->create_service<std_srvs::srv::Trigger>(
            "zero_position_control", std::bind(&BridgeCore::zeroPositionControlServiceCB_, this, _1, _2));

        startControlService_ = nh->create_service<bridge_interface::srv::SetDefaultPosition>(
            "start_control", std::bind(&BridgeCore::startControlServiceCB_, this, _1, _2));

        last_state_time_ = nh->get_clock()->now();
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

        std::vector<std::string> joint_names;
        if (nh->get_parameter("joint_names", joint_names))
        {
            joints_.clear();
            bool ret = true;
            for (const auto &name : joint_names)
            {
                Joint joint_info;
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
            if (controlStarted_)
            {
                // Safety check
                if (not checkState_())
                {
                    RCLCPP_ERROR(nh->get_logger(), "Robot state check failed. Please inspect the robot carefully.");
                    controlStarted_ = false;
                }

                controlStarted_ &= (nh->get_clock()->now()).seconds() <= tValid_;

                if (!controlStarted_)
                {
                    finishControl_();
                    RCLCPP_ERROR(nh->get_logger(), "Control finished due to safety check or time out.");
                }

                std::unique_lock<std::mutex> lock(mutex_);
                // Update the low command
                publishLowCommand_();
                lock.unlock();
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

        auto dt_state_ = (nh->get_clock()->now() - last_state_time_).seconds();

        // Check if the state message is received within the expected interval
        if (dt_state_ > 0.2)
        {
            RCLCPP_ERROR(nh->get_logger(),
                         "Robot signal lost! No LowState message received for %.2f seconds. "
                         "Expected interval ~0.002s (500Hz). Shutting down the node to prevent unsafe operation.",
                         dt_state_);
            return false;
        }
        return true;
    }

    bool BridgeCore::checkState_()
    {
        if (!checkStateFreshness_())
        {
            return false;
        }
        // Check if the state message has valid data
        if (currentState_.motor_state.size() != numJoint_)
        {
            RCLCPP_ERROR(nh->get_logger(),
                         "Motor state length mismatch: expected %d, got %ld",
                         numJoint_, currentState_.motor_state.size());
            return false;
        }
        // Check if the motor state values are valid (not NaN or Inf) and within limits
        for (size_t i = 0; i < numJoint_; ++i)
        {
            const auto &motor = currentState_.motor_state[i];
            if (!std::isfinite(motor.q) || !std::isfinite(motor.dq) ||
                !std::isfinite(motor.ddq) || !std::isfinite(motor.tau_est))
            {
                RCLCPP_ERROR(nh->get_logger(),
                             "Motor state at index %lu contains invalid (NaN/Inf) values.", i);
                return false;
            }

            const auto &joint = joints_[i];

            if (std::abs(motor.dq) > joint.dq_limit)
            {
                RCLCPP_ERROR(nh->get_logger(),
                             "Joint [%lu] dq (%.3f) exceeds limit (%.3f).",
                             i, motor.dq, joint.dq_limit);
                return false;
            }
        }

        // Check if the IMU state values are valid (not NaN or Inf)
        for (int i = 0; i < 3; ++i)
        {
            if (!std::isfinite(imu_.rpy[i]) ||
                !std::isfinite(imu_.gyroscope[i]) ||
                !std::isfinite(imu_.accelerometer[i]))
            {
                RCLCPP_ERROR(nh->get_logger(),
                             "IMU state contains invalid (NaN/Inf) values at index %d.", i);
                return false;
            }
        }

        if (std::abs(imu_.rpy[0]) > imu_rpy_threshold_ || std::abs(imu_.rpy[1]) > imu_rpy_threshold_)
        {
            RCLCPP_WARN(nh->get_logger(), "IMU base rpy values are too large: [%f, %f]", imu_.rpy[0], imu_.rpy[1]);
            return false;
        }

        return true;
    }

    bool BridgeCore::checkCommand_(bridge_interface::msg::RobotCmd::SharedPtr robotCommand)
    {
        // Check if the command message has valid data
        if (robotCommand->motor_cmd.size() != numJoint_)
        {
            RCLCPP_ERROR(nh->get_logger(),
                         "Command check failed: motor_cmd size mismatch. Expected %ld, got %ld.",
                         joints_.size(), lastCommand_.motor_cmd.size());
            return false;
        }
        int any_value_clipped = -1;
        for (size_t i = 0; i < numJoint_; ++i)
        {
            auto &cmd = robotCommand->motor_cmd[i];
            auto &joint_info = joints_[i];
            // Check for invalid numbers
            if (!std::isfinite(cmd.q) || !std::isfinite(cmd.dq) ||
                !std::isfinite(cmd.tau) || !std::isfinite(cmd.kp) ||
                !std::isfinite(cmd.kd))
            {
                RCLCPP_ERROR(nh->get_logger(),
                                 "Command check failed: motorCmd[%lu] contains invalid (NaN/Inf) values.", i);
                return false; // Can't clip NaN/Inf, so still return false
            }

            if (abs(cmd.q) > 50 || abs(cmd.dq) > 1e2 ||
                abs(cmd.tau) > 1e3 || abs(cmd.kp) > 1e4 ||
                abs(cmd.kd) > 1e3)
            {
                RCLCPP_ERROR(nh->get_logger(),
                                 "Command check failed: motorCmd[%lu] contains unreasonably large values.", i);
                RCLCPP_ERROR(nh->get_logger(),
                                 "Values - q: %.3f, dq: %.3f, tau: %.3f, kp: %.3f, kd: %.3f",
                                 cmd.q, cmd.dq, cmd.tau, cmd.kp, cmd.kd);
                return false; // Values too large, likely an error
            }

            // Check gains
            if (joint_info.if_strong_joint) 
            {
                if (cmd.kp < lower_limbs_kp_min_ || cmd.kp > lower_limbs_kp_max_) any_value_clipped = i;
                cmd.kp = std::clamp(cmd.kp, lower_limbs_kp_min_, lower_limbs_kp_max_);
                cmd.kd = std::clamp(cmd.kd, lower_limbs_kd_min_, lower_limbs_kd_max_);
            }
            else
            {
                if (cmd.kp < upper_limbs_kp_min_ || cmd.kp > upper_limbs_kp_max_) any_value_clipped = i;
                cmd.kp = std::clamp(cmd.kp, upper_limbs_kp_min_, upper_limbs_kp_max_);
                cmd.kd = std::clamp(cmd.kd, upper_limbs_kd_min_, upper_limbs_kd_max_);
            }

            // // Check position
            // if (cmd.q < joint_info.q_min || cmd.q > joint_info.q_max)
            // {
            //     RCLCPP_WARN_ONCE(nh->get_logger(), "Clipping motor_cmd[%lu] q: %.3f", i, cmd.q);
            //     cmd.q = std::clamp(cmd.q, joint_info.q_min, joint_info.q_max);
            //     any_value_clipped = i;
            // }

            // Check velocity
            if (cmd.dq < -joint_info.dq_limit || cmd.dq > joint_info.dq_limit)
            {
                RCLCPP_WARN_ONCE(nh->get_logger(), "Clipping motor_cmd[%lu] dq: %.3f", i, cmd.dq);
                cmd.dq = std::clamp(cmd.dq, -joint_info.dq_limit, joint_info.dq_limit);
                any_value_clipped = i;
            }

            // Check torque
            if (cmd.tau < -joint_info.tau_limit || cmd.tau > joint_info.tau_limit)
            {
                RCLCPP_WARN_ONCE(nh->get_logger(), "Clipping motor_cmd[%lu] tau: %.3f", i, cmd.tau);
                cmd.tau = std::clamp(cmd.tau, -joint_info.tau_limit, joint_info.tau_limit);
                any_value_clipped = i;
            }

            if (any_value_clipped > 0)
            {
                RCLCPP_WARN(nh->get_logger(), "Joint %d were clipped to stay within safety limits", any_value_clipped);
            }
        }

        return true;
    }

    void BridgeCore::stopControlServiceCB_(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (controlStarted_) controlStarted_ = false;
        RCLCPP_INFO(nh->get_logger(), "Stopping control...");
        response->success = true;
        response->message = "Stop control service activated";
    }

    void BridgeCore::readyPositionControlServiceCB_(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (!controlStarted_)
            initControl_(bridge_interface::msg::RobotCmd());
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
        if (!controlStarted_)
            initControl_(bridge_interface::msg::RobotCmd());
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
        
        if (default_position.size() == numJoint_ )
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

        initControl_(default_cmd);
        RCLCPP_INFO(nh->get_logger(), "Starting control...");
        response->success = true;
        response->message = "start control with invalid size of default position, kp or kd";
    }
}
