#include "X2_bridge.hpp"


using namespace std::chrono_literals;

sairol_bridge::X2Bridge::X2Bridge(rclcpp::Node::SharedPtr node) : BridgeCore(node)
{

    nh->get_parameter_or("enable_head", enableHead_, false);
    nh->get_parameter_or("head_joint_count", headJointCount_, 0);

    if ((enableHead_ && headJointCount_ != 1) ||
        (!enableHead_ && headJointCount_ != 0) ||
        numJoint_ != 29 + headJointCount_)
    {
        throw std::invalid_argument("X2 requires 29 joints without the head or 30 joints with head yaw.");
    }

    nh->get_parameter_or("leg_state_timeout", legStateTimeout_, 0.2);
    if (!std::isfinite(legStateTimeout_) || legStateTimeout_ <= 0.0)
    {
        throw std::invalid_argument("leg_state_timeout must be finite and positive.");
    }
    nh->get_parameter_or("waist_state_timeout", waistStateTimeout_, 0.2);
    if (!std::isfinite(waistStateTimeout_) || waistStateTimeout_ <= 0.0)
    {
        throw std::invalid_argument("waist_state_timeout must be finite and positive.");
    }
    nh->get_parameter_or("arm_state_timeout", armStateTimeout_, 0.2);
    if (!std::isfinite(armStateTimeout_) || armStateTimeout_ <= 0.0)
    {
        throw std::invalid_argument("arm_state_timeout must be finite and positive.");
    }
    nh->get_parameter_or("head_state_timeout", headStateTimeout_, 0.2);
    if (!std::isfinite(headStateTimeout_) || headStateTimeout_ <= 0.0)
    {
        throw std::invalid_argument("head_state_timeout must be finite and positive.");
    }
    nh->get_parameter_or("imu_state_timeout", imuStateTimeout_, 0.2);
    if (!std::isfinite(imuStateTimeout_) || imuStateTimeout_ <= 0.0)
    {
        throw std::invalid_argument("imu_state_timeout must be finite and positive.");
    }

    nh->get_parameter_or("check_component_skew", checkComponentSkew_, false);
    nh->get_parameter_or("include_imu_in_time_check", includeImuInTimeCheck_, true);
    if (checkComponentSkew_)
    {
        double seconds = 0.0;
        if (!nh->get_parameter("max_component_skew", seconds) || !std::isfinite(seconds) || seconds <= 0.0)
        {
            throw std::invalid_argument("max_component_skew must be explicitly configured as finite positive seconds.");
        }
        const long double nanoseconds = static_cast<long double>(seconds) * 1000000000.0L;
        if (nanoseconds < 1.0L || nanoseconds > static_cast<long double>(std::numeric_limits<int64_t>::max()))
        {
            throw std::invalid_argument("max_component_skew is outside the supported nanosecond range.");
        }
        maxComponentSkewNs_ = static_cast<int64_t>(nanoseconds);
    }

    auto qos = rclcpp::SensorDataQoS();
    qos.keep_last(1);

    legStateSubscriber_ = nh->create_subscription<aimdk_msgs::msg::JointStateArray>(
        "/aima/hal/joint/leg/state", qos, std::bind(&sairol_bridge::X2Bridge::legStateHandler_, this, std::placeholders::_1));

    waistStateSubscriber_ = nh->create_subscription<aimdk_msgs::msg::JointStateArray>(
        "/aima/hal/joint/waist/state", qos, std::bind(&sairol_bridge::X2Bridge::waistStateHandler_, this, std::placeholders::_1));

    armStateSubscriber_ = nh->create_subscription<aimdk_msgs::msg::JointStateArray>(
        "/aima/hal/joint/arm/state", qos, std::bind(&sairol_bridge::X2Bridge::armStateHandler_, this, std::placeholders::_1));

    if (enableHead_)
    {
        headStateSubscriber_ = nh->create_subscription<aimdk_msgs::msg::JointStateArray>(
            "/aima/hal/joint/head/state", qos, std::bind(&sairol_bridge::X2Bridge::headStateHandler_, this, std::placeholders::_1));
    }

    imuStateSubscriber_ = nh->create_subscription<sensor_msgs::msg::Imu>(
        "/aima/hal/imu/torso/state", qos, std::bind(&sairol_bridge::X2Bridge::imuStateHandler_, this, std::placeholders::_1));

    while (!checkExternalPublisher_("/aima/hal/joint/leg/command") ||
           !checkExternalPublisher_("/aima/hal/joint/waist/command") ||
           !checkExternalPublisher_("/aima/hal/joint/arm/command") ||
           (enableHead_ && !checkExternalPublisher_("/aima/hal/joint/head/command")))
    {
        rclcpp::sleep_for(std::chrono::milliseconds(1000));
    }

    legCommandPublisher_ = nh->create_publisher<aimdk_msgs::msg::JointCommandArray>(
        "/aima/hal/joint/leg/command", qos);

    waistCommandPublisher_ = nh->create_publisher<aimdk_msgs::msg::JointCommandArray>(
        "/aima/hal/joint/waist/command", qos);

    armCommandPublisher_ = nh->create_publisher<aimdk_msgs::msg::JointCommandArray>(
        "/aima/hal/joint/arm/command", qos);

    if (enableHead_)
    {
        headCommandPublisher_ = nh->create_publisher<aimdk_msgs::msg::JointCommandArray>(
            "/aima/hal/joint/head/command", qos);
    }
}

bool sairol_bridge::X2Bridge::checkExternalPublisher_(std::string topic_name)
{
    auto publishers_info = nh->get_publishers_info_by_topic(topic_name);
    int publisher_count = publishers_info.size();
    if (publisher_count > 0)
    {
        RCLCPP_ERROR_STREAM(nh->get_logger(),
                           "Detected " << publisher_count << " publishers on "
                           << topic_name.c_str());
        return false;
    }
    return true;
}


void sairol_bridge::X2Bridge::stop()
{
    // Wait for the control thread to finish
    if (controlThread_.joinable())
    {
        controlThread_.join();
    }

    RCLCPP_INFO(nh->get_logger(), "X2 Bridge stopped.");
}

void sairol_bridge::X2Bridge::legStateHandler_(aimdk_msgs::msg::JointStateArray::SharedPtr msg)
{
    std::unique_lock<std::mutex> lock(mutex_);

    legStateValid_ = false;
    if (!checkJointStateMessage_(msg, 12, 0, 12, "leg"))
    {
        return;
    }

    for (size_t i = 0; i < 12; ++i)
    {
        currentState_.motor_state[0 + i].q = msg->joints[i].position;
        currentState_.motor_state[0 + i].dq = msg->joints[i].velocity;
        currentState_.motor_state[0 + i].tau_est = msg->joints[i].effort;
        // Joint acceleration is not provided; use a placeholder
        currentState_.motor_state[0 + i].ddq = 0.0;
    }

    legStateStamp_ = msg->header.meas_stamp;
    lastLegStateTime_ = std::chrono::steady_clock::now();
    legStateValid_ = true;
}

void sairol_bridge::X2Bridge::waistStateHandler_(aimdk_msgs::msg::JointStateArray::SharedPtr msg)
{
    std::unique_lock<std::mutex> lock(mutex_);

    waistStateValid_ = false;
    if (!checkJointStateMessage_(msg, 3, 12, 3, "waist"))
    {
        return;
    }

    for (size_t i = 0; i < 3; ++i)
    {
        currentState_.motor_state[12 + i].q = msg->joints[i].position;
        currentState_.motor_state[12 + i].dq = msg->joints[i].velocity;
        currentState_.motor_state[12 + i].tau_est = msg->joints[i].effort;
        // Joint acceleration is not provided; use a placeholder
        currentState_.motor_state[12 + i].ddq = 0.0;
    }

    waistStateStamp_ = msg->header.meas_stamp;
    lastWaistStateTime_ = std::chrono::steady_clock::now();
    waistStateValid_ = true;
}

void sairol_bridge::X2Bridge::armStateHandler_(aimdk_msgs::msg::JointStateArray::SharedPtr msg)
{
    std::unique_lock<std::mutex> lock(mutex_);

    armStateValid_ = false;
    if (!checkJointStateMessage_(msg, 14, 15, 14, "arm"))
    {
        return;
    }

    for (size_t i = 0; i < 14; ++i)
    {
        currentState_.motor_state[15 + i].q = msg->joints[i].position;
        currentState_.motor_state[15 + i].dq = msg->joints[i].velocity;
        currentState_.motor_state[15 + i].tau_est = msg->joints[i].effort;
        // Joint acceleration is not provided; use a placeholder
        currentState_.motor_state[15 + i].ddq = 0.0;
    }

    armStateStamp_ = msg->header.meas_stamp;
    lastArmStateTime_ = std::chrono::steady_clock::now();
    armStateValid_ = true;
}

void sairol_bridge::X2Bridge::headStateHandler_(aimdk_msgs::msg::JointStateArray::SharedPtr msg)
{
    std::unique_lock<std::mutex> lock(mutex_);

    headStateValid_ = false;
    if (!enableHead_)
    {
        return;
    }
    if (!checkJointStateMessage_(msg, 2, 29, static_cast<size_t>(headJointCount_), "head"))
    {
        return;
    }

    for (size_t i = 0; i < static_cast<size_t>(headJointCount_); ++i)
    {
        currentState_.motor_state[29 + i].q = msg->joints[i].position;
        currentState_.motor_state[29 + i].dq = msg->joints[i].velocity;
        currentState_.motor_state[29 + i].tau_est = msg->joints[i].effort;
        // Joint acceleration is not provided; use a placeholder
        currentState_.motor_state[29 + i].ddq = 0.0;
    }

    headStateStamp_ = msg->header.meas_stamp;
    lastHeadStateTime_ = std::chrono::steady_clock::now();
    headStateValid_ = true;
}

void sairol_bridge::X2Bridge::imuStateHandler_(sensor_msgs::msg::Imu::SharedPtr msg)
{
    std::unique_lock<std::mutex> lock(mutex_);
    imuStateValid_ = false;

    if (msg->orientation_covariance[0] == -1.0 ||
        msg->angular_velocity_covariance[0] == -1.0 ||
        msg->linear_acceleration_covariance[0] == -1.0)
    {
        RCLCPP_ERROR(nh->get_logger(), "X2 IMU reports unavailable measurements.");
        return;
    }

    const double values[] = {msg->orientation.w, msg->orientation.x,
        msg->orientation.y, msg->orientation.z,
        msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z,
        msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z};
    for (const auto value : values)
    {
        if (!std::isfinite(value) || std::abs(value) > std::numeric_limits<float>::max())
        {
            RCLCPP_ERROR(nh->get_logger(), "X2 IMU contains invalid values.");
            return;
        }
    }

    const double norm = std::sqrt(values[0] * values[0] + values[1] * values[1] +
                                  values[2] * values[2] + values[3] * values[3]);
    if (norm < 1e-12)
    {
        RCLCPP_ERROR(nh->get_logger(), "X2 IMU quaternion has zero norm.");
        return;
    }

    const double w = values[0] / norm;
    const double x = values[1] / norm;
    const double y = values[2] / norm;
    const double z = values[3] / norm;

    // Keep the HAL torso frame; verify alignment on the robot
    // Quaternion order: w, x, y, z; angles in radians
    imu_.quaternion = {static_cast<float>(w), static_cast<float>(x),
                       static_cast<float>(y), static_cast<float>(z)};
    imu_.rpy[0] = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
    imu_.rpy[1] = std::asin(std::clamp(2.0 * (w * y - z * x), -1.0, 1.0));
    imu_.rpy[2] = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    for (size_t i = 0; i < 3; ++i)
    {
        imu_.gyroscope[i] = values[4 + i];
        imu_.accelerometer[i] = values[7 + i];
    }

    imuStateStamp_ = msg->header.stamp;
    lastImuStateTime_ = std::chrono::steady_clock::now();
    imuStateValid_ = true;
}

void sairol_bridge::X2Bridge::publishLowCommand_()
{
    rclcpp::Time current = nh->get_clock()->now();

    float_t phase = 1.0f;
    if (tFinal_ - tStart_ > 1e-6) {
        phase = (current.seconds() - tStart_) / (tFinal_ - tStart_);
    }

    phase = std::clamp(phase, 0.0f, 1.0f); // Ensure phase is between 0 and 1

    aimdk_msgs::msg::JointCommandArray leg_cmd;
    aimdk_msgs::msg::JointCommandArray waist_cmd;
    aimdk_msgs::msg::JointCommandArray arm_cmd;
    aimdk_msgs::msg::JointCommandArray head_cmd;

    leg_cmd.joints.resize(12);
    waist_cmd.joints.resize(3);
    arm_cmd.joints.resize(14);
    if (enableHead_)
    {
        head_cmd.joints.resize(2);
    }

    for (int i = 0; i < numJoint_; ++i)
    {
        aimdk_msgs::msg::JointCommand cmd;
        auto &last_cmd = lastCommand_.motor_cmd[i];
        auto &joint_info = joints_[i];

        if (1e-6 < cmdInterpOrder_ && cmdInterpOrder_ < 1.0 - 1e-6)
        {
            // Low pass filter for cmd
            cmd.position = last_cmd.q * cmdInterpOrder_ + cmdParams_[i].q_0 * (1 - cmdInterpOrder_);
            cmd.velocity = last_cmd.dq * cmdInterpOrder_ + cmdParams_[i].dq_0 * (1 - cmdInterpOrder_);
        }
        else {
            // Interpolation
            cmd.position = cmdParams_[i].q_0 + cmdParams_[i].q_1 * phase;
            cmd.velocity = cmdParams_[i].dq_0 + cmdParams_[i].dq_1 * phase;
        }

        cmd.stiffness = cmdParams_[i].kp_0 + cmdParams_[i].kp_1 * phase;
        cmd.damping = cmdParams_[i].kd_0 + cmdParams_[i].kd_1 * phase;
        cmd.velocity = std::clamp<double>(cmd.velocity, -joint_info.dq_limit, joint_info.dq_limit);

        if (torqueControl_)
        {
            cmd.effort = cmd.stiffness * (cmd.position - currentState_.motor_state[i].q) + cmd.damping * (cmd.velocity - currentState_.motor_state[i].dq) + cmdParams_[i].tau_0;
            cmd.effort = std::clamp<double>(cmd.effort, -joint_info.tau_limit, joint_info.tau_limit);
            cmd.stiffness = 0.0;
            cmd.damping = 0.0;
        }
        else
        {
            cmd.effort = cmdParams_[i].tau_0 + cmdParams_[i].tau_1 * phase;
        }

        // Limit position when stiffness is positive
        if (cmd.stiffness > 0.0)
        {
            cmd.position = std::clamp(cmd.position,
                (cmd.damping * (currentState_.motor_state[i].dq - cmd.velocity) - joint_info.tau_limit) / cmd.stiffness + currentState_.motor_state[i].q,
                (cmd.damping * (currentState_.motor_state[i].dq - cmd.velocity) + joint_info.tau_limit) / cmd.stiffness + currentState_.motor_state[i].q);
        }

        last_cmd.q = cmd.position;
        last_cmd.dq = cmd.velocity;
        last_cmd.kp = cmd.stiffness;
        last_cmd.kd = cmd.damping;
        last_cmd.tau = cmd.effort;

        if (i < 12)
        {
            leg_cmd.joints[i] = cmd;
        }
        else if (i < 15)
        {
            waist_cmd.joints[i - 12] = cmd;
        }
        else if (i < 29)
        {
            arm_cmd.joints[i - 15] = cmd;
        }
        else
        {
            head_cmd.joints[i - 29] = cmd;
        }
    }

    leg_cmd.header.stamp = current;
    waist_cmd.header.stamp = current;
    arm_cmd.header.stamp = current;

    legCommandPublisher_->publish(leg_cmd);
    waistCommandPublisher_->publish(waist_cmd);
    armCommandPublisher_->publish(arm_cmd);
    if (enableHead_)
    {
        head_cmd.header.stamp = current;
        headCommandPublisher_->publish(head_cmd);
    }
}

bool sairol_bridge::X2Bridge::initControl_(bridge_interface::msg::RobotCmd default_cmd)
{
    for (int i = 0; i < numJoint_; ++i)
    {
        lowCommandDesired_.motor_cmd[i].q = currentState_.motor_state[i].q;
        lowCommandDesired_.motor_cmd[i].kp = joints_[i].kp;
        lowCommandDesired_.motor_cmd[i].kd = joints_[i].kd;
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
            lowCommandDesired_.motor_cmd[i].q = default_cmd.motor_cmd[i].q;
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

void sairol_bridge::X2Bridge::finishControl_() {
    RCLCPP_INFO(nh->get_logger(), "finishControl_ called from X2Bridge");
}

bool sairol_bridge::X2Bridge::checkJointStateMessage_(aimdk_msgs::msg::JointStateArray::SharedPtr msg, size_t message_count, size_t state_offset, size_t active_count, std::string group_name)
{
    // State callback already holds mutex_
    if (msg->joints.size() != message_count || currentState_.motor_state.size() < state_offset + active_count)
    {
        RCLCPP_ERROR(nh->get_logger(), "X2 %s state length mismatch.", group_name.c_str());
        return false;
    }

    // Check group state
    if (msg->state.value != aimdk_msgs::msg::DomainErrorState::NONE)
    {
        RCLCPP_ERROR(nh->get_logger(), "X2 %s group reports state %u.", group_name.c_str(), msg->state.value);
        return false;
    }

    const double max_value = std::numeric_limits<float>::max();
    for (size_t i = 0; i < active_count; ++i)
    {
        const auto &joint = msg->joints[i];
        if (!std::isfinite(joint.position) ||
            !std::isfinite(joint.velocity) ||
            !std::isfinite(joint.effort) ||
            std::abs(joint.position) > max_value ||
            std::abs(joint.velocity) > max_value ||
            std::abs(joint.effort) > max_value || joint.error_code != 0)
        {
            RCLCPP_ERROR(nh->get_logger(), "X2 %s joint %zu has invalid data or error code %u.", group_name.c_str(), i, joint.error_code);
            return false;
        }
    }
    return true;
}

bool sairol_bridge::X2Bridge::checkStateFreshness_()
{
    // Core calls this without mutex_ held; do not lock it before this call
    std::unique_lock<std::mutex> lock(mutex_);
    const auto now = std::chrono::steady_clock::now();
    if ((!legStateValid_ || std::chrono::duration<double>(now - lastLegStateTime_).count() > legStateTimeout_))
    {
        RCLCPP_ERROR(nh->get_logger(), "X2 leg state is missing, invalid or stale.");
        return false;
    }
    if ((!waistStateValid_ || std::chrono::duration<double>(now - lastWaistStateTime_).count() > waistStateTimeout_))
    {
        RCLCPP_ERROR(nh->get_logger(), "X2 waist state is missing, invalid or stale.");
        return false;
    }
    if ((!armStateValid_ || std::chrono::duration<double>(now - lastArmStateTime_).count() > armStateTimeout_))
    {
        RCLCPP_ERROR(nh->get_logger(), "X2 arm state is missing, invalid or stale.");
        return false;
    }
    if (enableHead_ && (!headStateValid_ || std::chrono::duration<double>(now - lastHeadStateTime_).count() > headStateTimeout_))
    {
        RCLCPP_ERROR(nh->get_logger(), "X2 head state is missing, invalid or stale.");
        return false;
    }
    if ((!imuStateValid_ || std::chrono::duration<double>(now - lastImuStateTime_).count() > imuStateTimeout_))
    {
        RCLCPP_ERROR(nh->get_logger(), "X2 imu state is missing, invalid or stale.");
        return false;
    }
    if (checkComponentSkew_)
    {
        // Compare source timestamps; verify a common clock on the robot
        const builtin_interfaces::msg::Time stamps[] = {legStateStamp_, waistStateStamp_, armStateStamp_,
            enableHead_ ? headStateStamp_ : legStateStamp_,
            includeImuInTimeCheck_ ? imuStateStamp_ : legStateStamp_};
        int64_t oldest_stamp = std::numeric_limits<int64_t>::max();
        int64_t newest_stamp = 0;
        for (const auto &stamp : stamps)
        {
            if (stamp.sec < 0 || stamp.nanosec >= 1000000000U || (stamp.sec == 0 && stamp.nanosec == 0))
            {
                RCLCPP_ERROR(nh->get_logger(), "X2 component timestamp is zero or invalid.");
                return false;
            }
            const int64_t stamp_ns = static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
            oldest_stamp = std::min(oldest_stamp, stamp_ns);
            newest_stamp = std::max(newest_stamp, stamp_ns);
        }
        if (newest_stamp - oldest_stamp > maxComponentSkewNs_)
        {
            RCLCPP_ERROR(nh->get_logger(), "X2 component timestamp skew %.9f s exceeds its limit.", (newest_stamp - oldest_stamp) / 1000000000.0);
            return false;
        }
    }
    return true;
}
