#include "X2_bridge.hpp"


using namespace std::chrono_literals;

sairol_bridge::X2Bridge::X2Bridge(rclcpp::Node::SharedPtr node) : BridgeCore(node)
{

    nh->get_parameter_or("enable_head", enableHead_, false);
    nh->get_parameter_or("head_joint_count", headJointCount_, 0);

    if ((enableHead_ && headJointCount_ != 1 && headJointCount_ != 2) ||
        (!enableHead_ && headJointCount_ != 0) ||
        numJoint_ != 29 + headJointCount_)
    {
        throw std::invalid_argument("X2 requires 29 body joints and explicitly configured active head axes (0, 1 or 2).");
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
        // X2 does not provide joint acceleration; this is a placeholder.
        currentState_.motor_state[0 + i].ddq = 0.0;
    }

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
        // X2 does not provide joint acceleration; this is a placeholder.
        currentState_.motor_state[12 + i].ddq = 0.0;
    }

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
        // X2 does not provide joint acceleration; this is a placeholder.
        currentState_.motor_state[15 + i].ddq = 0.0;
    }

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
        // X2 does not provide joint acceleration; this is a placeholder.
        currentState_.motor_state[29 + i].ddq = 0.0;
    }

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

    // Preserve the HAL torso frame. Its alignment must be verified before robot control.
    // Bridge quaternion order is w, x, y, z; angles are radians.
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

    lastImuStateTime_ = std::chrono::steady_clock::now();
    imuStateValid_ = true;
}

bool sairol_bridge::X2Bridge::checkJointStateMessage_(aimdk_msgs::msg::JointStateArray::SharedPtr msg, size_t message_count, size_t state_offset, size_t active_count, std::string group_name)
{
    // Called with mutex_ held by the state callback; do not lock it again here.
    if (msg->joints.size() != message_count || currentState_.motor_state.size() < state_offset + active_count)
    {
        RCLCPP_ERROR(nh->get_logger(), "X2 %s state length mismatch.", group_name.c_str());
        return false;
    }

    // Only the normal group state is accepted for control.
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
    // Called by BridgeCore::checkState_ before the common data checks.
    // Existing locking behavior is unchanged; this function does not acquire mutex_.
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
    return true;
}
