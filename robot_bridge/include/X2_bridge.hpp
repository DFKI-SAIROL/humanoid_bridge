#ifndef SAIROL_BRIDGE_X2_BRIDGE_HPP_
#define SAIROL_BRIDGE_X2_BRIDGE_HPP_

#include <memory>
#include <chrono>
#include <thread>
#include <algorithm>
#include <string>
#include <functional>
#include <stdexcept>
#include <cstdint>
#include "bridge_core.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "aimdk_msgs/msg/joint_command_array.hpp"
#include "aimdk_msgs/msg/joint_state_array.hpp"
#include "aimdk_msgs/srv/get_system_state.hpp"
#include "aimdk_msgs/srv/migrate_system_state.hpp"


namespace sairol_bridge {


class X2Bridge : public BridgeCore {
public:
    explicit X2Bridge(rclcpp::Node::SharedPtr node);
    void stop() override;

private:

    void publishLowCommand_();

    void legStateHandler_(aimdk_msgs::msg::JointStateArray::SharedPtr message);
    void waistStateHandler_(aimdk_msgs::msg::JointStateArray::SharedPtr message);
    void armStateHandler_(aimdk_msgs::msg::JointStateArray::SharedPtr message);
    void headStateHandler_(aimdk_msgs::msg::JointStateArray::SharedPtr message);
    void imuStateHandler_(sensor_msgs::msg::Imu::SharedPtr message);

    bool checkJointStateMessage_(aimdk_msgs::msg::JointStateArray::SharedPtr message, size_t message_count, size_t state_offset, size_t active_count, std::string group_name);
    bool checkStateFreshness_() override;

    bool initControl_(bridge_interface::msg::RobotCmd default_cmd) override;
    void finishControl_() override;
    void enterDevelopMode_();
    bool checkExternalPublisher_(std::string topic_name);

    rclcpp::Subscription<aimdk_msgs::msg::JointStateArray>::SharedPtr legStateSubscriber_;
    rclcpp::Subscription<aimdk_msgs::msg::JointStateArray>::SharedPtr waistStateSubscriber_;
    rclcpp::Subscription<aimdk_msgs::msg::JointStateArray>::SharedPtr armStateSubscriber_;
    rclcpp::Subscription<aimdk_msgs::msg::JointStateArray>::SharedPtr headStateSubscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuStateSubscriber_;

    rclcpp::Publisher<aimdk_msgs::msg::JointCommandArray>::SharedPtr legCommandPublisher_;
    rclcpp::Publisher<aimdk_msgs::msg::JointCommandArray>::SharedPtr waistCommandPublisher_;
    rclcpp::Publisher<aimdk_msgs::msg::JointCommandArray>::SharedPtr armCommandPublisher_;
    rclcpp::Publisher<aimdk_msgs::msg::JointCommandArray>::SharedPtr headCommandPublisher_;

    bool enableHead_{false};
    int headJointCount_{0};
    bool legStateValid_{false};
    std::chrono::steady_clock::time_point lastLegStateTime_;
    double legStateTimeout_{0.2};

    bool waistStateValid_{false};
    std::chrono::steady_clock::time_point lastWaistStateTime_;
    double waistStateTimeout_{0.2};
    bool armStateValid_{false};
    std::chrono::steady_clock::time_point lastArmStateTime_;
    double armStateTimeout_{0.2};
    bool headStateValid_{false};
    std::chrono::steady_clock::time_point lastHeadStateTime_;
    double headStateTimeout_{0.2};
    bool imuStateValid_{false};
    std::chrono::steady_clock::time_point lastImuStateTime_;
    double imuStateTimeout_{0.2};

    bool checkComponentSkew_{false};
    bool includeImuInTimeCheck_{true};
    int64_t maxComponentSkewNs_{0};
    builtin_interfaces::msg::Time legStateStamp_;
    builtin_interfaces::msg::Time waistStateStamp_;
    builtin_interfaces::msg::Time armStateStamp_;
    builtin_interfaces::msg::Time headStateStamp_;
    builtin_interfaces::msg::Time imuStateStamp_;

};
}
#endif // SAIROL_BRIDGE_X2_BRIDGE_HPP_