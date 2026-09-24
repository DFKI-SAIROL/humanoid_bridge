#ifndef SAIROL_BRIDGE_X2_BRIDGE_HPP_
#define SAIROL_BRIDGE_X2_BRIDGE_HPP_

#include <memory>
#include <chrono>
#include <thread>
#include <algorithm>
#include <string>
#include "bridge_core.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "aimdk_msgs/msg/joint_command_array.hpp"
#include "aimdk_msgs/msg/joint_state_array.hpp"


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

    bool initControl_(bridge_interface::msg::RobotCmd default_cmd) override;
    void finishControl_() override;
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
    bool legStateValid_{false};
    std::chrono::steady_clock::time_point lastLegStateTime_;
    aimdk_msgs::msg::JointStateArray::SharedPtr legStateMessage_;

};
}
#endif // SAIROL_BRIDGE_X2_BRIDGE_HPP_