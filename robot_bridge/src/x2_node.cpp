#include "rclcpp/rclcpp.hpp"
#include "X2_bridge.hpp"


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto options = rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true);
    rclcpp::Node::SharedPtr nh = std::make_shared<rclcpp::Node>("robot_bridge", options);

    rclcpp::sleep_for(std::chrono::milliseconds(100));

    std::string robot_name;
    nh->get_parameter("robot_name", robot_name);

    std::shared_ptr<sairol_bridge::BridgeCore> bridge;

    auto x2_bridge = std::make_shared<sairol_bridge::X2Bridge>(nh);
    bridge = x2_bridge;

    if (bridge)
    {
        bridge->start();
        rclcpp::spin(nh);
    }

    bridge->stop();
    return 0;
}
