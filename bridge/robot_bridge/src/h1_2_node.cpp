#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "H1_2_bridge.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);
    rclcpp::Node::SharedPtr nh = std::make_shared<rclcpp::Node>("robot_bridge", options);

    std::shared_ptr<sairol_bridge::BridgeCore> bridge;
    try
    {
        auto h1_2_bridge = std::make_shared<sairol_bridge::H1_2Bridge>(nh);
        bridge = h1_2_bridge;
    }
    catch (const sairol_bridge::ConfigError &)   // logged: "Failed to load parameters."
    {
        rclcpp::shutdown();
        return 1;
    }

    if (bridge)
    {
        bridge->start();
    }

    rclcpp::spin(nh);
    bridge->stop();
    rclcpp::shutdown();

    return 0;
}
