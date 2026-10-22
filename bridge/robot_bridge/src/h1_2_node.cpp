#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "H1_2_bridge.hpp"

int main(int argc, char **argv)
try
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

    if (bridge && rclcpp::ok())   // not after Ctrl-C in the constructor's waits: spin would throw
    {
        bridge->start();
        rclcpp::spin(nh);
    }
    bridge->stop();
    rclcpp::shutdown();

    return 0;
}
catch (const std::exception &)
{
    // Ctrl-C before spin (main's sleep, the parameter load, between an ok() check and the next
    // graph call): that call threw on the shut-down context. Exit as after spin; anything else aborts.
    if (!rclcpp::ok())
    {
        return 0;
    }
    throw;
}
