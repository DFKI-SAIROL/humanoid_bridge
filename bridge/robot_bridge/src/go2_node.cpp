#include "rclcpp/rclcpp.hpp"
#include "Go2_bridge.hpp"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <thread>


int main(int argc, char **argv)
try
{
    rclcpp::init(argc, argv);

    auto options = rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true);
    rclcpp::Node::SharedPtr nh = std::make_shared<rclcpp::Node>("robot_bridge", options);

    rclcpp::sleep_for(std::chrono::milliseconds(100));

    // The Go2 reads its remote: a config without the key still needs it to arm.
    if (!nh->has_parameter("estop.require_remote"))
    {
        RCLCPP_WARN(nh->get_logger(), "No estop.require_remote in the config: the Go2 default is true, "
                                      "arming needs the remote");
        nh->declare_parameter("estop.require_remote", true);
    }

    std::shared_ptr<sairol_bridge::BridgeCore> bridge;

    try
    {
        bridge = std::make_shared<sairol_bridge::Go2Bridge>(nh);
    }
    catch (const sairol_bridge::ConfigError &)   // logged: "Failed to load parameters."
    {
        rclcpp::shutdown();
        return 1;
    }

    // Read once above, while readers such as robot_web_commander read the live values: refuse every change.
    auto read_only = nh->add_on_set_parameters_callback([](const std::vector<rclcpp::Parameter> &) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = false;
        result.reason = "robot_bridge parameters are read-only: edit the config and restart the bridge";
        return result;
    });

    if (bridge && rclcpp::ok())   // not after Ctrl-C in the constructor's waits: spin would throw
    {
        bridge->start();
        rclcpp::spin(nh);
    }

    bridge->stop();
    std::thread([] {
        std::this_thread::sleep_for(std::chrono::seconds(3));
        std::fflush(nullptr);
        std::_Exit(0);
    }).detach();
    return 0;
}
catch (const std::exception &)
{
    if (!rclcpp::ok())
    {
        return 0;
    }
    throw;
}
