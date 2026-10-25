#ifndef SAIROL_BRIDGE_GO2_BRIDGE_HPP_
#define SAIROL_BRIDGE_GO2_BRIDGE_HPP_

#include <memory>
#include <chrono>
#include <thread>
#include <algorithm>
#include "bridge_core.hpp"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/low_state.hpp"


namespace sairol_bridge {

// Unitree Go2 on unitree_go: /lowstate in, /lowcmd out (go2_frame.hpp), the Unitree remote on
// LowState.wireless_remote as on the G1. 12 motors; slots 12-19 of the frame stay stopped.
class Go2Bridge : public BridgeCore {
public:
    explicit Go2Bridge(rclcpp::Node::SharedPtr node);
    void stop() override;

private:
    void lowStateHandler_(unitree_go::msg::LowState::SharedPtr message);
    void publishLowCommand_() override;
    void publishStopCommand_() override;
    bool initControl_(bridge_interface::msg::RobotCmd default_cmd) override;
    void finishControl_() override;
    // Before the context shuts down (SIGINT, SIGTERM): damp frames for SHUTDOWN_DAMP_S, not silence at once.
    void dampBeforeShutdown_();
    bool checkExternalPublisher_(const std::string &topic_name);
    // Refused while anything besides this bridge publishes /lowcmd (the sport service after a restart).
    bool robotAllowsArming_(std::string &why) override;
    estop::Cause robotFault_() override;
    void watchLowcmd_();

    rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr lowStateSubscriber_;
    rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr lowCommandPublisher_;
    rclcpp::TimerBase::SharedPtr lowcmdWatchTimer_;
    estop::Cause lastRemote_{estop::Cause::None};       // executor thread only
    estop::Cause lastLowcmdFault_{estop::Cause::None};  // executor thread only
    bool stateLogged_{false};
};
}
#endif // SAIROL_BRIDGE_GO2_BRIDGE_HPP_
