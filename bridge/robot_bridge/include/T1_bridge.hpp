#ifndef SAIROL_BRIDGE_T1_BRIDGE_HPP_
#define SAIROL_BRIDGE_T1_BRIDGE_HPP_

#include <memory>
#include <chrono>
#include <thread>
#include <algorithm>
#include "bridge_core.hpp"
#include "booster_mode.hpp"
#include "booster_interface/srv/rpc_service.hpp"
#include "booster_interface/msg/low_state.hpp"
#include "booster_interface/msg/low_cmd.hpp"
#include "booster_interface/msg/remote_controller_state.hpp"


namespace sairol_bridge {

// Booster T1 on booster_interface: /low_state in, /joint_ctrl out, its mode over booster_rpc_service.
// Custom mode on arming, damping when the bridge stops. The remote is the T1 gamepad on
// /remote_controller_state (estop.remote_keys: t1).
class T1Bridge : public BridgeCore {
public:
    explicit T1Bridge(rclcpp::Node::SharedPtr node);
    void stop() override;

private:
    void lowStateHandler_(booster_interface::msg::LowState::SharedPtr message);
    void remoteHandler_(booster_interface::msg::RemoteControllerState::SharedPtr message);
    void publishLowCommand_() override;
    void publishStopCommand_() override;
    bool initControl_(bridge_interface::msg::RobotCmd default_cmd) override;
    void finishControl_() override;
    bool robotAllowsArming_(std::string &why) override;
    estop::Cause robotFault_() override;
    void watchJointCtrl_();
    void dampBeforeShutdown_();
    // Asks for the mode; the reply is logged. False when the service is not there.
    bool switchMode_(booster::RobotMode mode);

    rclcpp::Subscription<booster_interface::msg::LowState>::SharedPtr lowStateSubscriber_;
    rclcpp::Subscription<booster_interface::msg::RemoteControllerState>::SharedPtr remoteSubscriber_;
    rclcpp::Publisher<booster_interface::msg::LowCmd>::SharedPtr lowCommandPublisher_;
    rclcpp::Client<booster_interface::srv::RpcService>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr jointCtrlWatchTimer_;
    estop::Cause lastRemote_{estop::Cause::None};          // executor thread only
    estop::Cause lastJointCtrlFault_{estop::Cause::None};  // executor thread only
    bool stateLogged_{false};
};
}
#endif // SAIROL_BRIDGE_T1_BRIDGE_HPP_
