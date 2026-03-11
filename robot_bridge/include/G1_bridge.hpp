#ifndef SAIROL_BRIDGE_G1_BRIDGE_HPP_
#define SAIROL_BRIDGE_G1_BRIDGE_HPP_

#include <memory>
#include <chrono>
#include <thread>
#include <algorithm>
#include "bridge_core.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "unitree_hg/msg/low_cmd.hpp"
#include "unitree_hg/msg/low_state.hpp"
#include "unitree_hg/msg/motor_cmd.hpp"
#include "common/motor_crc_hg.h"
#include "unitree_go/msg/wireless_controller.hpp"
#include "std_msgs/msg/string.hpp"


namespace sairol_bridge {


enum WirelessKey_H1_G1 : uint32_t
{
    KEY_R1     = 1 << 0,    // 0b00000000 00000001 = 1
    KEY_L1     = 1 << 1,    // 0b00000000 00000010 = 2
    KEY_START  = 1 << 2,    // 0b00000000 00000100 = 4
    KEY_SELECT = 1 << 3,    // 0b00000000 00001000 = 8
    KEY_R2     = 1 << 4,    // 0b00000000 00010000 = 16
    KEY_L2     = 1 << 5,    // 0b00000000 00100000 = 32

    KEY_A      = 1 << 8,    // 0b00000001 00000000 = 256
    KEY_B      = 1 << 9,    // 0b00000010 00000000 = 512
    KEY_X      = 1 << 10,   // 0b00000100 00000000 = 1024
    KEY_Y      = 1 << 11,   // 0b00001000 00000000 = 2048

    KEY_UP     = 1 << 12,   // 0b00010000 00000000 = 4096
    KEY_RIGHT  = 1 << 13,   // 0b00100000 00000000 = 8192
    KEY_DOWN   = 1 << 14,   // 0b01000000 00000000 = 16384
    KEY_LEFT   = 1 << 15    // 0b10000000 00000000 = 32768
};
enum PRorAB { PR = 0, AB = 1 };

class G1Bridge : public BridgeCore {
public:
    explicit G1Bridge(rclcpp::Node::SharedPtr node);
    void stop() override;

private:

    void wireless_callback(unitree_go::msg::WirelessController::SharedPtr data);
    void publishLowCommand_();
    void publishLowCommandOLD_();
    void lowStateHandler_(unitree_hg::msg::LowState::SharedPtr message);
    bool initControl_(bridge_interface::msg::RobotCmd default_cmd) override;
    void finishControl_() override; 
    bool checkExternalPublisher_(std::string topic_name);

    rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr lowStateSubscriber_;
    rclcpp::Subscription<unitree_go::msg::WirelessController>::SharedPtr remoteControlSubscriber_;
    rclcpp::Publisher<unitree_hg::msg::LowCmd>::SharedPtr lowCommandPublisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmdInputSubscriber_;
    void cmdInputCallback_(std_msgs::msg::String::SharedPtr msg);
    int mode_machine_{0};


    

};
}
#endif // SAIROL_BRIDGE_T1_BRIDGE_HPP_