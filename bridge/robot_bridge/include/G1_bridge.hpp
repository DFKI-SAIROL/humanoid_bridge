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
#include "unitree_hg/msg/hand_cmd.hpp"
#include "unitree_hg/msg/hand_state.hpp"
#include "bridge_interface/msg/hand_cmd.hpp"
#include "common/motor_crc_hg.h"


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

// Dex3-1 hands. A path of their own: own topics, own enable, own timer. They are
// not part of the 29-joint body vector -- different device, different state
// topic, different rate.
enum HandSide { HAND_LEFT = 0, HAND_RIGHT = 1, NUM_HANDS = 2 };

// Everything one hand owns. Two of these, indexed by HandSide.
struct HandChannel
{
    std::string name;

    std::vector<Joint> joints;      // numHandJoint_, in DDS order
    std::vector<CmdParams> params;

    bridge_interface::msg::HandCmd desired;
    bridge_interface::msg::HandCmd last;   // what was published last tick

    unitree_hg::msg::HandState state;
    rclcpp::Time state_time;
    bool state_seen{false};

    bool live{false};               // false once a guard or the watchdog fired
    double t_start{0}, t_final{0}, t_valid{0};

    rclcpp::Subscription<unitree_hg::msg::HandState>::SharedPtr state_sub;
    rclcpp::Subscription<bridge_interface::msg::HandCmd>::SharedPtr cmd_sub;
    rclcpp::Publisher<unitree_hg::msg::HandCmd>::SharedPtr cmd_pub;
};

class G1Bridge : public BridgeCore {
public:
    explicit G1Bridge(rclcpp::Node::SharedPtr node);
    void stop() override;

private:

    void publishLowCommand_();
    void publishStopCommand_() override;
    void publishLowCommandOLD_();
    void lowStateHandler_(unitree_hg::msg::LowState::SharedPtr message);
    bool initControl_(bridge_interface::msg::RobotCmd default_cmd) override;
    void finishControl_() override;
    bool checkExternalPublisher_(std::string topic_name);
    // Refused while anything besides this bridge publishes /lowcmd (a restarted robot's motion service).
    bool robotAllowsArming_(std::string &why) override;

    // --- Dex3 hands --------------------------------------------------------
    // All of these run on the executor thread (subscriptions and the wall timer
    // alike), which a single-threaded spin serialises. The body path's control
    // thread touches none of it, so the hands need no lock of their own.
    bool loadHandParameters_();
    void handStateHandler_(int side, unitree_hg::msg::HandState::SharedPtr message);
    void handCmdCallBack_(int side, bridge_interface::msg::HandCmd::SharedPtr message);
    bool checkHandCommand_(int side, bridge_interface::msg::HandCmd::SharedPtr message);
    bool checkHandState_(int side);
    void publishHandCommand_();
    void publishOneHand_(int side, bool limp);
    void startHandControlServiceCB_(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void stopHandControlServiceCB_(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                   std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    // The Dex3 mode bitfield: motor id in bits 0-3, status enable in bit 4,
    // timeout enable in bit 7. A motor left at mode 0 ignores the command, so
    // every motor gets this on every frame. Clients never see it.
    static uint8_t handMode_(int motor_index)
    {
        return static_cast<uint8_t>((motor_index & 0x0F) | (0x01 << 4) | (0x01 << 7));
    }

    rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr lowStateSubscriber_;
    rclcpp::Publisher<unitree_hg::msg::LowCmd>::SharedPtr lowCommandPublisher_;
    int mode_machine_{0};
    estop::Cause lastRemote_{estop::Cause::None};   // executor thread only

    HandChannel hands_[NUM_HANDS];
    rclcpp::TimerBase::SharedPtr handTimer_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr startHandControlService_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stopHandControlService_;

    std::vector<double> handReadyQ_;
    bool handStarted_{false};
    int numHandJoint_{7};
    double handControlDt_{0.01};
    double handStateTimeout_{0.2};
    float_t handKpMin_{0}, handKpMax_{0}, handKdMin_{0}, handKdMax_{0};
    float_t handTemperatureLimit_{80.0};
};
}
#endif // SAIROL_BRIDGE_T1_BRIDGE_HPP_