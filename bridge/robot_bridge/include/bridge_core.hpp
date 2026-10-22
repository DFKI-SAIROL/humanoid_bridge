// bridge_core.hpp ------------------------------------------------------------
#pragma once

#include "rclcpp/rclcpp.hpp"
#include "bridge_interface/msg/robot_cmd.hpp"
#include "bridge_interface/msg/low_cmd.hpp"
#include "bridge_interface/msg/low_state.hpp"
#include "bridge_interface/msg/imu_state.hpp"
#include "bridge_interface/srv/set_default_position.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/string.hpp"
#include "joint.hpp"
#include "estop.hpp"

#include <atomic>
#include <string>
#include <vector>
#include <limits>
#include <mutex>
#include <thread>
#include <cmath>
#include <algorithm>
#include <cassert>

namespace sairol_bridge
{ 
    struct CmdParams
    {
        float_t q_0{0}, q_1{0};
        float_t dq_0{0}, dq_1{0};
        float_t tau_0{0}, tau_1{0};
        float_t kp_0{0}, kp_1{0};
        float_t kd_0{0}, kd_1{0};
    };

    class BridgeCore
    {
    public:
        explicit BridgeCore(rclcpp::Node::SharedPtr node);
        virtual ~BridgeCore();

        rclcpp::Node::SharedPtr nh;

        void start();
        virtual void stop();

    protected:
        bool loadParameters_();
        void calculateInterpolationParams_(float_t duration,
                                           float_t interpolation_order,
                                           bool hold_position = false);

        void robotCmdCallBack_(bridge_interface::msg::RobotCmd::SharedPtr message);

        // Each robot bridge but X2 (which makes its own three) calls this after its wait
        // loops: DDS stores a call made during the wait and runs it once spin starts.
        void createArmingServices_();

        void readyPositionControl_();
        void zeroPositionControl_();
        void update_();
        virtual void publishLowCommand_() = 0;
        virtual bool initControl_(bridge_interface::msg::RobotCmd default_cmd) = 0;
        bool checkCommand_(bridge_interface::msg::RobotCmd::SharedPtr robotCommand);

        // One motor's command, against one joint's limits. Rejects what cannot be
        // salvaged (NaN/Inf, absurd magnitudes) and clamps what can (kp, kd, dq,
        // tau), setting `clipped` when it did. Does not touch q: the body path
        // bounds q by implied torque in publishLowCommand_, the hand path clamps
        // it to [q_min, q_max] itself.
        bool checkMotorCmd_(bridge_interface::msg::MotorCmd &cmd,
                            const Joint &joint_info,
                            float_t kp_min, float_t kp_max,
                            float_t kd_min, float_t kd_max,
                            size_t index, bool &clipped);

        // Which fault the latest state shows, or None. Logs at most once a second.
        estop::Cause checkState_();
        // State loss check, called by checkState_ without mutex_; a robot with its own state streams overrides it.
        virtual bool checkStateFreshness_();
        virtual void finishControl_() = 0;

        // E-stop (docs/estop.md). Every stop goes through enterStop_, from any thread.
        bool loadEstopParameters_();
        void enterStop_(estop::Cause cause);
        // The only way the arming services start control: refused while latched, on a
        // checkState_ fault, and without the remote when estop.require_remote.
        bool armControl_(bridge_interface::msg::RobotCmd default_cmd, std::string &why);
        // False only when estop.require_remote and no remote header for REMOTE_TIMEOUT_S.
        bool remoteAllowsArming_(std::string &why);
        // Robot-specific arming refusals. Default: none.
        virtual bool robotAllowsArming_(std::string &why) { (void)why; return true; }
        // Called by update_() with mutex_ held, while stopped. Default: silent.
        virtual void publishStopCommand_() {}
        void syncLatchFlags_();   // mutex_ held
        std::string describeState_();
        // mutex_ held for every state change, so two threads cannot publish out of order.
        void publishBridgeState_(const std::string &state);


        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stopControlService_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr readyPositionService_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr zeroPositionService_;
        rclcpp::Service<bridge_interface::srv::SetDefaultPosition>::SharedPtr startControlService_;
        rclcpp::Subscription<bridge_interface::msg::RobotCmd>::SharedPtr desiredSubscriber_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr estopDampService_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr estopHoldService_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr estopResetService_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr bridgeStatePublisher_;

        std::vector<Joint> joints_;
        std::vector<CmdParams> cmdParams_;
        std::vector<double> ready_q_;
        float_t cmdInterpOrder_{0.0};

        bridge_interface::msg::RobotCmd lowCommandDesired_;
        bridge_interface::msg::LowCmd lastCommand_;
        bridge_interface::msg::LowState currentState_;
        bridge_interface::msg::ImuState imu_;

        rclcpp::Time last_state_time_;

        std::thread controlThread_;
        std::mutex mutex_;
        double tStart_, tFinal_, tValid_;
        const float_t INF_{std::numeric_limits<float>::infinity()};

        int numJoint_{0};
        int emptyJointIndex_{0};
        float_t duration_{1.0};
        float_t controlDt_{0.002};

        float_t upper_limbs_kp_min_{0}, upper_limbs_kp_max_{0};
        float_t upper_limbs_kd_min_{0}, upper_limbs_kd_max_{0};
        float_t lower_limbs_kp_min_{0}, lower_limbs_kp_max_{0};
        float_t lower_limbs_kd_min_{0}, lower_limbs_kd_max_{0};

        bool torqueControl_{false};
        std::atomic<bool> controlStarted_{false};   // tracking /robot_cmd
        bool receivedCmd_{false}; // Whether the control is initialized
        float_t imu_rpy_threshold_{1.0}; // Threshold for IMU roll/pitch in radians

        // E-stop. latch_ and holdQ_ are guarded by mutex_; the atomics mirror
        // latch_ for readers that must not lock.
        estop::Config estopConfig_;
        estop::SafetyLatch latch_;
        std::vector<float_t> holdQ_;
        std::atomic<bool> latched_{false};
        std::atomic<bool> stopPublishing_{false};
        std::atomic<uint16_t> remoteKeys_{0};
        std::atomic<double> lastRemoteHeader_{-1e9};   // clock seconds; set by the robot's state handler
        estop::Cause lastFault_{estop::Cause::None};   // control thread only
        bool invalidFrame_{false};   // control thread only: publishLowCommand_ sent nothing, a field not finite


        virtual void stopControlServiceCB_(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                   std::shared_ptr<std_srvs::srv::Trigger::Response> response);

        void readyPositionControlServiceCB_(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                            std::shared_ptr<std_srvs::srv::Trigger::Response> response);

        void zeroPositionControlServiceCB_(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                           std::shared_ptr<std_srvs::srv::Trigger::Response> response);

        void startControlServiceCB_(const std::shared_ptr<bridge_interface::srv::SetDefaultPosition::Request> request,
                                          std::shared_ptr<bridge_interface::srv::SetDefaultPosition::Response> response);

        void estopDampServiceCB_(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                 std::shared_ptr<std_srvs::srv::Trigger::Response> response);

        void estopHoldServiceCB_(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                 std::shared_ptr<std_srvs::srv::Trigger::Response> response);

        void estopResetServiceCB_(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                  std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    };

} // namespace sairol_bridge