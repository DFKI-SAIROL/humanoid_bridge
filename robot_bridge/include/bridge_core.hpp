// bridge_core.hpp ------------------------------------------------------------
#pragma once

#include "rclcpp/rclcpp.hpp"
#include "bridge_interface/msg/robot_cmd.hpp"
#include "bridge_interface/msg/low_cmd.hpp"
#include "bridge_interface/msg/low_state.hpp"
#include "bridge_interface/msg/imu_state.hpp"
#include "bridge_interface/srv/set_default_position.hpp"
#include "std_srvs/srv/trigger.hpp"

#include <vector>
#include <limits>
#include <mutex>
#include <thread>
#include <cmath>
#include <algorithm>
#include <cassert>

namespace sairol_bridge
{ 
    typedef float float_t;

    struct Joint
    {
        int idx;
        float_t q_min{0};
        float_t q_max{0};
        float_t dq_limit{0};
        float_t tau_limit{0};
        float_t kp{0};
        float_t kd{0};
        bool if_strong_joint{false}; // Whether the joint is a strong joint
        bool if_parallel_joint{false}; // Whether the joint is a parallel joint
    };

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

        void readyPositionControl_();
        void zeroPositionControl_();
        void update_();
        virtual void publishLowCommand_() = 0;
        virtual bool initControl_(bridge_interface::msg::RobotCmd default_cmd) = 0;
        bool checkCommand_(bridge_interface::msg::RobotCmd::SharedPtr robotCommand);
        bool checkState_();
        virtual bool checkStateFreshness_();
        virtual void finishControl_() = 0;


        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stopControlService_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr readyPositionService_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr zeroPositionService_;
        rclcpp::Service<bridge_interface::srv::SetDefaultPosition>::SharedPtr startControlService_;
        rclcpp::Subscription<bridge_interface::msg::RobotCmd>::SharedPtr desiredSubscriber_;

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
        bool controlStarted_{false};
        bool receivedCmd_{false}; // Whether the control is initialized
        float_t imu_rpy_threshold_{1.0}; // Threshold for IMU roll/pitch in radians


        virtual void stopControlServiceCB_(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                   std::shared_ptr<std_srvs::srv::Trigger::Response> response);

        void readyPositionControlServiceCB_(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                            std::shared_ptr<std_srvs::srv::Trigger::Response> response);

        void zeroPositionControlServiceCB_(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                           std::shared_ptr<std_srvs::srv::Trigger::Response> response);

        void startControlServiceCB_(const std::shared_ptr<bridge_interface::srv::SetDefaultPosition::Request> request,
                                          std::shared_ptr<bridge_interface::srv::SetDefaultPosition::Response> response);
    };

} // namespace sairol_bridge