#include "G1_bridge.hpp"
// #include "unitree_hg/message_utils.hpp"

using namespace std::chrono_literals;

sairol_bridge::G1Bridge::G1Bridge(rclcpp::Node::SharedPtr node) : BridgeCore(node)
{

    lowCommandDesired_.motor_cmd.resize(numJoint_);
    lastCommand_.motor_cmd.resize(numJoint_);
    currentState_.motor_state.resize(numJoint_);
    cmdParams_.resize(numJoint_);

    lowStateSubscriber_ = nh->create_subscription<unitree_hg::msg::LowState>(
        "/lowstate", 1, std::bind(&sairol_bridge::G1Bridge::lowStateHandler_, this, std::placeholders::_1));

    // remoteControlSubscriber_ = nh->create_subscription<unitree_go::msg::WirelessController>(
    //     "/wirelesscontroller", 10, std::bind(&sairol_bridge::G1Bridge::wireless_callback, this, std::placeholders::_1));

    while (!checkExternalPublisher_("/lowcmd"))
    {
        rclcpp::sleep_for(std::chrono::milliseconds(1000));
    }

    lowCommandPublisher_ = nh->create_publisher<unitree_hg::msg::LowCmd>("/lowcmd", 10); // /joint_ctrl

    // Waiting for publisher on topic lowstate
    RCLCPP_INFO(nh->get_logger(), "Waiting for publisher on topic /lowstate...");

    while (nh->count_publishers("/lowstate") == 0)
    {
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
}


bool sairol_bridge::G1Bridge::checkExternalPublisher_(std::string topic_name)
{
    auto publishers_info = nh->get_publishers_info_by_topic(topic_name);
    int publisher_count = publishers_info.size();
    if (publisher_count > 0)
    {
        RCLCPP_ERROR_STREAM(nh->get_logger(),
                     "Detected " << publisher_count << " publishers on " << topic_name.c_str() << 
                     ". Please change to the debug mode by pressing L2 + R2");
        return false;
    }
    return true;
}


void sairol_bridge::G1Bridge::stop()
{
    // Stop the control thread if it's running
    if (controlThread_.joinable())
    {
        controlThread_.join();
    }

    RCLCPP_INFO(nh->get_logger(), "G1 Bridge stopped.");
}

void sairol_bridge::G1Bridge::wireless_callback(unitree_go::msg::WirelessController::SharedPtr data)
{
    uint32_t key = data->keys;

    if ((key & (KEY_L2 | KEY_START)) == (KEY_L2 | KEY_START))  // L2 + START : start control
    {
        RCLCPP_INFO(nh->get_logger(), "Starting control...");
        initControl_(bridge_interface::msg::RobotCmd());
        return;
    }
    else if ((key & (KEY_L2 | KEY_UP | KEY_LEFT)) == (KEY_L2 | KEY_UP | KEY_LEFT))  // L2 + UP + LEFT : stop control
    {
        RCLCPP_INFO(nh->get_logger(), "Stopping control...");
        if (controlStarted_) controlStarted_ = false;
        return;
    }
    else if (key & KEY_L1)  //L1 : Ready position
    {
        RCLCPP_INFO(nh->get_logger(), "Ready position control...");
        if (!controlStarted_) initControl_(bridge_interface::msg::RobotCmd());
        readyPositionControl_();
        calculateInterpolationParams_(duration_, 1, true);
        return;
    }
    else if (key & KEY_R1)  // R1 : Zero position
    {
        RCLCPP_INFO(nh->get_logger(), "Zero position control...");
        if (!controlStarted_) initControl_(bridge_interface::msg::RobotCmd());
        zeroPositionControl_();
        calculateInterpolationParams_(duration_, 1, true);
        return;
    }

}

void sairol_bridge::G1Bridge::lowStateHandler_(unitree_hg::msg::LowState::SharedPtr msg)
{
        // update mode machine
    if (mode_machine_ != msg->mode_machine) {
      if (mode_machine_ == 0) {
        RCLCPP_INFO(nh->get_logger(), "G1 type: %d", unsigned(msg->mode_machine));
      }
      mode_machine_ = msg->mode_machine;
    }

    if (std::abs(msg->imu_state.rpy[0]) > imu_rpy_threshold_ || std::abs(msg->imu_state.rpy[1]) > imu_rpy_threshold_)
    {
        RCLCPP_WARN(nh->get_logger(), "IMU base rpy values are too large: [%f, %f]", msg->imu_state.rpy[0], msg->imu_state.rpy[1]);
        controlStarted_ = false;
    }
    // Update the last state time using the same clock source
    last_state_time_ = nh->get_clock()->now();
    const auto motor_state_count = msg->motor_state.size();
    const auto expected_motor_state_count = static_cast<size_t>(numJoint_);
    const auto copy_count = std::min(motor_state_count, expected_motor_state_count);

    if (motor_state_count != expected_motor_state_count)
    {
        RCLCPP_WARN_ONCE(
            nh->get_logger(),
            "LowState carries %zu motor states, but robot_bridge is configured for %zu joints. "
            "Only the first %zu entries will be used.",
            motor_state_count,
            expected_motor_state_count,
            copy_count);
    }

    for (size_t i = 0; i < copy_count; ++i)
    {
        currentState_.motor_state[i].q = msg->motor_state[i].q;
        currentState_.motor_state[i].dq = msg->motor_state[i].dq;
        currentState_.motor_state[i].ddq = msg->motor_state[i].ddq;
        currentState_.motor_state[i].tau_est = msg->motor_state[i].tau_est;
    }

    imu_.rpy = msg->imu_state.rpy;
    imu_.gyroscope = msg->imu_state.gyroscope;
    imu_.accelerometer = msg->imu_state.accelerometer;
    imu_.quaternion = msg->imu_state.quaternion;
}

void sairol_bridge::G1Bridge::publishLowCommand_()
{
    rclcpp::Time current = nh->get_clock()->now();

    float_t phase = 1.0f;
    if (tFinal_ - tStart_ > 1e-6) {
        phase = (current.seconds() - tStart_) / (tFinal_ - tStart_);
    }

    phase = std::clamp(phase, 0.0f, 1.0f); // Ensure phase is between 0 and 1

    unitree_hg::msg::LowCmd unitree_cmd;
    unitree_cmd.mode_machine = mode_machine_;
    unitree_cmd.mode_pr = PRorAB::PR;

    for (int i = 0; i < numJoint_; ++i)
    {
        auto &cmd = unitree_cmd.motor_cmd[i];
        auto &last_cmd = lastCommand_.motor_cmd[i];
        auto &joint_info = joints_[i];

        unitree_cmd.motor_cmd[i].mode = 1; // Enable motor

        if (1e-6 < cmdInterpOrder_ && cmdInterpOrder_ < 1.0 - 1e-6)
        {
            // Low pass filter for cmd
            cmd.q = last_cmd.q * cmdInterpOrder_ + cmdParams_[i].q_0 * (1 - cmdInterpOrder_);
            cmd.dq = last_cmd.dq * cmdInterpOrder_ + cmdParams_[i].dq_0 * (1 - cmdInterpOrder_);
        }
        else {
            // Interpolation
            cmd.q = cmdParams_[i].q_0 + cmdParams_[i].q_1 * phase;
            cmd.dq = cmdParams_[i].dq_0 + cmdParams_[i].dq_1 * phase;
        }

        cmd.kp = cmdParams_[i].kp_0 + cmdParams_[i].kp_1 * phase;
        cmd.kd = cmdParams_[i].kd_0 + cmdParams_[i].kd_1 * phase;
        cmd.dq = std::clamp(cmd.dq, -joint_info.dq_limit, joint_info.dq_limit);

        if (torqueControl_)
        {
            cmd.tau = cmd.kp * (cmd.q - currentState_.motor_state[i].q) + cmd.kd * (cmd.dq - currentState_.motor_state[i].dq) + cmdParams_[i].tau_0;
            cmd.tau = std::clamp(cmd.tau, -joint_info.tau_limit, joint_info.tau_limit);
            cmd.kp = 0.0;
            cmd.kd = 0.0;
        }
        else
        {
            cmd.tau = cmdParams_[i].tau_0 + cmdParams_[i].tau_1 * phase;
            // RCLCPP_WARN(nh->get_logger(), "Torque control is disabled for joint %d, but tau command is non-zero. Check if this is intended!", i);
            // RCLCPP_WARN(nh->get_logger(), "tau command for joint %d: %f", i, cmd.tau);
        }  
        // cmd.q = std::clamp(cmd.q, 
        //     (-cmd.kd * (currentState_.motor_state[i].dq - cmd.dq) - joint_info.tau_limit) / cmd.kp + currentState_.motor_state[i].q, 
        //     (-cmd.kd * (currentState_.motor_state[i].dq - cmd.dq) + joint_info.tau_limit) / cmd.kp + currentState_.motor_state[i].q);

        last_cmd.q = cmd.q;
        last_cmd.dq = cmd.dq;
        last_cmd.kp = cmd.kp;
        last_cmd.kd = cmd.kd;
        last_cmd.tau = cmd.tau;     
        
        if (receivedCmd_ && joints_[i].if_parallel_joint)
        {
            RCLCPP_WARN(nh->get_logger(), "DID PARALLEL JOINT CONTROL FOR JOINT %d, CHECK IF THIS IS INTENDED!", i);

            cmd.tau = std::clamp((last_cmd.q - currentState_.motor_state[i].q) * cmd.kp, -joint_info.tau_limit, joint_info.tau_limit);
            cmd.q = currentState_.motor_state[i].q;
            cmd.kp = 0.0;
        }   
        
    }
    get_crc(unitree_cmd);
    lowCommandPublisher_->publish(unitree_cmd);
}

bool sairol_bridge::G1Bridge::initControl_(bridge_interface::msg::RobotCmd default_cmd)
{
    last_state_time_ = nh->get_clock()->now();

    for (int i = 0; i < numJoint_; ++i)
    {
        lowCommandDesired_.motor_cmd[i].q = currentState_.motor_state[i].q;
        lowCommandDesired_.motor_cmd[i].kp = joints_[i].kp;
        lowCommandDesired_.motor_cmd[i].kd = joints_[i].kd;
    }

    calculateInterpolationParams_(0.0, 1, true);

    controlStarted_ = true;
    receivedCmd_ = true;
    rclcpp::Rate rate(100);
    rate.sleep();
    RCLCPP_INFO(nh->get_logger(), "Control initialized successfully.");
    
    if (default_cmd.motor_cmd.size() == numJoint_)
    {
        for (size_t i = 0; i < numJoint_; ++i)
        {
            lowCommandDesired_.motor_cmd[i].q = default_cmd.motor_cmd[i].q;
            lowCommandDesired_.motor_cmd[i].kp = default_cmd.motor_cmd[i].kp;
            lowCommandDesired_.motor_cmd[i].kd = default_cmd.motor_cmd[i].kd;
        }
    }
    else 
    {
        for (size_t i = 0; i < numJoint_; ++i)
        {
            lowCommandDesired_.motor_cmd[i].q = ready_q_[i];
            lowCommandDesired_.motor_cmd[i].kp = joints_[i].kp;
            lowCommandDesired_.motor_cmd[i].kd = joints_[i].kd;
        }
    }

    calculateInterpolationParams_(duration_, 1, true);

    return true;
}

void sairol_bridge::G1Bridge::finishControl_() {
    RCLCPP_INFO(nh->get_logger(), "finishControl_ called from G1Bridge");
}



