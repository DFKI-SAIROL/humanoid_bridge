// booster_mode.hpp -------------------------------------------------------------
// The Booster SDK's mode switch, without the SDK: the values T1_bridge sends on booster_rpc_service.
// From DFKI-SAIROL/booster_robotics_sdk@88ce1691: include/booster/robot/common/robot_shared.hpp
// (RobotMode), include/booster/robot/b1/b1_loco_api.hpp (LocoApiId, ChangeModeParameter::ToJson).
// tools/check-booster-sdk.py checks them against an SDK clone.
#pragma once

#include <cstdint>
#include <string>

namespace sairol_bridge::booster
{
    enum class RobotMode : int
    {
        Unknown = -1,
        Damping = 0,   // every motor damps: a standing robot falls
        Prepare = 1,   // stands on both feet; can switch to walking
        Walking = 2,
        Custom = 3,    // the robot follows /joint_ctrl
        Soccer = 4,
    };

    constexpr int64_t API_CHANGE_MODE = 2000;   // LocoApiId::kChangeMode

    // ChangeModeParameter's body, as nlohmann::json dumps it.
    inline std::string changeModeBody(RobotMode mode)
    {
        return "{\"mode\":" + std::to_string(static_cast<int>(mode)) + "}";
    }

    inline const char *name(RobotMode mode)
    {
        switch (mode)
        {
        case RobotMode::Damping: return "damping";
        case RobotMode::Prepare: return "prepare";
        case RobotMode::Walking: return "walking";
        case RobotMode::Custom: return "custom";
        case RobotMode::Soccer: return "soccer";
        default: return "unknown";
        }
    }
} // namespace sairol_bridge::booster
