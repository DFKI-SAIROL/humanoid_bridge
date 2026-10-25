// Unit tests for booster_mode.hpp. No ROS, no SDK. Run: build/robot_bridge/test_booster
#include "booster_mode.hpp"

#include <gtest/gtest.h>

using namespace sairol_bridge::booster;

// The values of DFKI-SAIROL/booster_robotics_sdk@88ce1691 (tools/check-booster-sdk.py reads them).
TEST(BoosterMode, ValuesAsTheSdk)
{
    EXPECT_EQ(static_cast<int>(RobotMode::Damping), 0);
    EXPECT_EQ(static_cast<int>(RobotMode::Prepare), 1);
    EXPECT_EQ(static_cast<int>(RobotMode::Walking), 2);
    EXPECT_EQ(static_cast<int>(RobotMode::Custom), 3);
    EXPECT_EQ(API_CHANGE_MODE, 2000);
}

TEST(BoosterMode, BodyAsNlohmannDumpsIt)
{
    EXPECT_EQ(changeModeBody(RobotMode::Custom), "{\"mode\":3}");
    EXPECT_EQ(changeModeBody(RobotMode::Damping), "{\"mode\":0}");
    EXPECT_STREQ(name(RobotMode::Custom), "custom");
}
