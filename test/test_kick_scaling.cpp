#include <gtest/gtest.h>

#include <limits>

#include "oxebots_comms/kick_scaling.hpp"

namespace
{

uint8_t firmware_team_pct_to_u8(uint8_t pct)
{
    const uint8_t clamped = (pct > 100U) ? 100U : pct;
    return (clamped > 0U)
               ? static_cast<uint8_t>((static_cast<uint16_t>(clamped) * 255U + 50U) / 100U)
               : 0U;
}

uint8_t firmware_u8_to_kicker_duty(uint8_t velocity)
{
    if (velocity == 0U) {
        return 0U;
    }
    const uint8_t duty =
        static_cast<uint8_t>((static_cast<uint16_t>(velocity) * 100U + 127U) / 255U);
    return (duty < 1U) ? 1U : duty;
}

}  // namespace

TEST(KickScalingTest, ZeroNegativeAndNonFiniteMapToZero)
{
    const double qnan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();

    for (double bad : {0.0, -0.0, -0.5, -10.0, qnan, inf, -inf}) {
        EXPECT_EQ(oxebots_comms::kick_speed_to_team_percent(bad), 0U);
        EXPECT_EQ(oxebots_comms::kick_speed_to_u8(bad), 0U);
    }
}

TEST(KickScalingTest, LinearMappingAndClamping)
{
    EXPECT_EQ(oxebots_comms::kick_speed_to_team_percent(0.01), 1U);
    EXPECT_EQ(oxebots_comms::kick_speed_to_u8(0.01), 1U);

    EXPECT_EQ(oxebots_comms::kick_speed_to_team_percent(1.5), 50U);
    EXPECT_EQ(oxebots_comms::kick_speed_to_u8(1.5), 128U);

    EXPECT_EQ(oxebots_comms::kick_speed_to_team_percent(3.0), 100U);
    EXPECT_EQ(oxebots_comms::kick_speed_to_u8(3.0), 255U);

    EXPECT_EQ(oxebots_comms::kick_speed_to_team_percent(5.0), 100U);
    EXPECT_EQ(oxebots_comms::kick_speed_to_u8(5.0), 255U);
}

TEST(KickScalingTest, CustomAndInvalidMaxKickSpeed)
{
    EXPECT_EQ(oxebots_comms::kick_speed_to_team_percent(3.0, 6.0), 50U);
    EXPECT_EQ(oxebots_comms::kick_speed_to_u8(3.0, 6.0), 128U);

    const double qnan = std::numeric_limits<double>::quiet_NaN();
    for (double bad_max : {0.0, -2.0, qnan}) {
        EXPECT_EQ(oxebots_comms::kick_speed_to_team_percent(3.0, bad_max), 100U);
        EXPECT_EQ(oxebots_comms::kick_speed_to_u8(3.0, bad_max), 255U);
    }
}

TEST(KickScalingTest, EndToEndMatchesFirmwareKickerDuty)
{
    const uint8_t pct_half = oxebots_comms::kick_speed_to_team_percent(1.5);
    EXPECT_EQ(firmware_u8_to_kicker_duty(firmware_team_pct_to_u8(pct_half)), 50U);

    const uint8_t pct_full = oxebots_comms::kick_speed_to_team_percent(3.0);
    EXPECT_EQ(firmware_u8_to_kicker_duty(firmware_team_pct_to_u8(pct_full)), 100U);

    const uint8_t u8_full = oxebots_comms::kick_speed_to_u8(3.0);
    EXPECT_EQ(firmware_u8_to_kicker_duty(u8_full), 100U);
}
