#include <gtest/gtest.h>

#include "oxebots_comms/channel_migration.hpp"

using oxebots_comms::ChannelMigrationMachine;

TEST(ChannelMigrationTest, PartialConfirmationDoesNotPauseConfirmedRobot)
{
    ChannelMigrationMachine mach;
    mach.start(76, 90, 104, 118, {0, 1}, 15, 40, true);
    ASSERT_TRUE(mach.is_active());
    EXPECT_TRUE(mach.is_robot_pending(0));
    EXPECT_TRUE(mach.is_robot_pending(1));

    // Tick 0: both robots get SET_CHANNEL on old_tx=76 + VerifyOnNew on 104
    const auto a0 = mach.tick(0);
    EXPECT_EQ(a0.size(), 4U);

    // Robot 0 confirms immediately on attempt 1 -> no longer pending!
    mach.on_new_channel_ack(0);
    EXPECT_FALSE(mach.is_robot_pending(0));
    EXPECT_TRUE(mach.is_robot_confirmed(0));
    EXPECT_TRUE(mach.is_robot_pending(1));
    EXPECT_TRUE(mach.is_active());

    // Subsequent ticks only retry Robot 1 while Robot 0 keeps receiving strategy on ch 104.
    for (int attempt = 2; attempt <= 5; ++attempt) {
        const auto actions = mach.tick(attempt * 50);
        for (const auto& act : actions) {
            EXPECT_NE(act.robot_id, 0U);
        }
    }

    // Robot 1 confirms on attempt 6 -> migration completes with both confirmed!
    const auto a6 = mach.tick(300);
    EXPECT_FALSE(a6.empty());
    mach.on_new_channel_ack(1);
    EXPECT_FALSE(mach.is_active());

    const auto sum = mach.summary();
    EXPECT_TRUE(sum.finished);
    EXPECT_FALSE(sum.rollback_required);
    EXPECT_TRUE(sum.persist_nvs);
    EXPECT_EQ(sum.final_tx_channel, 104U);
    EXPECT_EQ(sum.final_rx_channel, 118U);
    EXPECT_EQ(sum.confirmed_ids.size(), 2U);
    EXPECT_TRUE(sum.unconfirmed_ids.empty());
}

TEST(ChannelMigrationTest, OfflineRobotStopsAtMaxAttemptsAndZeroConfirmedTriggersRollback)
{
    ChannelMigrationMachine mach;
    mach.start(76, 90, 104, 118, {2}, 15, 40, true);

    int send_old_count = 0;
    for (int step = 0; step < 30 && mach.is_active(); ++step) {
        const auto actions = mach.tick(step * 40);
        for (const auto& act : actions) {
            if (act.type == ChannelMigrationMachine::ActionType::SendSetChannelOnOld) {
                ++send_old_count;
            }
        }
    }
    EXPECT_FALSE(mach.is_active());
    EXPECT_EQ(send_old_count, 15);

    const auto sum = mach.summary();
    EXPECT_TRUE(sum.finished);
    EXPECT_TRUE(sum.rollback_required);
    EXPECT_EQ(sum.final_tx_channel, 76U);
    EXPECT_EQ(sum.final_rx_channel, 90U);
    EXPECT_TRUE(sum.confirmed_ids.empty());
    ASSERT_EQ(sum.unconfirmed_ids.size(), 1U);
    EXPECT_EQ(sum.unconfirmed_ids[0], 2U);
}

TEST(ChannelMigrationTest, RxOnlyMigrationRequiresTelemetryConfirmation)
{
    ChannelMigrationMachine mach;
    mach.start(104, 114, 104, 118, {1}, 15, 40, true);
    ASSERT_TRUE(mach.requires_telemetry_confirm());

    const auto actions = mach.tick(0);
    ASSERT_GE(actions.size(), 2U);
    EXPECT_EQ(actions[1].type, ChannelMigrationMachine::ActionType::VerifyTelemetryOnNew);

    // Downlink Auto-ACK alone must NOT confirm when tx_ch did not change!
    mach.on_new_channel_ack(1);
    EXPECT_TRUE(mach.is_active());
    EXPECT_TRUE(mach.is_robot_pending(1));

    // Receiving RobotTelemetry on new rx_ch=118 confirms migration!
    mach.on_new_channel_telemetry(1);
    EXPECT_FALSE(mach.is_active());
    EXPECT_TRUE(mach.is_robot_confirmed(1));
    EXPECT_FALSE(mach.summary().rollback_required);
}
