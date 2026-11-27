#include <gtest/gtest.h>

#include <array>

#include "oxebots_comms/channel_policy.hpp"

using oxebots_comms::ChannelPolicyConfig;
using oxebots_comms::ChannelPolicyEngine;
using oxebots_comms::LinkHealthSample;
using oxebots_comms::LossTriggerKind;

TEST(ChannelPolicyTest, SelectsOptimalDistanceCleanPairAboveWifi)
{
    std::array<uint8_t, 126> busy{};
    for (int ch = 0; ch <= 64; ++ch) busy[ch] = 70;
    busy[101] = 38;
    busy[102] = 3;

    const auto pair =
        oxebots_comms::select_best_channel_pair(busy.data(), 0, 126, true, 10, 14, 0xFF, 0xFF);
    EXPECT_GE(pair.tx_channel, 100U);
    EXPECT_LE(pair.tx_channel, 124U);
    EXPECT_GE(pair.rx_channel, 100U);
    EXPECT_LE(pair.rx_channel, 124U);
    EXPECT_EQ(pair.separation_mhz, 14U);
    EXPECT_EQ(pair.tx_busy_pct, 0U);
    EXPECT_EQ(pair.rx_busy_pct, 0U);
}

TEST(ChannelPolicyTest, ReactiveTxAndRxLossExcludesFailingChannelAndRespectsCooldown)
{
    ChannelPolicyConfig cfg{};
    cfg.loss_confirm_windows = 3;
    cfg.min_window_tx = 20;
    cfg.migration_cooldown_s = 20.0;
    ChannelPolicyEngine engine(cfg);

    LinkHealthSample idle{};
    idle.window_tx_count = 5;
    idle.overall_ack_rate_pct = 0.0;
    EXPECT_EQ(engine.evaluate_link_health(idle, 1.0), LossTriggerKind::None);

    LinkHealthSample bad_tx{};
    bad_tx.active_tx_channel = 104;
    bad_tx.active_rx_channel = 118;
    bad_tx.window_tx_count = 40;
    bad_tx.overall_ack_rate_pct = 65.0;
    EXPECT_EQ(engine.evaluate_link_health(bad_tx, 2.0), LossTriggerKind::None);
    EXPECT_EQ(engine.evaluate_link_health(bad_tx, 2.5), LossTriggerKind::None);
    EXPECT_EQ(engine.evaluate_link_health(bad_tx, 3.0), LossTriggerKind::DownlinkTxLoss);

    // Even if RPD at base shows 0% on ch 104, DownlinkTxLoss must exclude ch 104!
    std::array<uint8_t, 126> clean{};
    const auto decision =
        engine.decide_from_scan(clean, 104, 118, true, false, LossTriggerKind::DownlinkTxLoss);
    EXPECT_TRUE(decision.should_migrate);
    EXPECT_NE(decision.target.tx_channel, 104U);
    EXPECT_GE(decision.target.separation_mhz, 10U);

    // Cooldown suppresses immediate re-trigger
    engine.note_migration_started(3.0);
    EXPECT_EQ(engine.evaluate_link_health(bad_tx, 10.0), LossTriggerKind::None);

    // Scan in progress suppresses false loss
    LinkHealthSample during_scan = bad_tx;
    during_scan.scan_in_progress = true;
    EXPECT_EQ(engine.evaluate_link_health(during_scan, 30.0), LossTriggerKind::None);
}

TEST(ChannelPolicyTest, HysteresisPreventsFlappingOnSmallNoiseDiff)
{
    ChannelPolicyConfig cfg{};
    cfg.periodic_busy_threshold_pct = 25;
    cfg.min_busy_improvement_pct = 15;
    ChannelPolicyEngine engine(cfg);

    std::array<uint8_t, 126> busy{};
    busy.fill(10);
    busy[104] = 20;  // Below periodic threshold (25%)
    busy[118] = 10;

    const auto dec1 =
        engine.decide_from_scan(busy, 104, 118, true, false, LossTriggerKind::None);
    EXPECT_FALSE(dec1.should_migrate);

    // Now ch 104 spikes to 45% while 108/122 are 0% -> should migrate!
    busy[104] = 45;
    busy[108] = 0;
    busy[122] = 0;
    const auto dec2 =
        engine.decide_from_scan(busy, 104, 118, true, false, LossTriggerKind::None);
    EXPECT_TRUE(dec2.should_migrate);
    EXPECT_NE(dec2.target.tx_channel, 104U);
}
