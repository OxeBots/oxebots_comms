#pragma once

#include <algorithm>
#include <array>
#include <cstdint>

namespace oxebots_comms
{

struct ChannelPairChoice
{
    uint8_t tx_channel = 76;
    uint8_t rx_channel = 90;
    uint8_t separation_mhz = 14;
    uint8_t tx_busy_pct = 0;
    uint8_t rx_busy_pct = 0;
    uint32_t score = 0;
};

struct ChannelPolicyConfig
{
    uint8_t min_separation_mhz = 10;
    uint8_t optimal_separation_mhz = 14;
    int startup_busy_threshold_pct = 15;
    int periodic_busy_threshold_pct = 25;
    int min_busy_improvement_pct = 15;
    double loss_ack_threshold_pct = 85.0;
    double loss_retry_threshold = 1.5;
    double loss_tel_rx_threshold_pct = 75.0;
    int loss_confirm_windows = 3;
    uint32_t min_window_tx = 20;
    uint32_t min_window_tel_req = 6;
    double migration_cooldown_s = 20.0;
};

struct LinkHealthSample
{
    uint8_t active_tx_channel = 76;
    uint8_t active_rx_channel = 90;
    uint8_t radio_ready_mask = 3;
    bool scan_in_progress = false;
    bool migration_in_progress = false;
    double overall_ack_rate_pct = 100.0;
    double overall_avg_retries = 0.0;
    double overall_tel_rx_rate_pct = 100.0;
    uint32_t window_tx_count = 0;
    uint32_t window_tel_req_count = 0;
};

enum class LossTriggerKind : uint8_t
{
    None = 0,
    DownlinkTxLoss = 1,
    UplinkRxLoss = 2,
    Both = 3,
};

inline uint32_t channel_noise_cost(
    const uint8_t* busy_pct, uint8_t start_channel, uint8_t count, uint8_t idx, bool wide_band)
{
    const uint16_t ch = static_cast<uint16_t>(start_channel) + idx;
    uint32_t score = static_cast<uint32_t>(busy_pct[idx]) * 100U;
    if (wide_band) {
        if (idx > 0) score += static_cast<uint32_t>(busy_pct[idx - 1]) * 35U;
        if (idx + 1 < count) score += static_cast<uint32_t>(busy_pct[idx + 1]) * 35U;
        if (idx > 1) score += static_cast<uint32_t>(busy_pct[idx - 2]) * 10U;
        if (idx + 2 < count) score += static_cast<uint32_t>(busy_pct[idx + 2]) * 10U;
    } else {
        if (idx > 0) score += static_cast<uint32_t>(busy_pct[idx - 1]) * 10U;
        if (idx + 1 < count) score += static_cast<uint32_t>(busy_pct[idx + 1]) * 10U;
    }
    if (ch < 85) {
        score += 20U;
    } else if (ch < 100 || ch > 124) {
        score += 5U;
    }
    return score;
}

inline ChannelPairChoice select_best_channel_pair(
    const uint8_t* busy_pct,
    uint8_t start_channel,
    uint8_t count,
    bool dual_radio,
    uint8_t min_sep = 10,
    uint8_t optimal_sep = 14,
    uint8_t exclude_tx = 0xFF,
    uint8_t exclude_rx = 0xFF,
    bool wide_band = true)
{
    ChannelPairChoice out{};
    if (busy_pct == nullptr || count == 0) return out;

    const uint8_t valid_count =
        (static_cast<uint16_t>(start_channel) + count > 126U)
            ? static_cast<uint8_t>(126U - start_channel)
            : count;
    if (min_sep == 0) min_sep = 10;
    if (optimal_sep < min_sep) optimal_sep = min_sep;

    std::array<uint32_t, 126> costs{};
    std::array<bool, 126> eligible{};
    for (uint8_t i = 0; i < valid_count; ++i) {
        if (wide_band && valid_count >= 3 && (i == 0 || i + 1 == valid_count)) continue;
        costs[i] = channel_noise_cost(busy_pct, start_channel, valid_count, i, wide_band);
        eligible[i] = true;
    }

    if (!dual_radio) {
        auto search_single = [&](bool apply_excl) -> bool {
            bool found = false;
            uint32_t best_score = 0xFFFFFFFFUL;
            for (uint8_t i = 0; i < valid_count; ++i) {
                if (!eligible[i]) continue;
                const uint8_t ch = static_cast<uint8_t>(start_channel + i);
                if (apply_excl && ch == exclude_tx) continue;
                if (!found || costs[i] < best_score) {
                    found = true;
                    best_score = costs[i];
                    out.tx_channel = ch;
                    out.rx_channel = ch;
                    out.separation_mhz = 0;
                    out.tx_busy_pct = busy_pct[i];
                    out.rx_busy_pct = busy_pct[i];
                    out.score = best_score;
                }
            }
            return found;
        };
        if (!search_single(true)) {
            search_single(false);
        }
        return out;
    }

    auto search_pair = [&](uint8_t req_sep, bool apply_excl) -> bool {
        bool found = false;
        uint32_t best_score = 0xFFFFFFFFUL;
        for (uint8_t i = 0; i < valid_count; ++i) {
            if (!eligible[i]) continue;
            const uint8_t tx_ch = static_cast<uint8_t>(start_channel + i);
            if (apply_excl && tx_ch == exclude_tx) continue;
            for (uint8_t j = 0; j < valid_count; ++j) {
                if (!eligible[j] || i == j) continue;
                const uint8_t rx_ch = static_cast<uint8_t>(start_channel + j);
                if (apply_excl && rx_ch == exclude_rx) continue;

                const uint8_t sep = (rx_ch > tx_ch) ? static_cast<uint8_t>(rx_ch - tx_ch)
                                                    : static_cast<uint8_t>(tx_ch - rx_ch);
                if (sep < req_sep) continue;

                const uint32_t sep_dev = (sep > optimal_sep)
                                             ? static_cast<uint32_t>(sep - optimal_sep)
                                             : static_cast<uint32_t>(optimal_sep - sep);
                const uint32_t pair_score =
                    (costs[i] * 12U) / 10U + costs[j] + sep_dev * 8U + ((tx_ch > rx_ch) ? 1U : 0U);
                if (!found || pair_score < best_score) {
                    found = true;
                    best_score = pair_score;
                    out.tx_channel = tx_ch;
                    out.rx_channel = rx_ch;
                    out.separation_mhz = sep;
                    out.tx_busy_pct = busy_pct[i];
                    out.rx_busy_pct = busy_pct[j];
                    out.score = pair_score;
                }
            }
        }
        return found;
    };

    if (search_pair(min_sep, true)) return out;
    if ((exclude_tx != 0xFF || exclude_rx != 0xFF) && search_pair(min_sep, false)) return out;
    if (search_pair(1, false)) return out;
    return select_best_channel_pair(
        busy_pct, start_channel, valid_count, false, min_sep, optimal_sep, exclude_tx, exclude_rx,
        wide_band);
}

class ChannelPolicyEngine
{
public:
    explicit ChannelPolicyEngine(const ChannelPolicyConfig& cfg = {}) : cfg_(cfg) {}

    void set_config(const ChannelPolicyConfig& cfg) { cfg_ = cfg; }
    const ChannelPolicyConfig& config() const { return cfg_; }

    LossTriggerKind evaluate_link_health(const LinkHealthSample& sample, double now_s)
    {
        if (sample.scan_in_progress || sample.migration_in_progress) {
            return LossTriggerKind::None;
        }
        if (last_migration_s_ >= 0.0 && (now_s - last_migration_s_) < cfg_.migration_cooldown_s) {
            consecutive_tx_loss_ = 0;
            consecutive_rx_loss_ = 0;
            return LossTriggerKind::None;
        }

        bool tx_degraded = false;
        if (sample.window_tx_count >= cfg_.min_window_tx) {
            tx_degraded = (sample.overall_ack_rate_pct < cfg_.loss_ack_threshold_pct) ||
                          (sample.overall_avg_retries > cfg_.loss_retry_threshold);
        }

        bool rx_degraded = false;
        if (sample.window_tel_req_count >= cfg_.min_window_tel_req && !tx_degraded) {
            // Only classify as pure uplink RX degradation when downlink TX itself is healthy.
            rx_degraded = (sample.overall_tel_rx_rate_pct < cfg_.loss_tel_rx_threshold_pct);
        }

        if (tx_degraded) {
            ++consecutive_tx_loss_;
        } else {
            consecutive_tx_loss_ = 0;
        }

        if (rx_degraded) {
            ++consecutive_rx_loss_;
        } else {
            consecutive_rx_loss_ = 0;
        }

        const bool fire_tx = (consecutive_tx_loss_ >= cfg_.loss_confirm_windows);
        const bool fire_rx = (consecutive_rx_loss_ >= cfg_.loss_confirm_windows);
        if (fire_tx && fire_rx) return LossTriggerKind::Both;
        if (fire_tx) return LossTriggerKind::DownlinkTxLoss;
        if (fire_rx) return LossTriggerKind::UplinkRxLoss;
        return LossTriggerKind::None;
    }

    void note_migration_started(double now_s)
    {
        last_migration_s_ = now_s;
        consecutive_tx_loss_ = 0;
        consecutive_rx_loss_ = 0;
    }

    bool in_cooldown(double now_s) const
    {
        return last_migration_s_ >= 0.0 && (now_s - last_migration_s_) < cfg_.migration_cooldown_s;
    }

    struct MigrationDecision
    {
        bool should_migrate = false;
        ChannelPairChoice target{};
    };

    MigrationDecision decide_from_scan(
        const std::array<uint8_t, 126>& busy_pct,
        uint8_t current_tx_ch,
        uint8_t current_rx_ch,
        bool dual_radio,
        bool is_startup,
        LossTriggerKind loss_trigger) const
    {
        MigrationDecision decision{};

        uint8_t excl_tx = 0xFF;
        uint8_t excl_rx = 0xFF;
        if (loss_trigger == LossTriggerKind::DownlinkTxLoss ||
            loss_trigger == LossTriggerKind::Both) {
            excl_tx = current_tx_ch;
        }
        if (loss_trigger == LossTriggerKind::UplinkRxLoss ||
            loss_trigger == LossTriggerKind::Both) {
            excl_rx = current_rx_ch;
        }

        const ChannelPairChoice candidate = select_best_channel_pair(
            busy_pct.data(),
            0,
            126,
            dual_radio,
            cfg_.min_separation_mhz,
            cfg_.optimal_separation_mhz,
            excl_tx,
            excl_rx,
            true);
        decision.target = candidate;

        if (candidate.tx_channel == current_tx_ch && candidate.rx_channel == current_rx_ch) {
            return decision;
        }

        // If triggered by real sustained packet loss, migrate immediately to escape the failing channel.
        if (loss_trigger != LossTriggerKind::None) {
            decision.should_migrate = true;
            return decision;
        }

        const uint8_t cur_tx_busy = (current_tx_ch < 126) ? busy_pct[current_tx_ch] : 100;
        const uint8_t cur_rx_busy = (current_rx_ch < 126) ? busy_pct[current_rx_ch] : 100;
        const int cur_worst = std::max(cur_tx_busy, cur_rx_busy);
        const int new_worst = std::max(candidate.tx_busy_pct, candidate.rx_busy_pct);

        // In Dual-Radio mode, also fix invalid separation (e.g., tx == rx after boot from single-radio).
        const uint8_t cur_sep = (current_rx_ch > current_tx_ch)
                                    ? static_cast<uint8_t>(current_rx_ch - current_tx_ch)
                                    : static_cast<uint8_t>(current_tx_ch - current_rx_ch);
        if (dual_radio && cur_sep < cfg_.min_separation_mhz) {
            decision.should_migrate = true;
            return decision;
        }

        const int busy_threshold =
            is_startup ? cfg_.startup_busy_threshold_pct : cfg_.periodic_busy_threshold_pct;
        if (cur_worst >= busy_threshold &&
            (cur_worst - new_worst) >= cfg_.min_busy_improvement_pct) {
            decision.should_migrate = true;
        }
        return decision;
    }

private:
    ChannelPolicyConfig cfg_;
    int consecutive_tx_loss_ = 0;
    int consecutive_rx_loss_ = 0;
    double last_migration_s_ = -1.0;
};

}  // namespace oxebots_comms
