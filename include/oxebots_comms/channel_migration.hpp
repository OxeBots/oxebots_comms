#pragma once

#include <algorithm>
#include <cstdint>
#include <map>
#include <set>
#include <vector>

namespace oxebots_comms
{

static constexpr uint8_t kMaxRfChannel = 125;
static constexpr uint8_t kRxChannelFollowTx = 0xFF;

// Encode a (base_tx_ch, base_rx_ch) pair into RobotConfig.param (uint16).
// Bit 15 = 0 -> legacy same-channel; Bit 15 = 1 -> Full-Duplex split-channel.
inline constexpr uint16_t encode_channel_pair_param(uint8_t base_tx_ch, uint8_t base_rx_ch)
{
    if (base_rx_ch == kRxChannelFollowTx || base_rx_ch == base_tx_ch) {
        return static_cast<uint16_t>(base_tx_ch & 0x7FU);
    }
    return static_cast<uint16_t>(
        0x8000U | (static_cast<uint16_t>(base_rx_ch & 0x7FU) << 8U) |
        static_cast<uint16_t>(base_tx_ch & 0x7FU));
}

inline bool decode_channel_pair_param(
    uint16_t param, uint8_t* out_downlink_ch, uint8_t* out_uplink_ch)
{
    if (out_downlink_ch == nullptr || out_uplink_ch == nullptr) return false;
    if ((param & 0x8000U) == 0U) {
        if (param > kMaxRfChannel) return false;
        const uint8_t ch = static_cast<uint8_t>(param);
        *out_downlink_ch = ch;
        *out_uplink_ch = ch;
        return true;
    }
    const uint8_t dl = static_cast<uint8_t>(param & 0x7FU);
    const uint8_t ul = static_cast<uint8_t>((param >> 8U) & 0x7FU);
    if (dl > kMaxRfChannel || ul > kMaxRfChannel) return false;
    *out_downlink_ch = dl;
    *out_uplink_ch = ul;
    return true;
}

/**
 * @brief Deterministic state machine for zero-downtime Full-Duplex channel migration.
 *
 * Robots that have already confirmed on the new channel pair immediately receive normal
 * 100 Hz strategy commands without interruption, while unconfirmed robots receive interleaved
 * SET_CHANNEL retries on old_tx_channel (up to max_attempts, default 15) followed by active
 * verification on (target_tx_channel, target_rx_channel).
 *
 * Confirmation rule:
 *  - If target_tx_channel != old_tx_channel: hardware Auto-ACK on target_tx_channel proves
 *    the robot decoded the atomic RobotConfig pair (both downlink and uplink updated).
 *  - If target_tx_channel == old_tx_channel (only RX/uplink changed): requires a valid
 *    RobotTelemetry frame received on target_rx_channel.
 *
 * Safety rule:
 *  - If initial_targets was non-empty and ZERO robots confirm after max_attempts, reports
 *    rollback_required = true so the bridge reverts to (old_tx_channel, old_rx_channel).
 */
class ChannelMigrationMachine
{
public:
    enum class ActionType : uint8_t
    {
        SendSetChannelOnOld,
        SendBroadcastOnOld,
        VerifyOnNew,
        VerifyTelemetryOnNew,
    };

    struct Action
    {
        ActionType type = ActionType::SendSetChannelOnOld;
        uint8_t robot_id = 0;
        uint8_t rf_channel = 0;
        uint16_t config_param = 0;
    };

    struct Summary
    {
        bool finished = false;
        bool rollback_required = false;
        bool persist_nvs = false;
        uint8_t final_tx_channel = 0;
        uint8_t final_rx_channel = 0;
        std::vector<uint8_t> confirmed_ids;
        std::vector<uint8_t> unconfirmed_ids;
    };

    void start(
        uint8_t old_tx_ch,
        uint8_t old_rx_ch,
        uint8_t target_tx_ch,
        uint8_t target_rx_ch,
        const std::vector<uint8_t>& active_robot_ids,
        uint8_t max_attempts = 15,
        int64_t retry_period_ms = 40,
        bool persist_nvs = true)
    {
        old_tx_ch_ = old_tx_ch;
        old_rx_ch_ = old_rx_ch;
        target_tx_ch_ = target_tx_ch;
        target_rx_ch_ = (target_rx_ch == kRxChannelFollowTx) ? target_tx_ch : target_rx_ch;
        config_param_ = encode_channel_pair_param(target_tx_ch_, target_rx_ch_);
        max_attempts_ = (max_attempts == 0) ? 15 : max_attempts;
        retry_period_ms_ = std::max<int64_t>(10, retry_period_ms);
        persist_nvs_ = persist_nvs;
        requires_telemetry_confirm_ = (target_tx_ch_ == old_tx_ch_ && target_rx_ch_ != old_rx_ch_);

        pending_.clear();
        confirmed_.clear();
        failed_.clear();
        attempts_.clear();
        last_attempt_ms_.clear();
        broadcast_sent_count_ = 0;

        for (uint8_t rid : active_robot_ids) {
            if (rid <= 10) {
                pending_.insert(rid);
                attempts_[rid] = 0;
                last_attempt_ms_[rid] = -retry_period_ms_;  // Immediate first attempt
            }
        }
        had_initial_targets_ = !pending_.empty();
        active_ = true;
    }

    bool is_active() const { return active_; }
    uint8_t old_tx_channel() const { return old_tx_ch_; }
    uint8_t old_rx_channel() const { return old_rx_ch_; }
    uint8_t target_tx_channel() const { return target_tx_ch_; }
    uint8_t target_rx_channel() const { return target_rx_ch_; }
    uint16_t config_param() const { return config_param_; }
    bool requires_telemetry_confirm() const { return requires_telemetry_confirm_; }

    // Confirmed robots (and any robot not in pending_) keep receiving normal 100 Hz commands!
    bool is_robot_pending(uint8_t rid) const { return active_ && pending_.count(rid) > 0; }
    bool is_robot_confirmed(uint8_t rid) const { return confirmed_.count(rid) > 0; }

    std::vector<Action> tick(int64_t now_ms)
    {
        std::vector<Action> actions;
        if (!active_) return actions;

        if (!had_initial_targets_) {
            // No known active robots: emit 3 broadcast SET_CHANNEL copies on old_tx_ch and finish.
            if (broadcast_sent_count_ < 3) {
                Action bcast{};
                bcast.type = ActionType::SendBroadcastOnOld;
                bcast.robot_id = 15;
                bcast.rf_channel = old_tx_ch_;
                bcast.config_param = config_param_;
                actions.push_back(bcast);
                ++broadcast_sent_count_;
            }
            if (broadcast_sent_count_ >= 3) {
                active_ = false;
            }
            return actions;
        }

        bool any_attempt_this_tick = false;
        uint8_t max_attempt_seen = 0;

        for (auto it = pending_.begin(); it != pending_.end();) {
            const uint8_t rid = *it;
            if (attempts_[rid] >= max_attempts_) {
                failed_.insert(rid);
                it = pending_.erase(it);
                continue;
            }
            if (now_ms - last_attempt_ms_[rid] >= retry_period_ms_) {
                attempts_[rid]++;
                last_attempt_ms_[rid] = now_ms;
                any_attempt_this_tick = true;
                max_attempt_seen = std::max(max_attempt_seen, attempts_[rid]);

                Action send_old{};
                send_old.type = ActionType::SendSetChannelOnOld;
                send_old.robot_id = rid;
                send_old.rf_channel = old_tx_ch_;
                send_old.config_param = config_param_;
                actions.push_back(send_old);

                Action verify{};
                verify.type = requires_telemetry_confirm_ ? ActionType::VerifyTelemetryOnNew
                                                          : ActionType::VerifyOnNew;
                verify.robot_id = rid;
                verify.rf_channel = target_tx_ch_;
                verify.config_param = config_param_;
                actions.push_back(verify);
            }
            ++it;
        }

        // Emit a periodic NO_ACK broadcast on old_tx_ch every 3 retry rounds for stragglers.
        if (any_attempt_this_tick && (max_attempt_seen % 3U == 0U)) {
            Action bcast{};
            bcast.type = ActionType::SendBroadcastOnOld;
            bcast.robot_id = 15;
            bcast.rf_channel = old_tx_ch_;
            bcast.config_param = config_param_;
            actions.push_back(bcast);
        }

        if (pending_.empty()) {
            active_ = false;
        }
        return actions;
    }

    // Notify that robot_id returned hardware Auto-ACK on target_tx_channel.
    void on_new_channel_ack(uint8_t robot_id)
    {
        if (!active_ || requires_telemetry_confirm_) return;
        if (pending_.erase(robot_id) > 0) {
            confirmed_.insert(robot_id);
            if (pending_.empty()) {
                active_ = false;
            }
        }
    }

    // Notify that robot_id sent a valid RobotTelemetry frame received on target_rx_channel.
    void on_new_channel_telemetry(uint8_t robot_id)
    {
        if (!active_) return;
        if (pending_.erase(robot_id) > 0) {
            confirmed_.insert(robot_id);
            if (pending_.empty()) {
                active_ = false;
            }
        }
    }

    // Check if migration just finished (call after tick() / confirmation callbacks).
    Summary summary() const
    {
        Summary s{};
        s.finished = !active_;
        s.confirmed_ids.assign(confirmed_.begin(), confirmed_.end());
        s.unconfirmed_ids.assign(failed_.begin(), failed_.end());
        for (uint8_t rid : pending_) {
            s.unconfirmed_ids.push_back(rid);
        }

        if (had_initial_targets_ && confirmed_.empty() && !active_) {
            s.rollback_required = true;
            s.persist_nvs = false;
            s.final_tx_channel = old_tx_ch_;
            s.final_rx_channel = old_rx_ch_;
        } else {
            s.rollback_required = false;
            s.persist_nvs = persist_nvs_;
            s.final_tx_channel = target_tx_ch_;
            s.final_rx_channel = target_rx_ch_;
        }
        return s;
    }

private:
    bool active_ = false;
    bool had_initial_targets_ = false;
    bool persist_nvs_ = true;
    bool requires_telemetry_confirm_ = false;
    uint8_t old_tx_ch_ = 76;
    uint8_t old_rx_ch_ = 90;
    uint8_t target_tx_ch_ = 76;
    uint8_t target_rx_ch_ = 90;
    uint16_t config_param_ = 76;
    uint8_t max_attempts_ = 15;
    int64_t retry_period_ms_ = 40;
    int broadcast_sent_count_ = 0;

    std::set<uint8_t> pending_;
    std::set<uint8_t> confirmed_;
    std::set<uint8_t> failed_;
    std::map<uint8_t, uint8_t> attempts_;
    std::map<uint8_t, int64_t> last_attempt_ms_;
};

}  // namespace oxebots_comms
