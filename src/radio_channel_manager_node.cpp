#include <algorithm>
#include <array>
#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "oxebots_comms/channel_policy.hpp"
#include "oxebots_interfaces/msg/game_state.hpp"
#include "oxebots_interfaces/msg/radio_link_stats.hpp"
#include "oxebots_interfaces/srv/migrate_channel.hpp"
#include "oxebots_interfaces/srv/scan_spectrum.hpp"

class RadioChannelManagerNode : public rclcpp::Node
{
public:
    RadioChannelManagerNode() : Node("radio_channel_manager")
    {
        this->declare_parameter("scan_on_startup", true);
        this->declare_parameter("startup_delay_s", 2.0);
        this->declare_parameter("periodic_scan_interval_s", 30.0);
        this->declare_parameter("scan_samples_per_channel", 24);
        this->declare_parameter("min_channel_separation", 10);
        this->declare_parameter("optimal_channel_separation", 14);
        this->declare_parameter("startup_busy_threshold_pct", 15);
        this->declare_parameter("periodic_busy_threshold_pct", 25);
        this->declare_parameter("min_busy_improvement_pct", 15);
        this->declare_parameter("loss_ack_threshold_pct", 85.0);
        this->declare_parameter("loss_retry_threshold", 1.5);
        this->declare_parameter("loss_tel_rx_threshold_pct", 75.0);
        this->declare_parameter("loss_confirm_windows", 3);
        this->declare_parameter("min_window_tx", 20);
        this->declare_parameter("min_window_tel_req", 6);
        this->declare_parameter("migration_max_attempts", 15);
        this->declare_parameter("migration_cooldown_s", 20.0);
        this->declare_parameter("periodic_only_when_stopped", false);

        scan_on_startup_ = this->get_parameter("scan_on_startup").as_bool();
        startup_delay_s_ = this->get_parameter("startup_delay_s").as_double();
        periodic_scan_interval_s_ = this->get_parameter("periodic_scan_interval_s").as_double();
        scan_samples_ = static_cast<uint8_t>(
            std::clamp<int64_t>(this->get_parameter("scan_samples_per_channel").as_int(), 8, 64));
        migration_max_attempts_ = static_cast<uint8_t>(
            std::clamp<int64_t>(this->get_parameter("migration_max_attempts").as_int(), 1, 50));
        periodic_only_when_stopped_ = this->get_parameter("periodic_only_when_stopped").as_bool();

        oxebots_comms::ChannelPolicyConfig cfg{};
        cfg.min_separation_mhz = static_cast<uint8_t>(
            std::clamp<int64_t>(this->get_parameter("min_channel_separation").as_int(), 2, 40));
        cfg.optimal_separation_mhz = static_cast<uint8_t>(
            std::clamp<int64_t>(this->get_parameter("optimal_channel_separation").as_int(), 4, 50));
        cfg.startup_busy_threshold_pct =
            static_cast<int>(this->get_parameter("startup_busy_threshold_pct").as_int());
        cfg.periodic_busy_threshold_pct =
            static_cast<int>(this->get_parameter("periodic_busy_threshold_pct").as_int());
        cfg.min_busy_improvement_pct =
            static_cast<int>(this->get_parameter("min_busy_improvement_pct").as_int());
        cfg.loss_ack_threshold_pct = this->get_parameter("loss_ack_threshold_pct").as_double();
        cfg.loss_retry_threshold = this->get_parameter("loss_retry_threshold").as_double();
        cfg.loss_tel_rx_threshold_pct =
            this->get_parameter("loss_tel_rx_threshold_pct").as_double();
        cfg.loss_confirm_windows =
            static_cast<int>(this->get_parameter("loss_confirm_windows").as_int());
        cfg.min_window_tx =
            static_cast<uint32_t>(std::max<int64_t>(1, this->get_parameter("min_window_tx").as_int()));
        cfg.min_window_tel_req = static_cast<uint32_t>(
            std::max<int64_t>(1, this->get_parameter("min_window_tel_req").as_int()));
        cfg.migration_cooldown_s = this->get_parameter("migration_cooldown_s").as_double();
        policy_.set_config(cfg);

        scan_client_ =
            this->create_client<oxebots_interfaces::srv::ScanSpectrum>("/nrf24/scan_spectrum");
        migrate_client_ =
            this->create_client<oxebots_interfaces::srv::MigrateChannel>("/nrf24/migrate_channel");

        stats_sub_ = this->create_subscription<oxebots_interfaces::msg::RadioLinkStats>(
            "/nrf24/link_stats",
            10,
            std::bind(&RadioChannelManagerNode::on_link_stats, this, std::placeholders::_1));

        game_state_sub_ = this->create_subscription<oxebots_interfaces::msg::GameState>(
            "/game_state",
            10,
            std::bind(&RadioChannelManagerNode::on_game_state, this, std::placeholders::_1));

        if (scan_on_startup_) {
            const auto delay_ms = static_cast<int64_t>(std::max(0.2, startup_delay_s_) * 1000.0);
            startup_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(delay_ms),
                std::bind(&RadioChannelManagerNode::on_startup_timer, this));
        }

        if (periodic_scan_interval_s_ > 0.0) {
            const auto period_ms =
                static_cast<int64_t>(std::max(5.0, periodic_scan_interval_s_) * 1000.0);
            periodic_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(period_ms),
                std::bind(&RadioChannelManagerNode::on_periodic_timer, this));
        }

        RCLCPP_INFO(
            this->get_logger(),
            "RadioChannelManager pronto (min_sep=%u MHz, optimal_sep=%u MHz, periodic=%.0fs, "
            "max_attempts=%u).",
            cfg.min_separation_mhz,
            cfg.optimal_separation_mhz,
            periodic_scan_interval_s_,
            migration_max_attempts_);
    }

private:
    double now_seconds() const
    {
        return std::chrono::duration<double>(
                   std::chrono::steady_clock::now().time_since_epoch())
            .count();
    }

    void on_game_state(const oxebots_interfaces::msg::GameState::SharedPtr msg)
    {
        if (!msg) return;
        has_game_state_ = true;
        game_stopped_ =
            (msg->phase == oxebots_interfaces::msg::GameState::PHASE_HALT ||
             msg->phase == oxebots_interfaces::msg::GameState::PHASE_TIMEOUT ||
             msg->phase == oxebots_interfaces::msg::GameState::PHASE_STOP);
    }

    void on_link_stats(const oxebots_interfaces::msg::RadioLinkStats::SharedPtr msg)
    {
        if (!msg) return;
        last_stats_ = *msg;
        has_stats_ = true;

        oxebots_comms::LinkHealthSample sample{};
        sample.active_tx_channel = msg->active_tx_channel;
        sample.active_rx_channel = msg->active_rx_channel;
        sample.radio_ready_mask = msg->radio_ready_mask;
        sample.scan_in_progress = msg->scan_in_progress;
        sample.migration_in_progress = msg->migration_in_progress;
        sample.overall_ack_rate_pct = msg->overall_ack_rate_pct;
        sample.overall_avg_retries = msg->overall_avg_retries;
        sample.overall_tel_rx_rate_pct = msg->overall_tel_rx_rate_pct;
        sample.window_tx_count = msg->window_tx_count;
        sample.window_tel_req_count = msg->window_tel_req_count;

        const auto trigger = policy_.evaluate_link_health(sample, now_seconds());
        if (trigger != oxebots_comms::LossTriggerKind::None && !operation_in_flight_) {
            RCLCPP_WARN(
                this->get_logger(),
                "Degradação de link detectada (trigger=%u | ACK=%.1f%% retries=%.2f tel=%.1f%% | "
                "TX=%u RX=%u) — iniciando scan reativo...",
                static_cast<unsigned>(trigger),
                msg->overall_ack_rate_pct,
                msg->overall_avg_retries,
                msg->overall_tel_rx_rate_pct,
                msg->active_tx_channel,
                msg->active_rx_channel);
            trigger_scan_and_evaluate(false, trigger);
        }
    }

    void on_startup_timer()
    {
        if (startup_timer_) {
            startup_timer_->cancel();
        }
        RCLCPP_INFO(this->get_logger(), "Executando scan inicial de espectro RF...");
        trigger_scan_and_evaluate(true, oxebots_comms::LossTriggerKind::None);
    }

    void on_periodic_timer()
    {
        if (operation_in_flight_) return;
        if (periodic_only_when_stopped_ && has_game_state_ && !game_stopped_) {
            return;
        }
        if (policy_.in_cooldown(now_seconds())) {
            return;
        }
        if (has_stats_ && (last_stats_.scan_in_progress || last_stats_.migration_in_progress)) {
            return;
        }
        trigger_scan_and_evaluate(false, oxebots_comms::LossTriggerKind::None);
    }

    void trigger_scan_and_evaluate(bool is_startup, oxebots_comms::LossTriggerKind loss_trigger)
    {
        if (operation_in_flight_) return;
        if (!scan_client_->service_is_ready() || !migrate_client_->service_is_ready()) {
            RCLCPP_DEBUG(
                this->get_logger(),
                "Serviços do nrf24_bridge_node ainda indisponíveis; aguardando próximo ciclo.");
            return;
        }

        operation_in_flight_ = true;
        auto req = std::make_shared<oxebots_interfaces::srv::ScanSpectrum::Request>();
        req->start_channel = 0;
        req->end_channel = 125;
        req->samples_per_channel = scan_samples_;
        req->min_separation = policy_.config().min_separation_mhz;
        req->optimal_separation = policy_.config().optimal_separation_mhz;

        scan_client_->async_send_request(
            req,
            [this, is_startup, loss_trigger](
                rclcpp::Client<oxebots_interfaces::srv::ScanSpectrum>::SharedFuture future) {
                const auto resp = future.get();
                if (!resp || !resp->success) {
                    RCLCPP_WARN(
                        this->get_logger(),
                        "Scan de espectro falhou: %s",
                        resp ? resp->message.c_str() : "sem resposta");
                    operation_in_flight_ = false;
                    return;
                }

                const bool dual_radio =
                    has_stats_ ? ((last_stats_.radio_ready_mask & 0x03U) == 0x03U)
                               : (resp->best_tx_channel != resp->best_rx_channel);

                std::array<uint8_t, 126> busy{};
                std::copy(resp->busy_pct.begin(), resp->busy_pct.end(), busy.begin());

                const auto decision = policy_.decide_from_scan(
                    busy,
                    resp->current_tx_channel,
                    resp->current_rx_channel,
                    dual_radio,
                    is_startup,
                    loss_trigger);

                RCLCPP_INFO(
                    this->get_logger(),
                    "Scan OK | Atual: TX=%u(%u%%) RX=%u(%u%%) | Ótimo: TX=%u(%u%%) RX=%u(%u%%) "
                    "sep=%u MHz | migrar=%s",
                    resp->current_tx_channel,
                    resp->current_tx_busy_pct,
                    resp->current_rx_channel,
                    resp->current_rx_busy_pct,
                    decision.target.tx_channel,
                    decision.target.tx_busy_pct,
                    decision.target.rx_channel,
                    decision.target.rx_busy_pct,
                    decision.target.separation_mhz,
                    decision.should_migrate ? "SIM" : "NAO");

                if (!decision.should_migrate) {
                    operation_in_flight_ = false;
                    return;
                }

                policy_.note_migration_started(now_seconds());
                auto mreq = std::make_shared<oxebots_interfaces::srv::MigrateChannel::Request>();
                mreq->target_tx_channel = decision.target.tx_channel;
                mreq->target_rx_channel = decision.target.rx_channel;
                mreq->max_attempts = migration_max_attempts_;
                mreq->persist_nvs = true;

                migrate_client_->async_send_request(
                    mreq,
                    [this](rclcpp::Client<oxebots_interfaces::srv::MigrateChannel>::SharedFuture
                               mfuture) {
                        const auto mresp = mfuture.get();
                        if (mresp && mresp->accepted) {
                            RCLCPP_INFO(
                                this->get_logger(),
                                "Migração concluída: TX=%u RX=%u | confirmados=%zu | "
                                "nao_confirmados=%zu | %s",
                                mresp->active_tx_channel,
                                mresp->active_rx_channel,
                                mresp->confirmed_ids.size(),
                                mresp->unconfirmed_ids.size(),
                                mresp->message.c_str());
                        } else {
                            RCLCPP_WARN(
                                this->get_logger(),
                                "Migração rejeitada/falhou: %s",
                                mresp ? mresp->message.c_str() : "sem resposta");
                        }
                        operation_in_flight_ = false;
                    });
            });
    }

    bool scan_on_startup_ = true;
    double startup_delay_s_ = 2.0;
    double periodic_scan_interval_s_ = 30.0;
    uint8_t scan_samples_ = 24;
    uint8_t migration_max_attempts_ = 15;
    bool periodic_only_when_stopped_ = false;

    bool has_stats_ = false;
    oxebots_interfaces::msg::RadioLinkStats last_stats_{};
    bool has_game_state_ = false;
    bool game_stopped_ = true;
    bool operation_in_flight_ = false;

    oxebots_comms::ChannelPolicyEngine policy_;
    rclcpp::Client<oxebots_interfaces::srv::ScanSpectrum>::SharedPtr scan_client_;
    rclcpp::Client<oxebots_interfaces::srv::MigrateChannel>::SharedPtr migrate_client_;
    rclcpp::Subscription<oxebots_interfaces::msg::RadioLinkStats>::SharedPtr stats_sub_;
    rclcpp::Subscription<oxebots_interfaces::msg::GameState>::SharedPtr game_state_sub_;
    rclcpp::TimerBase::SharedPtr startup_timer_;
    rclcpp::TimerBase::SharedPtr periodic_timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RadioChannelManagerNode>());
    rclcpp::shutdown();
    return 0;
}
