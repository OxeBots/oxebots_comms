#pragma once

#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include "oxebots_comms/base_station_client.hpp"
#include "oxebots_comms/channel_migration.hpp"
#include "oxebots_comms/channel_policy.hpp"
#include "oxebots_interfaces/base_station_protocol_bp.h"
#include "oxebots_interfaces/msg/game_data.hpp"
#include "oxebots_interfaces/msg/radio_link_stats.hpp"
#include "oxebots_interfaces/msg/robot_cmd.hpp"
#include "oxebots_interfaces/msg/robot_kinematic_pose.hpp"
#include "oxebots_interfaces/msg/robot_telemetry.hpp"
#include "oxebots_interfaces/msg/robot_trajectory_setpoint.hpp"
#include "oxebots_interfaces/srv/migrate_channel.hpp"
#include "oxebots_interfaces/srv/scan_spectrum.hpp"
#include "oxebots_interfaces/srv/set_robot_id.hpp"
#include "oxebots_interfaces/ssl_robot_protocol_bp.h"

/**
 * @brief Nó ROS 2 ponte entre o software de alto nível e a ESP32 Base Station (UART0 COBS+CRC8).
 *
 * Arquitetura de Comunicação:
 * - PC <-> ESP32 Base Station: 921600 baud, COBS(msg_type | bitproto | crc8) + 0x00 (v2).
 * - Base Station Dual-Radio Full-Duplex: Rádio TX dedicado em tx_ch (downlink + Auto-ACK) e
 *   Rádio RX dedicado em rx_ch (uplink telemetria + scan RPD em background sem pausar TX).
 *   Fallback automático para 1 rádio (rx_ch == tx_ch) quando o 2º módulo RX estiver ausente.
 * - Serviços não-bloqueantes (/nrf24/scan_spectrum, /nrf24/migrate_channel, /nrf24/set_robot_id)
 *   e publicação de qualidade de link em /nrf24/link_stats (2 Hz).
 */
class Nrf24HardwareBridge : public rclcpp::Node
{
public:
    explicit Nrf24HardwareBridge(bool enable_ros_loop = true);
    ~Nrf24HardwareBridge() override;

    oxebots_comms::BaseStationClient& base_client() { return base_client_; }

private:
    using ScanSrv = oxebots_interfaces::srv::ScanSpectrum;
    using MigrateSrv = oxebots_interfaces::srv::MigrateChannel;
    using SetIdSrv = oxebots_interfaces::srv::SetRobotId;

    struct PendingRobotCommand
    {
        struct RobotCommand cmd = {};
        std::chrono::steady_clock::time_point updated_at{};
        bool pending = false;
    };

    struct RobotLinkState
    {
        int cmd_since_last_tel = 0;
        int consecutive_tx_fails = 0;
        int consecutive_tel_losses = 0;
        int consecutive_tel_losses_with_tx_ok = 0;
        bool is_online = true;
        std::chrono::steady_clock::time_point last_tel_attempt{};
        std::chrono::steady_clock::time_point last_tx_ok{};

        // 2 Hz sliding window counters for /nrf24/link_stats
        uint32_t win_tx_sent = 0;
        uint32_t win_tx_ack_ok = 0;
        uint32_t win_tx_retries_sum = 0;
        uint32_t win_tel_req = 0;
        uint32_t win_tel_rx = 0;
    };

    oxebots_comms::BaseStationClient base_client_;
    struct BaseInfo base_info_ = {};
    int telemetry_request_interval_ = 5;
    double max_kick_speed_ = 3.0;

    std::map<uint8_t, PendingRobotCommand> pending_commands_;
    std::vector<uint8_t> pending_calibrations_;
    std::unordered_map<uint8_t, RobotLinkState> link_states_;
    std::unordered_map<uint8_t, double> robot_orientations_;
    std::set<uint8_t> vision_ally_ids_;

    // Async background RX scan state
    bool scan_in_progress_ = false;
    std::shared_ptr<rmw_request_id_t> pending_scan_header_;
    std::shared_ptr<ScanSrv::Request> pending_scan_req_;
    std::chrono::steady_clock::time_point scan_started_at_{};

    // Zero-downtime interleaved channel migration state
    oxebots_comms::ChannelMigrationMachine migration_;
    std::shared_ptr<rmw_request_id_t> pending_migrate_header_;

    void send_command(const oxebots_interfaces::msg::RobotCmd::SharedPtr msg);
    void send_trajectory_setpoint(
        const oxebots_interfaces::msg::RobotTrajectorySetpoint::SharedPtr msg);
    void game_data_callback(const oxebots_interfaces::msg::GameData::SharedPtr msg);
    void calibrate_robot_callback(const std_msgs::msg::UInt8::SharedPtr msg);
    void tx_timer_callback();
    void publish_link_stats();

    void handle_scan_spectrum(
        std::shared_ptr<rmw_request_id_t> request_header,
        std::shared_ptr<ScanSrv::Request> request);
    void handle_migrate_channel(
        std::shared_ptr<rmw_request_id_t> request_header,
        std::shared_ptr<MigrateSrv::Request> request);
    void handle_set_robot_id(
        const std::shared_ptr<SetIdSrv::Request> request,
        std::shared_ptr<SetIdSrv::Response> response);

    void poll_async_scan();
    void step_channel_migration();
    void finish_channel_migration();
    bool apply_base_channels(uint8_t tx_ch, uint8_t rx_ch, bool persist_nvs);
    std::vector<uint8_t> collect_active_robot_ids(const std::vector<uint8_t>& explicit_ids) const;

    void configure_base_station();
    bool transmit_command(const struct RobotCommand& cmd, struct BaseRfTxStatus* out_status);
    bool request_and_receive_telemetry(uint8_t robot_id);
    void publish_telemetry(const struct RobotTelemetry& telemetry_bp);
    struct Pose kinematic_pose_to_bitproto(
        const oxebots_interfaces::msg::RobotKinematicPose& msg);

    rclcpp::Publisher<oxebots_interfaces::msg::RobotTelemetry>::SharedPtr telemetry_pub_;
    rclcpp::Publisher<oxebots_interfaces::msg::RadioLinkStats>::SharedPtr link_stats_pub_;
    rclcpp::Subscription<oxebots_interfaces::msg::RobotCmd>::SharedPtr command_sub_;
    rclcpp::Subscription<oxebots_interfaces::msg::RobotTrajectorySetpoint>::SharedPtr
        trajectory_setpoint_sub_;
    rclcpp::Subscription<oxebots_interfaces::msg::GameData>::SharedPtr game_data_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr calibrate_robot_sub_;
    rclcpp::Service<ScanSrv>::SharedPtr scan_srv_;
    rclcpp::Service<MigrateSrv>::SharedPtr migrate_srv_;
    rclcpp::Service<SetIdSrv>::SharedPtr set_id_srv_;
    rclcpp::TimerBase::SharedPtr tx_timer_;
    rclcpp::TimerBase::SharedPtr link_stats_timer_;
};
