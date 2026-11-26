#pragma once

#include <chrono>
#include <cstdint>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include "oxebots_comms/base_station_client.hpp"
#include "oxebots_interfaces/base_station_protocol_bp.h"
#include "oxebots_interfaces/msg/game_data.hpp"
#include "oxebots_interfaces/msg/robot_cmd.hpp"
#include "oxebots_interfaces/msg/robot_kinematic_pose.hpp"
#include "oxebots_interfaces/msg/robot_telemetry.hpp"
#include "oxebots_interfaces/msg/robot_trajectory_setpoint.hpp"
#include "oxebots_interfaces/ssl_robot_protocol_bp.h"

/**
 * @brief Nó ROS 2 ponte entre o software de alto nível e a ESP32 Base Station (UART0 COBS+CRC8).
 *
 * Arquitetura de Comunicação:
 * - PC <-> ESP32 Base Station: 921600 baud, COBS(msg_type | bitproto | crc8) + 0x00.
 * - Base Station <-> Robôs: nRF24L01+ 2 Mbps, endereço exclusivo por robô com Auto-ACK em
 *   hardware (Pipe 1) + Pipe 2 broadcast NO_ACK para comandos globais/emergência.
 * - Agilidade de Canal: suporta varredura de espectro RPD (0..125) e migração coordenada de
 *   canal via CONFIG_FLAG_SET_CHANNEL (robôs) + BS_MSG_SET_CONFIG (Base).
 */
class Nrf24HardwareBridge : public rclcpp::Node
{
public:
    explicit Nrf24HardwareBridge(bool enable_ros_loop = true);
    ~Nrf24HardwareBridge() override;

    oxebots_comms::BaseStationClient& base_client() { return base_client_; }

private:
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
        bool is_online = true;
        std::chrono::steady_clock::time_point last_tel_attempt{};
    };

    oxebots_comms::BaseStationClient base_client_;
    int telemetry_request_interval_ = 5;
    double max_kick_speed_ = 3.0;

    std::map<uint8_t, PendingRobotCommand> pending_commands_;
    std::vector<uint8_t> pending_calibrations_;
    std::unordered_map<uint8_t, RobotLinkState> link_states_;
    std::unordered_map<uint8_t, double> robot_orientations_;

    void send_command(const oxebots_interfaces::msg::RobotCmd::SharedPtr msg);
    void send_trajectory_setpoint(
        const oxebots_interfaces::msg::RobotTrajectorySetpoint::SharedPtr msg);
    void game_data_callback(const oxebots_interfaces::msg::GameData::SharedPtr msg);
    void calibrate_robot_callback(const std_msgs::msg::UInt8::SharedPtr msg);
    void tx_timer_callback();

    void configure_base_station();
    bool transmit_command(const struct RobotCommand& cmd, struct BaseRfTxStatus* out_status);
    bool request_and_receive_telemetry(uint8_t robot_id);
    void publish_telemetry(const struct RobotTelemetry& telemetry_bp);
    struct Pose kinematic_pose_to_bitproto(
        const oxebots_interfaces::msg::RobotKinematicPose& msg);

    rclcpp::Publisher<oxebots_interfaces::msg::RobotTelemetry>::SharedPtr telemetry_pub_;
    rclcpp::Subscription<oxebots_interfaces::msg::RobotCmd>::SharedPtr command_sub_;
    rclcpp::Subscription<oxebots_interfaces::msg::RobotTrajectorySetpoint>::SharedPtr
        trajectory_setpoint_sub_;
    rclcpp::Subscription<oxebots_interfaces::msg::GameData>::SharedPtr game_data_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr calibrate_robot_sub_;
    rclcpp::TimerBase::SharedPtr tx_timer_;
};
