#pragma once

#include <chrono>
#include <cstdint>
#include <iostream>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "oxebots_interfaces/msg/game_data.hpp"
#include "oxebots_interfaces/msg/robot_cmd.hpp"
#include "oxebots_interfaces/msg/robot_kinematic_pose.hpp"
#include "oxebots_interfaces/msg/robot_telemetry.hpp"
#include "oxebots_interfaces/msg/robot_trajectory_setpoint.hpp"
#include "oxebots_interfaces/ssl_robot_protocol_bp.h"

/**
 * @brief acha automaticamente a porta serial (ex: /dev/ttyUSB0) do dongle USB
 *        pelo Vendor ID (VID) e Product ID (PID) do conversor CH340 (1a86:7523).
 */
std::string Find_Nrf24_port(uint16_t port_pid, uint16_t port_vid);

/**
 * @brief Nó ROS 2 responsável pela ponte de comunicação via rádio nRF24L01 com os robôs.
 *
 * Arquitetura de Comunicação (Protocolo Bitproto):
 * - RobotCommand (30 bytes, MSG_TYPE_COMMAND = 0):
 *     Enviado continuamente (fire-and-forget, sem resposta de telemetria).
 *     Sincronizado na UART aguardando os 6 bytes de status (02 00 00 00 00 00) emitidos
 *     pelo microcontrolador STC15 do dongle USB ao concluir a transmissão RF (~12.5ms).
 * - RobotTelemetryRequest (5 bytes, MSG_TYPE_TELEMETRY_REQ = 3):
 *     Enviado explicitamente a cada N comandos por robô (configurável via parâmetro
 *     `telemetry_request_interval` no `match_config.yaml`), escalonado entre os robôs
 *     para que no máximo um robô responda telemetria por ciclo.
 * - RobotTelemetry (22 bytes, MSG_TYPE_TELEMETRY = 2):
 *     Resposta enviada pela ESP32 do robô exclusivamente quando recebe um RobotTelemetryRequest.
 * - Sondagem de Robôs Offline (Hot-Plug 1 Hz):
 *     Se um robô falhar em responder telemetria consecutivamente, ele continua recebendo
 *     RobotCommand normalmente na frequência máxima, mas seus pedidos de telemetria passam
 *     a ser feitos a 1 Hz até que ele seja ligado/volte a responder.
 */
class Nrf24HardwareBridge : public rclcpp::Node
{
public:
    explicit Nrf24HardwareBridge(bool enable_ros_loop = true);
    ~Nrf24HardwareBridge() override;

private:
    /**
     * @brief Armazena o último comando recebido do ROS 2 para um determinado robô,
     *        permitindo desacoplar a taxa dos tópicos ROS da escrita na porta serial.
     */
    struct PendingRobotCommand
    {
        struct RobotCommand cmd = {};
        std::chrono::steady_clock::time_point updated_at{};
        bool pending = false;
    };

    /**
     * @brief Rastreia o agendamento de requisições de telemetria e o estado online/offline
     *        de cada robô individualmente.
     */
    struct RobotTelemetryState
    {
        int cmd_since_last_tel = 0;
        int consecutive_losses = 0;
        bool is_online = true;
        std::chrono::steady_clock::time_point last_tel_attempt{};
    };

    int serial_fd_ = -1;
    int telemetry_request_interval_ = 5;

    std::map<uint8_t, PendingRobotCommand> pending_commands_;
    std::unordered_map<uint8_t, RobotTelemetryState> telemetry_states_;
    std::unordered_map<uint8_t, double> robot_orientations_;

    // Callbacks de subscrição e timer do ROS 2
    void send_command(const oxebots_interfaces::msg::RobotCmd::SharedPtr msg);
    void send_trajectory_setpoint(
        const oxebots_interfaces::msg::RobotTrajectorySetpoint::SharedPtr msg);
    void game_data_callback(const oxebots_interfaces::msg::GameData::SharedPtr msg);
    void tx_timer_callback();

    // Métodos internos de rádio, serialização Bitproto e configuração AT
    bool transmit_command(const struct RobotCommand& cmd);
    bool request_and_receive_telemetry(uint8_t robot_id);
    void publish_telemetry(const struct RobotTelemetry& telemetry_bp);
    void configure_nrf24();
    bool open_serial_port(const std::string& port_name);
    struct Pose kinematic_pose_to_bitproto(
        const oxebots_interfaces::msg::RobotKinematicPose& msg);

    // Interfaces ROS 2
    rclcpp::Publisher<oxebots_interfaces::msg::RobotTelemetry>::SharedPtr telemetry_pub_;
    rclcpp::Subscription<oxebots_interfaces::msg::RobotCmd>::SharedPtr command_sub_;
    rclcpp::Subscription<oxebots_interfaces::msg::RobotTrajectorySetpoint>::SharedPtr
        trajectory_setpoint_sub_;
    rclcpp::Subscription<oxebots_interfaces::msg::GameData>::SharedPtr game_data_sub_;
    rclcpp::TimerBase::SharedPtr tx_timer_;
};
