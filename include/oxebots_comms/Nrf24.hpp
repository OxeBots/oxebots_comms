#pragma once

#include <iostream>
#include <cstdint>
#include <string>
#include <vector>
#include <thread>
#include <memory>
#include <libserialport.h>
#include <boost/asio.hpp>
#include <termios.h>
#include "rclcpp/rclcpp.hpp"
#include "oxebots_interfaces/msg/robot_cmd.hpp"
#include "oxebots_interfaces/msg/robot_trajectory_setpoint.hpp"
#include "oxebots_interfaces/msg/robot_telemetry.hpp"
#include "oxebots_interfaces/ssl_robot_protocol_bp.h"
#include <queue>
#include <sys/ioctl.h>
#include <algorithm>
#include <utility>

// USB find func
std::string Find_Nrf24_port(uint16_t port_pid, uint16_t port_vid);

class Nrf24HardwareBridge : public rclcpp::Node
{
public:
    // construct the node
    Nrf24HardwareBridge();
    
    // Destruct the node
    ~Nrf24HardwareBridge();

private:
    std::queue<std::pair<uint8_t, std::vector<uint8_t>>> tx_queue_; // sending queue <robot_id, package>
    bool write_in_progress_ = false;                                // write control

    uint8_t recive_buffer_[256]; // USB enter buff
    std::vector<uint8_t> persistent_buffer_; // Cumulative buff ROS

    // Intern func
    void send_command(const oxebots_interfaces::msg::RobotCmd::SharedPtr msg);
    void send_trajectory_setpoint(const oxebots_interfaces::msg::RobotTrajectorySetpoint::SharedPtr msg);
    void enqueue_packet(uint8_t robot_id, struct RobotCommand cmd_bitproto);
    static struct Pose kinematic_pose_to_bitproto(const oxebots_interfaces::msg::RobotKinematicPose& pose);
    void start_receive();
    void handle_receive(const boost::system::error_code& error, std::size_t bytes_transferred);
    void publish_telemetry(const struct RobotTelemetry& telemetry_bp);
    void configure_nrf24();
    void start_next_write();

    // Creates the publisher and the subscriber
    rclcpp::Publisher<oxebots_interfaces::msg::RobotTelemetry>::SharedPtr telemetry_pub_;
    // "velocity" mode (default): le /robot_commands e manda so x_v,y_v,angular_vel (como hoje).
    // "point" mode: le /robot_trajectory_setpoints e manda o setpoint completo (ponto futuro +
    // velocidade + angulo + vision_pose), deixando o firmware do robo calcular as velocidades
    // de roda localmente em malha fechada. So uma das duas assinaturas fica ativa por vez.
    rclcpp::Subscription<oxebots_interfaces::msg::RobotCmd>::SharedPtr command_sub_;
    rclcpp::Subscription<oxebots_interfaces::msg::RobotTrajectorySetpoint>::SharedPtr trajectory_setpoint_sub_;

    // Creates the I/O execution context object and the serial context objetct
    boost::asio::io_context io_context_;
    std::unique_ptr<boost::asio::serial_port> serial_;

    // Asio thread
    std::thread io_thread_;
};