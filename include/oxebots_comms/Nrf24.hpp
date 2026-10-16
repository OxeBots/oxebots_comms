#pragma once

#include <iostream>
#include <cstdint>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include "oxebots_interfaces/msg/robot_cmd.hpp"
#include "oxebots_interfaces/msg/robot_telemetry.hpp"
#include "oxebots_interfaces/ssl_robot_protocol_bp.h"

// USB find func
std::string Find_Nrf24_port(uint16_t port_pid, uint16_t port_vid);

class Nrf24HardwareBridge : public rclcpp::Node
{
public:
    Nrf24HardwareBridge();
    ~Nrf24HardwareBridge();

private:
    int serial_fd_ = -1; // Descritor de ficheiro nativo do Linux

    // Intern func
    void send_command(const oxebots_interfaces::msg::RobotCmd::SharedPtr msg);
    bool send_and_wait_ack(const uint8_t* tx_buffer, uint8_t robot_id);
    void publish_telemetry(const struct RobotTelemetry& telemetry_bp);
    void configure_nrf24();
    bool open_serial_port(const std::string& port_name);

    rclcpp::Publisher<oxebots_interfaces::msg::RobotTelemetry>::SharedPtr telemetry_pub_;
    rclcpp::Subscription<oxebots_interfaces::msg::RobotCmd>::SharedPtr command_sub_;
};