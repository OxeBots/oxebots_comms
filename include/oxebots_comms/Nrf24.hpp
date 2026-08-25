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
#include "oxebots_interfaces/msg/robot_telemetry.hpp"
#include "oxebots_interfaces/ssl_robot_protocol_bp.h"
#include <sys/ioctl.h>
#include <algorithm>

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
    uint8_t recive_buffer_[256]; // USB enter buff
    std::vector<uint8_t> persistent_buffer_; // Cumulative buff ROS

    // Intern func
    void send_command(const oxebots_interfaces::msg::RobotCmd::SharedPtr msg);
    void start_receive();
    void handle_receive(const boost::system::error_code& error, std::size_t bytes_transferred);
    void publish_telemetry(const struct RobotTelemetry& telemetry_bp);
    void configure_nrf24();

    // Creates the publisher and the subscriber
    rclcpp::Publisher<oxebots_interfaces::msg::RobotTelemetry>::SharedPtr telemetry_pub_;
    rclcpp::Subscription<oxebots_interfaces::msg::RobotCmd>::SharedPtr command_sub_;

    // Creates the I/O execution context object and the serial context objetct
    boost::asio::io_context io_context_;
    std::unique_ptr<boost::asio::serial_port> serial_;

    // Asio thread
    std::thread io_thread_;
};