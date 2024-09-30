// Copyright 2024 Oxebots
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#pragma once

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <netinet/in.h>

#include <iostream>
#include <string>

#include <boost/asio.hpp>
#include <boost/asio/ip/multicast.hpp>

#include "rclcpp/logging.hpp"

using boost::asio::buffer;
using boost::asio::io_context;
using boost::asio::error::message_size;
using boost::asio::ip::address;
using boost::asio::ip::address_v4;
using boost::asio::ip::udp;
using boost::asio::ip::multicast::join_group;
using boost::system::error_code;
using boost::system::system_error;
using rclcpp::get_logger;

template<typename ProtoMessageType>
class UdpDriver
{
public:
  UdpDriver(
    const std::string & multicast_address, uint16_t port,
    const std::string & interface_address = "");

  virtual ~UdpDriver();

protected:
  virtual void on_receive(const ProtoMessageType & packet) = 0;

private:
  void start_receive();
  void handle_receive(const error_code & error, std::size_t bytes_transferred);

  io_context io_context_;
  udp::endpoint endpoint_;
  udp::endpoint sender_endpoint_;
  ProtoMessageType packet_;

  udp::socket socket_;
  std::thread io_thread_;

  static constexpr uint16_t bufferSize = 4096;
  std::array<uint8_t, bufferSize> data_;
};

template<typename ProtoMessageType>
UdpDriver<ProtoMessageType>::UdpDriver(
  const std::string & multicast_address, uint16_t port, const std::string & interface_address)
: io_context_(), socket_(io_context_)
{
  try {
    const auto addr = address::from_string(multicast_address).to_v4();
    endpoint_ = udp::endpoint(addr, port);

    socket_.open(endpoint_.protocol());
    socket_.set_option(udp::socket::reuse_address(true));
    socket_.bind(endpoint_);

    auto iface_addr = address_v4::any();

    if (interface_address.empty()) {
      socket_.set_option(join_group(addr, iface_addr));

    } else {
      iface_addr = address::from_string(interface_address).to_v4();
      socket_.set_option(join_group(addr, iface_addr));
    }

    RCLCPP_INFO(
      rclcpp::get_logger("UdpDriver"), "Bound to %s:%d from %s", multicast_address.c_str(), port,
      iface_addr.to_string().c_str());

    start_receive();

    io_thread_ = std::thread([this]() {io_context_.run();});

    RCLCPP_INFO(get_logger("UdpDriver"), "Driver Up and Running...");
  } catch (const system_error & e) {
    RCLCPP_ERROR(rclcpp::get_logger("UdpDriver"), "Error initializing UdpDriver: %s", e.what());
  }
}

template<typename ProtoMessageType>
UdpDriver<ProtoMessageType>::~UdpDriver()
{
  error_code error;

  if (socket_.is_open()) {
    socket_.cancel(error);
    socket_.close(error);
  }

  if (error) {
    RCLCPP_ERROR(
      rclcpp::get_logger("UdpDriver"), "Error closing socket: %s", error.message().c_str());
  }

  io_context_.stop();

  if (io_thread_.joinable()) {
    io_thread_.join();
  }
}

template<typename ProtoMessageType>
void UdpDriver<ProtoMessageType>::start_receive()
{
  socket_.async_receive_from(
    buffer(data_), sender_endpoint_,
    [this](const error_code & error, std::size_t bytes_transferred) {
      handle_receive(error, bytes_transferred);
    });
}

template<typename ProtoMessageType>
void UdpDriver<ProtoMessageType>::handle_receive(
  const error_code & error, std::size_t bytes_transferred)
{
  if (!error || error == message_size) {
    if (packet_.ParseFromArray(data_.data(), bytes_transferred)) {
      on_receive(packet_);

    } else {
      RCLCPP_ERROR(get_logger("UdpDriver::handle_receive"), "Failed to parse packet.");
    }
  } else {
    RCLCPP_ERROR(
      rclcpp::get_logger("UdpDriver::handle_receive"), "Receive error: %s",
      error.message().c_str());
  }

  start_receive();
}
