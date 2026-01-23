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

#include <boost/asio.hpp>
#include <rclcpp/logging.hpp>

#include <array>
#include <string>
#include <thread>
#include <vector>

using boost::asio::ip::address;
using boost::asio::ip::address_v4;
using boost::asio::ip::udp;
using rclcpp::get_logger;

template <typename ProtoMessageType>
class UdpSender
{
   public:
    UdpSender(const std::string & ip_address, uint16_t port, const std::string & interface_address = "");

    ~UdpSender();

    void send(const ProtoMessageType & message);

   private:
    boost::asio::io_context io_context_;
    udp::socket socket_;
    udp::endpoint endpoint_;
    std::thread io_thread_;
};

template <typename ProtoMessageType>
UdpSender<ProtoMessageType>::UdpSender(const std::string & ip_address, uint16_t port,
                                       const std::string & interface_address)
: socket_(io_context_)
{
    try
    {
        const auto addr = address::from_string(ip_address);
        endpoint_ = udp::endpoint(addr, port);

        socket_.open(endpoint_.protocol());

        if (addr.is_multicast() && !interface_address.empty())
        {
            auto iface_addr = address::from_string(interface_address).to_v4();
            socket_.set_option(boost::asio::ip::multicast::outbound_interface(iface_addr));
            RCLCPP_INFO(get_logger("UdpSender"), "Multicast output interface set to: %s", interface_address.c_str());
        }

        io_thread_ = std::thread([this]() { io_context_.run(); });

        RCLCPP_INFO(get_logger("UdpSender"), "UDP sender initialized for %s:%d", ip_address.c_str(), port);
    }
    catch (const std::exception & e)
    {
        RCLCPP_ERROR(get_logger("UdpSender"), "Error initializing UdpSender: %s", e.what());
    }
}

template <typename ProtoMessageType>
UdpSender<ProtoMessageType>::~UdpSender()
{
    io_context_.stop();
    if (io_thread_.joinable())
        io_thread_.join();
}

template <typename ProtoMessageType>
void UdpSender<ProtoMessageType>::send(const ProtoMessageType & message)
{
    auto buffer = std::make_shared<std::vector<uint8_t>>(message.ByteSizeLong());
    message.SerializeToArray(buffer->data(), buffer->size());

    socket_.async_send_to(boost::asio::buffer(*buffer), endpoint_,
                          [this, buffer](boost::system::error_code ec, std::size_t /*bytes_sent*/) {
                              if (ec)
                                  RCLCPP_ERROR(get_logger("UdpSender::send"), "Error: %s", ec.message().c_str());
                          });
}
