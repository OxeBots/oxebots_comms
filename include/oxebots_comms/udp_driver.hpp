#ifndef UDP_DRIVER_H
#define UDP_DRIVER_H

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <netinet/in.h>

#include <boost/asio.hpp>
#include <boost/asio/ip/multicast.hpp>

#include <iostream>

#include "rclcpp/logging.hpp"

using namespace boost::asio::ip;

using boost::asio::buffer;
using boost::asio::io_context;
using boost::asio::error::message_size;
using boost::system::error_code;
using rclcpp::get_logger;

template <typename ProtoMessageType>
class UdpDriver
{
   public:
    UdpDriver();

    virtual ~UdpDriver();

   protected:
    virtual void on_receive(const ProtoMessageType & packet) = 0;
    void add_host(const std::string & multicast_address, uint16_t port);

   private:
    void start_receive();
    void handle_receive(const error_code & error,
                        std::size_t bytes_transferred);

    io_context io_context_;
    udp::endpoint endpoint_;
    udp::endpoint sender_endpoint_;
    ProtoMessageType packet_;

    udp::socket socket_;
    std::thread io_thread_;

    static constexpr uint16_t bufferSize = 4096;
    std::array<uint8_t, bufferSize> data_;
};

template <typename ProtoMessageType>
UdpDriver<ProtoMessageType>::UdpDriver() : io_context_(), socket_(io_context_)
{
    RCLCPP_INFO(get_logger("UdpDriver::UdpDriver"),
                "Driver Up and Running...");
}

template <typename ProtoMessageType>
UdpDriver<ProtoMessageType>::~UdpDriver()
{
    error_code error;
    socket_.close(error);

    if (error)
        RCLCPP_ERROR(rclcpp::get_logger("UdpDriver"),
                     "Error closing socket: %s", error.message().c_str());

    io_context_.stop();

    if (io_thread_.joinable()) io_thread_.join();
}

template <typename ProtoMessageType>
void UdpDriver<ProtoMessageType>::add_host(
  const std::string & multicast_address, uint16_t port)
{
    try
    {
        const auto addr = address::from_string(multicast_address).to_v4();

        endpoint_ = udp::endpoint(addr, port);

        socket_.open(endpoint_.protocol());
        socket_.set_option(udp::socket::reuse_address(true));
        socket_.bind(endpoint_);

        socket_.set_option(multicast::join_group(addr, address_v4::any()));

        RCLCPP_INFO(rclcpp::get_logger("UdpDriver::add_host"),
                    "Bound to %s:%d. Starting receive...",
                    multicast_address.c_str(), port);
        start_receive();

        io_thread_ = std::thread([this]() { io_context_.run(); });
    }
    catch (const boost::system::system_error & e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("UdpDriver::add_host"),
                     "Error adding host: %s", e.what());
    }
}

template <typename ProtoMessageType>
void UdpDriver<ProtoMessageType>::start_receive()
{
    socket_.async_receive_from(
      buffer(data_), sender_endpoint_,
      [this](const error_code & error, std::size_t bytes_transferred) {
          handle_receive(error, bytes_transferred);
      });
}

template <typename ProtoMessageType>
void UdpDriver<ProtoMessageType>::handle_receive(const error_code & error,
                                                 std::size_t bytes_transferred)
{
    if (!error || error == message_size)
    {
        if (packet_.ParseFromArray(data_.data(), bytes_transferred))
            on_receive(packet_);
        else
            RCLCPP_ERROR(get_logger("UdpDriver::handle_receive"),
                         "Failed to parse packet.");
    }
    else
        RCLCPP_ERROR(rclcpp::get_logger("UdpDriver::handle_receive"),
                     "Receive error: %s", error.message().c_str());

    start_receive();
}

#endif  // UDP_DRIVER_H
