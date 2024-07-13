#ifndef UDP_DRIVER_H
#define UDP_DRIVER_H

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
    UdpDriver(io_context & io_context);

    virtual ~UdpDriver() = default;

   protected:
    virtual void on_receive(const ProtoMessageType & packet) = 0;
    void add_host(const std::string & multicast_address, int port);
    void stop();

   private:
    void start_receive();
    void handle_receive(const error_code & error,
                        std::size_t bytes_transferred);

    udp::socket socket_;
    udp::endpoint endpoint_;
    udp::endpoint sender_endpoint_;
    ProtoMessageType packet_;
    static constexpr int bufferSize = 2048;
    char data_[bufferSize];
};

template <typename ProtoMessageType>
UdpDriver<ProtoMessageType>::UdpDriver(io_context & io_context)
: socket_(io_context)
{
    RCLCPP_INFO(get_logger("UdpDriver::UdpDriver"),
                "Driver Up and Running...");
}

template <typename ProtoMessageType>
void UdpDriver<ProtoMessageType>::stop()
{
    error_code error;
    socket_.close(error);

    if (error)
        RCLCPP_ERROR_STREAM(get_logger("UdpDriver::stop"), error.message());
}

template <typename ProtoMessageType>
void UdpDriver<ProtoMessageType>::add_host(
  const std::string & multicast_address, int port)
{
    endpoint_ = udp::endpoint(address::from_string(multicast_address), port);
    socket_.open(endpoint_.protocol());
    socket_.set_option(udp::socket::reuse_address(true));
    socket_.bind(endpoint_);

    socket_.set_option(multicast::join_group(
      address::from_string(multicast_address).to_v4(), address_v4::any()));

    RCLCPP_INFO(get_logger("UdpDriver::add_host"), "Bind to %s:%d.",
                multicast_address.c_str(), port);
    RCLCPP_INFO(get_logger("UdpDriver::add_host"), "Starting receive...");

    start_receive();
}

template <typename ProtoMessageType>
void UdpDriver<ProtoMessageType>::start_receive()
{
    socket_.async_receive_from(
      buffer(data_, bufferSize), sender_endpoint_,
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
        if (packet_.ParseFromArray(data_, bytes_transferred))
            on_receive(packet_);
        else
            RCLCPP_ERROR(get_logger("UdpDriver::handle_receive"),
                         "Failed to parse packet.");
    }
    else
        RCLCPP_ERROR_STREAM(get_logger("UdpDriver::handle_receive"),
                            error.message());
    start_receive();
}

#endif  // UDP_DRIVER_H
