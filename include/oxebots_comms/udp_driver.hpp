#ifndef UDP_DRIVER_H
#define UDP_DRIVER_H

#include <boost/asio.hpp>
#include <boost/asio/ip/multicast.hpp>

#include <iostream>

template <typename ProtoMessageType>
class UdpDriver
{
   public:
    UdpDriver(boost::asio::io_context & io_context,
              const std::string & multicast_address, int port);

    virtual ~UdpDriver() = default;

   protected:
    virtual void on_receive(const ProtoMessageType & packet) = 0;

   private:
    void start_receive();
    void handle_receive(const boost::system::error_code & error,
                        std::size_t bytes_transferred);

    boost::asio::ip::udp::socket socket_;
    boost::asio::ip::udp::endpoint endpoint_;
    boost::asio::ip::udp::endpoint sender_endpoint_;
    ProtoMessageType packet_;
    static constexpr int bufferSize = 2048;
    char data_[bufferSize];
};

template <typename ProtoMessageType>
UdpDriver<ProtoMessageType>::UdpDriver(boost::asio::io_context & io_context,
                                       const std::string & multicast_address,
                                       int port)
: socket_(io_context),
  endpoint_(boost::asio::ip::make_address(multicast_address), port)
{
    socket_.open(endpoint_.protocol());
    socket_.set_option(boost::asio::ip::udp::socket::reuse_address(true));
    socket_.bind(endpoint_);

    socket_.set_option(boost::asio::ip::multicast::join_group(
      boost::asio::ip::make_address(multicast_address).to_v4(),
      boost::asio::ip::address_v4::any()));

    std::cout << "UDP driver up and listening" << std::endl;
    start_receive();
}

template <typename ProtoMessageType>
void UdpDriver<ProtoMessageType>::start_receive()
{
    socket_.async_receive_from(boost::asio::buffer(data_, bufferSize),
                               sender_endpoint_,
                               [this](const boost::system::error_code & error,
                                      std::size_t bytes_transferred) {
                                   handle_receive(error, bytes_transferred);
                               });
}

template <typename ProtoMessageType>
void UdpDriver<ProtoMessageType>::handle_receive(
  const boost::system::error_code & error, std::size_t bytes_transferred)
{
    if (!error)
    {
        if (packet_.ParseFromArray(data_, bytes_transferred))
            on_receive(packet_);
        else
            std::cerr << "Failed to parse packet" << std::endl;
    }
    else
        std::cerr << "Receive error: " << error.message() << std::endl;

    start_receive();
}

#endif  // UDP_DRIVER_H
