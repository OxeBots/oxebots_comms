#pragma once

#include <oxebots_interfaces/ssl_gc_common.pb.h>
#include <oxebots_interfaces/ssl_gc_game_event.pb.h>
#include <oxebots_interfaces/ssl_gc_geometry.pb.h>
#include <oxebots_interfaces/ssl_gc_referee_message.pb.h>

#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include <rclcpp/rclcpp.hpp>

#include <vector>

#include "oxebots_comms/udp_driver.hpp"

class GCReceiverNode : public rclcpp::Node, public UdpDriver<Referee>
{
   private:
    boost::asio::io_context & io_context;
    std::thread io_thread;

   public:
    GCReceiverNode(boost::asio::io_context & io_context);

    ~GCReceiverNode();

   protected:
    void on_receive(const Referee & packet) override;
};
