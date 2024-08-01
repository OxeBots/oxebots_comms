#pragma once

#include <oxebots_interfaces/ssl_gc_common.pb.h>
#include <oxebots_interfaces/ssl_gc_game_event.pb.h>
#include <oxebots_interfaces/ssl_gc_geometry.pb.h>
#include <oxebots_interfaces/ssl_gc_referee_message.pb.h>
#include <oxebots_interfaces/ssl_gc_game_event.pb.h>

#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include <oxebots_interfaces/msg/game_event_proposal_group.hpp>
#include <rclcpp/rclcpp.hpp>

#include <vector>

#include "oxebots_comms/udp_driver.hpp"

class GCReceiverNode : public rclcpp::Node, public UdpDriver<Referee>
{
   private:
    boost::asio::io_context & io_context;
    std::thread io_thread;

    rclcpp::Publisher<
      oxebots_interfaces::msg::GameEventProposalGroup>::SharedPtr gc_publisher;

    void PublishGCData(
      oxebots_interfaces::msg::GameEventProposalGroup gc_event_group);

   public:
    GCReceiverNode(boost::asio::io_context & io_context);

    ~GCReceiverNode();

   protected:
    void on_receive(const Referee & packet) override;
};
