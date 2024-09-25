#pragma once

#include <geometry_msgs/msg/point32.h>
#include <oxebots_interfaces/ssl_gc_common.pb.h>
#include <oxebots_interfaces/ssl_gc_game_event.pb.h>
#include <oxebots_interfaces/ssl_gc_geometry.pb.h>
#include <oxebots_interfaces/ssl_gc_referee_message.pb.h>

#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include <oxebots_interfaces/msg/game_event_proposal_group.hpp>
#include <oxebots_interfaces/msg/referee.hpp>
#include <rclcpp/rclcpp.hpp>

#include <vector>

#include "oxebots_comms/udp_driver.hpp"

class GCReceiver : public rclcpp::Node, public UdpDriver<Referee>
{
   private:
    boost::asio::io_context & io_context;
    std::thread io_thread;

    rclcpp::Publisher<oxebots_interfaces::msg::Referee>::SharedPtr
      gc_publisher;

    void PublishGCData(oxebots_interfaces::msg::Referee gc_referee);

    oxebots_interfaces::msg::TeamInfo get_team_info(
      const Referee_TeamInfo & team);

    oxebots_interfaces::msg::GameEvent get_game_event(const GameEvent & event);

    void get_ball_left_field(const GameEvent_BallLeftField & ball_left_field,
                             oxebots_interfaces::msg::GameEvent & game_event);

    oxebots_interfaces::msg::Vector2 get_vector2(const Vector2 & vector);

   public:
    GCReceiver(boost::asio::io_context & io_context);

    ~GCReceiver();

   protected:
    void on_receive(const Referee & packet) override;
};
