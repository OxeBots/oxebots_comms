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

#include <geometry_msgs/msg/point32.h>
#include <oxebots_interfaces/ssl_gc_common.pb.h>
#include <oxebots_interfaces/ssl_gc_game_event.pb.h>
#include <oxebots_interfaces/ssl_gc_geometry.pb.h>
#include <oxebots_interfaces/ssl_gc_referee_message.pb.h>

#include <vector>

#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include <rclcpp/rclcpp.hpp>

#include "oxebots_comms/udp_driver.hpp"
#include "oxebots_interfaces/msg/game_event_proposal_group.hpp"
#include "oxebots_interfaces/msg/referee.hpp"

class GCReceiver : public rclcpp::Node, public UdpDriver<Referee>
{
private:
  rclcpp::Publisher<oxebots_interfaces::msg::Referee>::SharedPtr gc_publisher;

  void PublishGCData(oxebots_interfaces::msg::Referee gc_referee);

  oxebots_interfaces::msg::TeamInfo get_team_info(const Referee_TeamInfo & team);

  oxebots_interfaces::msg::GameEvent get_game_event(const GameEvent & event);

  void get_ball_left_field(
    const GameEvent_BallLeftField & ball_left_field,
    oxebots_interfaces::msg::GameEvent & game_event);

  oxebots_interfaces::msg::Vector2f get_vector2(const Vector2 & vector);

public:
  GCReceiver();

  ~GCReceiver();

protected:
  void on_receive(const Referee & packet) override;
};
