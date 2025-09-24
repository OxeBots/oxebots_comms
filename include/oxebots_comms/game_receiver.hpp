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

#include <oxebots_interfaces/messages_robocup_ssl_detection.pb.h>
#include <oxebots_interfaces/messages_robocup_ssl_wrapper.pb.h>

#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>
#include <vector>

#include "oxebots_comms/udp_receiver.hpp"
#include "oxebots_interfaces/msg/ball_position.hpp"
#include "oxebots_interfaces/msg/robot_game_data.hpp"
#include "oxebots_interfaces/msg/robot_position.hpp"

class GameReceiver : public rclcpp::Node, public UdpReceiver<SSL_WrapperPacket>
{
   private:
    rclcpp::Publisher<oxebots_interfaces::msg::RobotPosition>::SharedPtr robot_publisher;

    rclcpp::Publisher<oxebots_interfaces::msg::BallPosition>::SharedPtr ball_publisher;

    bool is_yellow_team;

   public:
    GameReceiver();

    ~GameReceiver();

   protected:
    void on_receive(const SSL_WrapperPacket & packet) override;

   private:
    void PublishRobotData(std::vector<oxebots_interfaces::msg::RobotGameData> allies,
                          std::vector<oxebots_interfaces::msg::RobotGameData> enemies);

    void PublishBallData(oxebots_interfaces::msg::BallPosition ball_data);
};
