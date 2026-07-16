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

#include <rclcpp/rclcpp.hpp>

#include <string>
#include <vector>

#include "oxebots_comms/udp_sender.hpp"
#include "oxebots_interfaces/msg/robot_cmd.hpp"
#include "oxebots_interfaces/msg/game_data.hpp"
#include "oxebots_interfaces/ssl_simulation_robot_control.pb.h"

class GrSimController : public rclcpp::Node
{
   private:
    struct RobotState {
        float x = 0.0f;
        float y = 0.0f;
        float orientation = 0.0f;
    };

    std::unique_ptr<UdpSender<RobotControl>> udp_sender_;
    rclcpp::Subscription<oxebots_interfaces::msg::RobotCmd>::SharedPtr command_subscription_;
    rclcpp::Subscription<oxebots_interfaces::msg::GameData>::SharedPtr game_data_subscription_;
    bool is_yellow_team_;
    std::map<uint32_t, RobotState> robot_states_;

    void clampVelocities(uint32_t robot_id, float& vx, float& vy);

   public:
    GrSimController();

    ~GrSimController();

   private:
    void command_callback(const oxebots_interfaces::msg::RobotCmd::SharedPtr msg);
    void game_data_callback(const oxebots_interfaces::msg::GameData::SharedPtr msg);
};
