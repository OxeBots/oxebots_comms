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

#include <ssl_simulation_robot_control.pb.h>

#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "oxebots_interfaces/msg/robot_cmd.hpp"
#include "oxebots_interfaces/msg/robot_cmd_data.hpp"
#include "robot_data_sender.hpp"

class Comms : public rclcpp::Node
{
private:
  RobotDataSender * data_sender;
  std::vector<RobotCommand> proto_command_list;
  int robot_list_size;

  rclcpp::Subscription<oxebots_interfaces::msg::RobotCmd>::SharedPtr cmd_sub;

  void HandleCmd(const oxebots_interfaces::msg::RobotCmd::SharedPtr msg);

public:
  Comms();
};
