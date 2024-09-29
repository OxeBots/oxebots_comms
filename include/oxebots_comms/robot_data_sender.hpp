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

#include <arpa/inet.h>
#include <oxebots_interfaces/ssl_simulation_robot_control.pb.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdlib>
#include <iostream>
#include <string>

#include <rclcpp/rclcpp.hpp>

class RobotDataSender
{
private:
  int socket_fd;
  struct sockaddr_in server_address;
  std::string host;
  int port;

public:
  RobotDataSender(std::string host, int port);
  ~RobotDataSender();
  void SendControl(RobotControl control);
};
