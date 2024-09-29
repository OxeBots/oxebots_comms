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

#include "oxebots_comms/robot_data_sender.hpp"

RobotDataSender::RobotDataSender(std::string host, int port)
{
  int socket_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (socket_fd < 0) {
    exit(EXIT_FAILURE);
  }

  struct sockaddr_in server_addr;
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(port);
  if (inet_pton(AF_INET, host.c_str(), &server_addr.sin_addr) <= 0) {
    exit(EXIT_FAILURE);
  }

  if (connect(socket_fd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
    exit(EXIT_FAILURE);
  }

  socket_fd = socket_fd;
  host = host;
  port = port;
  server_address = server_addr;
}

RobotDataSender::~RobotDataSender() {close(socket_fd);}

void RobotDataSender::SendControl(RobotControl control)
{
  std::string data_pkt;
  if (!control.SerializeToString(&data_pkt)) {
    exit(EXIT_FAILURE);
  }

  if (send(socket_fd, data_pkt.c_str(), data_pkt.size(), 0) < 0) {
    exit(EXIT_FAILURE);
  }
}
