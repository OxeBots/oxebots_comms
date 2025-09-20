#pragma once
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>

#include <cstdlib>
#include <iostream>
#include <string>

#include "oxebots_interfaces/ssl_simulation_robot_control.pb.h"

class RobotDataSender
{
   private:
    int socket_fd;
    std::string host;
    int port;

    public:
    struct sockaddr_in server_address;
    RobotDataSender(std::string host, int port);
    ~RobotDataSender();
    bool SendControl(RobotControl control);
};
