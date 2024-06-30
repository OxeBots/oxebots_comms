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
    struct sockaddr_in server_address;
    std::string host;
    std::string port;

   public:
    RobotDataSender(std::string host, std::string port);
    ~RobotDataSender();
    void SendControl(RobotControl control);
};
