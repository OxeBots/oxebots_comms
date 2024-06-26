#pragma once
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>

#include <cstdlib>
#include <iostream>
#include <string>

#include "ssl_simulation_robot_control.pb.h"

class SendData
{
   private:
    int socket_fd;
    struct sockaddr_in server_address;
    std::string host;
    std::string port;

   public:
    SendData(std::string host, std::string port);
    ~SendData();
    void SendControl(RobotControl control);
};
