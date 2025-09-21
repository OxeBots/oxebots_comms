#include "oxebots_comms/robot_data_sender.hpp"

RobotDataSender::RobotDataSender(std::string host, int port)
{
    this->socket_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (this->socket_fd < 0) {
        exit(EXIT_FAILURE);
    }
    memset(&this->server_address, 0, sizeof(this->server_address));
    this->server_address.sin_family = AF_INET;
    this->server_address.sin_port = htons(port);
    if (inet_pton(AF_INET, host.c_str(), &this->server_address.sin_addr) <= 0)
    {
        exit(EXIT_FAILURE);
    }
    if (connect(this->socket_fd, (struct sockaddr *)&this->server_address, sizeof(this->server_address)) < 0) {
        exit(EXIT_FAILURE);
    }
}

RobotDataSender::~RobotDataSender() { close(socket_fd); }

bool  RobotDataSender::SendControl(RobotControl control)
{
    std::string data_pkt;
    if (!control.SerializeToString(&data_pkt))
    {
        return false;
    }
    if (send(this->socket_fd, data_pkt.c_str(), data_pkt.size(), 0) < 0) {
        return false;
    }
    return true;
}
