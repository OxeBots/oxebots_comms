#include "oxebots_comms/robot_data_sender.hpp"

RobotDataSender::RobotDataSender(std::string host, std::string port)
{
    int socket_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (socket_fd < 0)
    {
        exit(EXIT_FAILURE);
    }

    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(atoi(port.c_str()));
    if (inet_pton(AF_INET, host.c_str(), &server_addr.sin_addr) <= 0)
    {
        exit(EXIT_FAILURE);
    }

    if (connect(socket_fd, (struct sockaddr *)&server_addr,
                sizeof(server_addr)) < 0)
    {
        exit(EXIT_FAILURE);
    }

    socket_fd = socket_fd;
    host = host;
    port = port;
    server_address = server_addr;
}

RobotDataSender::~RobotDataSender() { close(socket_fd); }

void RobotDataSender::SendControl(RobotControl control)
{
    std::string data_pkt;
    if (!control.SerializeToString(&data_pkt))
    {
        exit(EXIT_FAILURE);
    }

    if (send(socket_fd, data_pkt.c_str(), data_pkt.size(), 0) < 0)
    {
        exit(EXIT_FAILURE);
    }
}
