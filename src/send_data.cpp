#include "./headers/send_data.hpp"

SendData::SendData(std::string host, std::string port)
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

    this->socket_fd = socket_fd;
    this->host = host;
    this->port = port;
    this->server_address = server_addr;
}

SendData::~SendData() { close(this->socket_fd); }

void SendData::SendControl(RobotControl control)
{
    std::string control_str;
    if (!control.SerializeToString(&control_str))
    {
        exit(EXIT_FAILURE);
    }

    if (send(this->socket_fd, control_str.c_str(), control_str.size(), 0) < 0)
    {
        exit(EXIT_FAILURE);
    }
}
