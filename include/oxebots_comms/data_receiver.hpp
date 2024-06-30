#pragma once
#include <netinet/in.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdlib>
#include <string>

#define MAX_BUFFER_SIZE 2048

template <typename T>
class DataReceiver
{
   private:
    std::string host;
    std::string port;
    int socket_fd;

   public:
    DataReceiver(std::string host, std::string port);
    ~DataReceiver();
    T receive_data();
};

template <typename T>DataReceiver<T>::DataReceiver(std::string host, std::string port)
{
    int socket_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    int opt = 1;

    if (socket_fd < 0)
    {
        perror("Error creating socket");
        exit(EXIT_FAILURE);
    }

    struct sockaddr_in server_address;
    server_address.sin_family = AF_INET;
    server_address.sin_port = htons(stoi(port));

    if (setsockopt(socket_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt)) < 0)
    {
        perror("Error setting socket options");
        exit(EXIT_FAILURE);
    }

    server_address.sin_addr.s_addr = INADDR_ANY;

    if (bind(socket_fd, (struct sockaddr *)&server_address, sizeof(server_address)) < 0)
    {
        perror("Error binding socket");
        exit(EXIT_FAILURE);
    }

    this->host = host;
    this->port = port;
    this->socket_fd = socket_fd;
}

template <typename T>DataReceiver<T>::~DataReceiver()
{
    close(socket_fd);
}

template <typename T>T DataReceiver<T>::receive_data()
{
    T data;
    char buffer[MAX_BUFFER_SIZE] = {0};
    if (recv(socket_fd, buffer, MAX_BUFFER_SIZE, 0) < 0)
    {
        perror("Error receiving data");
        exit(EXIT_FAILURE);
    }

    data.ParseFromArray(buffer, MAX_BUFFER_SIZE);
    return data;
}
