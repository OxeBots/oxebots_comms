#pragma once
#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include <rclcpp/rclcpp.hpp>

#include <vector>

#include "oxebots_interfaces/msg/ball_position.hpp"
#include "oxebots_interfaces/msg/robot_game_data.hpp"
#include "oxebots_interfaces/msg/robot_position.hpp"
#include "oxebots_interfaces/ssl_vision_detection.pb.h"
#include "oxebots_interfaces/ssl_vision_wrapper.pb.h"
class GameReceiverNode : public rclcpp::Node
{
   private:
    rclcpp::Publisher<oxebots_interfaces::msg::RobotPosition>::SharedPtr
      robot_publisher;
    rclcpp::Publisher<oxebots_interfaces::msg::BallPosition>::SharedPtr
      ball_publisher;

    boost::asio::io_service io_service;
    boost::asio::ip::udp::socket socket;
    std::array<char, 1024> recv_buffer;
    boost::asio::ip::udp::endpoint endpoint;

    void PublishRobotData(
      std::vector<oxebots_interfaces::msg::RobotGameData> allies,
      std::vector<oxebots_interfaces::msg::RobotGameData> enemies);
    void PublishBallData(oxebots_interfaces::msg::BallPosition ball_data);

    void GetDetectionData(const boost::system::error_code & error,
                          std::size_t bytes_transferred);
    bool is_yellow_team;

    void StartReceive();

   public:
    GameReceiverNode();

    ~GameReceiverNode();
};
