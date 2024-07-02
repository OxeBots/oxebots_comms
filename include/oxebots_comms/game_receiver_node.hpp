#pragma once
#include <boost/asio.hpp>
#include <rclcpp/rclcpp.hpp>

#include <thread>
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

    std::thread udp_thread;
    boost::asio::io_service io_service;
    boost::asio::ip::udp::socket socket;

    void PublishData(
      std::vector<oxebots_interfaces::msg::RobotGameData> allies,
      std::vector<oxebots_interfaces::msg::RobotGameData> enemies,
      oxebots_interfaces::msg::BallPosition ball_data);
    void GetDetectionData();
    bool is_yellow_team;

   public:
    GameReceiverNode();

    ~GameReceiverNode();
};
