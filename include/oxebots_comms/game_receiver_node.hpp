#pragma once
#include <rclcpp/rclcpp.hpp>

#include <string>
#include <vector>

#include "data_receiver.hpp"
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

    DataReceiver<SSL_WrapperPacket> * game_receiver;

    void PublishData(
      std::vector<oxebots_interfaces::msg::RobotGameData> allies,
      std::vector<oxebots_interfaces::msg::RobotGameData> enemies,
      oxebots_interfaces::msg::BallPosition ball_data);
    void GetDetectionData();
    bool is_yellow_team;

   public:
    GameReceiverNode();
};
