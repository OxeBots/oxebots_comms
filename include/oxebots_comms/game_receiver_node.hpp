#pragma once

#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include <rclcpp/rclcpp.hpp>

#include <vector>

#include "oxebots_comms/udp_driver.hpp"
#include "oxebots_interfaces/msg/ball_position.hpp"
#include "oxebots_interfaces/msg/robot_game_data.hpp"
#include "oxebots_interfaces/msg/robot_position.hpp"
#include "oxebots_interfaces/ssl_vision_detection.pb.h"
#include "oxebots_interfaces/ssl_vision_wrapper.pb.h"

class GameReceiverNode : public rclcpp::Node,
                         public UdpDriver<SSL_WrapperPacket>
{
   private:
    rclcpp::Publisher<oxebots_interfaces::msg::RobotPosition>::SharedPtr
      robot_publisher;

    rclcpp::Publisher<oxebots_interfaces::msg::BallPosition>::SharedPtr
      ball_publisher;

    bool is_yellow_team;

    void StartReceive();

   public:
    GameReceiverNode(boost::asio::io_context & io_context);

    ~GameReceiverNode();

   protected:
    void on_receive(const SSL_WrapperPacket & packet) override;

   private:
    void PublishData(
      std::vector<oxebots_interfaces::msg::RobotGameData> allies,
      std::vector<oxebots_interfaces::msg::RobotGameData> enemies,
      oxebots_interfaces::msg::BallPosition ball_data);
};
