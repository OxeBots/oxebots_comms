// Copyright 2024 Oxebots
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include "oxebots_comms/game_receiver.hpp"

GameReceiver::GameReceiver()
: rclcpp::Node("oxebots_comms"),
  UdpDriver<SSL_WrapperPacket>(
    declare_parameter<std::string>("ssl_vision_ip", "224.5.23.2"),
    declare_parameter<uint16_t>("ssl_vision_port", 10006),
    declare_parameter<std::string>("interface_ip", ""))
{
  RCLCPP_INFO(get_logger(), "Starting game receiver module...");

  declare_parameter("robot_topic", "robot_data");
  declare_parameter("ball_topic", "ball_data");
  declare_parameter("topic_retention", 10);
  declare_parameter("is_yellow_team", false);

  robot_publisher = create_publisher<oxebots_interfaces::msg::RobotPosition>(
    get_parameter("robot_topic").as_string(), get_parameter("topic_retention").as_int());

  ball_publisher = create_publisher<oxebots_interfaces::msg::BallPosition>(
    get_parameter("ball_topic").as_string(), get_parameter("topic_retention").as_int());

  is_yellow_team = get_parameter("is_yellow_team").as_bool();

  RCLCPP_INFO(get_logger(), "Game receiver module started");
}

GameReceiver::~GameReceiver() {RCLCPP_INFO(get_logger(), "Stopping game receiver module...");}

void GameReceiver::on_receive(const SSL_WrapperPacket & packet)
{
  RCLCPP_DEBUG(get_logger(), "Received packet");
  SSL_DetectionFrame detection = packet.detection();
  std::vector<oxebots_interfaces::msg::RobotGameData> yellow_robots;
  std::vector<oxebots_interfaces::msg::RobotGameData> blue_robots;

  RCLCPP_DEBUG(get_logger(), "Parsing frame");
  RCLCPP_DEBUG(get_logger(), "Frame number: %d", detection.frame_number());
  RCLCPP_DEBUG(get_logger(), "Camera ID: %d", detection.camera_id());

  RCLCPP_DEBUG(get_logger(), "Parsing packet");
  RCLCPP_DEBUG(get_logger(), "Yellow robots: %d", detection.robots_yellow_size());
  if (detection.robots_yellow_size() > 0) {
    for (auto robot : detection.robots_yellow()) {
      oxebots_interfaces::msg::RobotGameData robot_data;
      robot_data.id = robot.robot_id();
      robot_data.x = robot.x();
      robot_data.y = robot.y();
      robot_data.orientation = robot.orientation();
      yellow_robots.push_back(robot_data);
    }
  }

  RCLCPP_DEBUG(get_logger(), "Blue robots: %d", detection.robots_blue_size());
  if (detection.robots_blue_size() > 0) {
    for (auto robot : detection.robots_blue()) {
      oxebots_interfaces::msg::RobotGameData robot_data;
      robot_data.id = robot.robot_id();
      robot_data.x = robot.x();
      robot_data.y = robot.y();
      robot_data.orientation = robot.orientation();
      blue_robots.push_back(robot_data);
    }
  }

  RCLCPP_DEBUG(get_logger(), "Parsing ball");
  if (detection.balls_size() > 0) {
    oxebots_interfaces::msg::BallPosition ball_data;
    ball_data.x = detection.balls(0).x();
    ball_data.y = detection.balls(0).y();
    ball_data.z = detection.balls(0).z();
    PublishBallData(ball_data);
  }

  RCLCPP_DEBUG(get_logger(), "Publishing data");
  if (is_yellow_team) {
    PublishRobotData(yellow_robots, blue_robots);
  } else {
    PublishRobotData(blue_robots, yellow_robots);
  }
}

void GameReceiver::PublishRobotData(
  std::vector<oxebots_interfaces::msg::RobotGameData> allies,
  std::vector<oxebots_interfaces::msg::RobotGameData> enemies)
{
  oxebots_interfaces::msg::RobotPosition robot_data;
  robot_data.allies = allies;
  robot_data.enemies = enemies;

  robot_publisher->publish(robot_data);
}

void GameReceiver::PublishBallData(oxebots_interfaces::msg::BallPosition ball_data)
{
  ball_publisher->publish(ball_data);
}
